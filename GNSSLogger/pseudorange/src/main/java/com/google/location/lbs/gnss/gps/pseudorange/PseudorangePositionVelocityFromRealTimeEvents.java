/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.location.lbs.gnss.gps.pseudorange;

import android.location.GnssClock;
import android.location.GnssMeasurement;
import android.location.GnssMeasurementsEvent;
import android.location.GnssNavigationMessage;
import android.location.GnssStatus;
import android.util.Log;
import com.google.location.lbs.gnss.gps.pseudorange.Ecef2EnuConverter.EnuValues;
import com.google.location.lbs.gnss.gps.pseudorange.Ecef2LlaConverter.GeodeticLlaValues;
//import android.location.cts.nano.Ephemeris.GpsEphemerisProto;
//import android.location.cts.nano.Ephemeris.GpsNavMessageProto;
//import android.location.cts.suplClient.SuplRrlpController;

import java.io.BufferedReader;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.List;
import java.util.concurrent.TimeUnit;


import com.google.location.suplclient.ephemeris.EphemerisResponse;
import com.google.location.suplclient.ephemeris.GalEphemeris;
import com.google.location.suplclient.ephemeris.GloEphemeris;
import com.google.location.suplclient.ephemeris.GnssEphemeris;
import com.google.location.suplclient.ephemeris.GpsEphemeris;
import com.google.location.suplclient.supl.SuplConnectionRequest;
import com.google.location.suplclient.supl.SuplController;
import com.google.location.suplclient.supl.SuplTester;

import static java.lang.System.currentTimeMillis;

/**
 * Helper class for calculating Gps position and velocity solution using weighted least squares
 * where the raw Gps measurements are parsed as a {@link BufferedReader} with the option to apply
 * doppler smoothing, carrier phase smoothing or no smoothing.
 *
 */
public class PseudorangePositionVelocityFromRealTimeEvents {

  private static final String TAG = "PseudorangePositionVelocityFromRealTimeEvents";
  private static final double SECONDS_PER_NANO = 1.0e-9;
  private static final int TOW_DECODED_MEASUREMENT_STATE_BIT = 3;
  /** Average signal travel time from GPS satellite and earth */
  private static final int VALID_ACCUMULATED_DELTA_RANGE_STATE = 1;
  private static final int MINIMUM_NUMBER_OF_USEFUL_SATELLITES = 6;
  private static final int C_TO_N0_THRESHOLD_DB_HZ = 18;

  private static final String SUPL_SERVER_NAME = "supl.google.com";
  private static final int SUPL_SERVER_PORT = 7275;

//  private GpsNavMessageProto mHardwareGpsNavMessageProto = null;

  // navigation message parser
  private GpsNavigationMessageStore mGpsNavigationMessageStore = new GpsNavigationMessageStore();
  private double[] mPositionSolutionLatLngDeg = GpsMathOperations.createAndFillArray(3, Double.NaN);
  private double[] mVelocitySolutionEnuMps = GpsMathOperations.createAndFillArray(3, Double.NaN);
  private final double[] mPositionVelocityUncertaintyEnu
      = GpsMathOperations.createAndFillArray(6, Double.NaN);
  private double[] mPseudorangeResidualsMeters =
      GpsMathOperations.createAndFillArray(
          GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES, Double.NaN
      );
  private boolean mFirstUsefulMeasurementSet = true;
  private int[] mReferenceLocation = null;
  private long mLastReceivedSuplMessageTimeMillis = 0;
  private long mDeltaTimeMillisToMakeSuplRequest = TimeUnit.MINUTES.toMillis(30);
  private boolean mFirstSuplRequestNeeded = true;
  private EphemerisResponse ephemerisResponse = null;
  private int successCount = 0;

  // Only the interface of pseudorange smoother is provided. Please implement customized smoother.
  PseudorangeSmoother mPseudorangeSmoother = new PseudorangeNoSmoothingSmoother();
  private final UserPositionVelocityWeightedLeastSquare mUserPositionVelocityLeastSquareCalculator =
      new UserPositionVelocityWeightedLeastSquare(mPseudorangeSmoother);
  private GpsMeasurement[] mUsefulSatellitesToReceiverMeasurements =
      new GpsMeasurement[GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES];
  private Long[] mUsefulSatellitesToTowNs =
      new Long[GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES];
  private long mLargestTowNs = Long.MIN_VALUE;
  private double mArrivalTimeSinceGPSWeekNs = 0.0;
  private int mDayOfYear1To366 = 0;
  private int mGpsWeekNumber = 0;
  private long mArrivalTimeSinceGpsEpochNs = 0;

  private GnssInsideLogger gnssInsideLogger;
  public PseudorangePositionVelocityFromRealTimeEvents(GnssInsideLogger gnssInsideLogger){
    this.gnssInsideLogger = gnssInsideLogger;
  }
  /**
   * Computes Weighted least square position and velocity solutions from a received {@link
   * GnssMeasurementsEvent} and store the result in {@link
   * PseudorangePositionVelocityFromRealTimeEvents#mPositionSolutionLatLngDeg} and {@link
   * PseudorangePositionVelocityFromRealTimeEvents#mVelocitySolutionEnuMps}
   */
  public void computePositionVelocitySolutionsFromRawMeas(GnssMeasurementsEvent event, IonoConfig ionoConfig)
      throws Exception {
    Log.w(TAG, "getSolutionsFromRawMeas called");
    if (mReferenceLocation == null) {
      // If no reference location is received, we can not get navigation message from SUPL and hence
      // we will not try to compute location.
      Log.d(TAG, " No reference Location ..... no position is calculated");
      return;
    }
    for (int i = 0; i < GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES; i++) {
      mUsefulSatellitesToReceiverMeasurements[i] = null;
      mUsefulSatellitesToTowNs[i] = null;
    }

    GnssClock gnssClock = event.getClock();
    mArrivalTimeSinceGpsEpochNs = gnssClock.getTimeNanos() - gnssClock.getFullBiasNanos();

    for (GnssMeasurement measurement : event.getMeasurements()) {
      // ignore any measurement if it is not from GPS constellation
      if (measurement.getConstellationType() != GnssStatus.CONSTELLATION_GPS
      && measurement.getConstellationType() != GnssStatus.CONSTELLATION_GALILEO) {
        continue;
      }
      // ignore raw data if time is zero, if signal to noise ratio is below threshold or if
      // TOW is not yet decoded
      if (measurement.getCn0DbHz() >= C_TO_N0_THRESHOLD_DB_HZ
          && (measurement.getState() & (1L << TOW_DECODED_MEASUREMENT_STATE_BIT)) != 0) {

        // calculate day of year and Gps week number needed for the least square
        GpsTime gpsTime = new GpsTime(mArrivalTimeSinceGpsEpochNs);
        // Gps weekly epoch in Nanoseconds: defined as of every Sunday night at 00:00:000
        long gpsWeekEpochNs = GpsTime.getGpsWeekEpochNano(gpsTime);
        mArrivalTimeSinceGPSWeekNs = mArrivalTimeSinceGpsEpochNs - gpsWeekEpochNs;
        mGpsWeekNumber = gpsTime.getGpsWeekSecond().first;
        // calculate day of the year between 1 and 366
        Calendar cal = gpsTime.getTimeInCalendar();
        mDayOfYear1To366 = cal.get(Calendar.DAY_OF_YEAR);

        long receivedGPSTowNs = measurement.getReceivedSvTimeNanos();
        if (receivedGPSTowNs > mLargestTowNs) {
          mLargestTowNs = receivedGPSTowNs;
        }

        int n = getConstellationIndexPadding(measurement.getConstellationType());
        mUsefulSatellitesToTowNs[n + measurement.getSvid() - 1] = receivedGPSTowNs;
        GpsMeasurement gpsReceiverMeasurement =
            new GpsMeasurement(
                (long) mArrivalTimeSinceGPSWeekNs,
                measurement.getAccumulatedDeltaRangeMeters(),
                measurement.getAccumulatedDeltaRangeState() == VALID_ACCUMULATED_DELTA_RANGE_STATE,
                measurement.getPseudorangeRateMetersPerSecond(),
                measurement.getCn0DbHz(),
                measurement.getAccumulatedDeltaRangeUncertaintyMeters(),
                measurement.getPseudorangeRateUncertaintyMetersPerSecond(),
                measurement.getConstellationType(),
                measurement.getReceivedSvTimeNanos());
        mUsefulSatellitesToReceiverMeasurements[n + measurement.getSvid() - 1] = gpsReceiverMeasurement;
      }
    }

    // check if we should continue using the navigation message from the SUPL server, or use the
    // navigation message from the device if we fully received it
    boolean useNavMessageFromSupl = true;
    if (useNavMessageFromSupl) {
      Log.d(TAG, "Using navigation message from SUPL server");

      if (mFirstSuplRequestNeeded
          || (currentTimeMillis() - mLastReceivedSuplMessageTimeMillis)
              > mDeltaTimeMillisToMakeSuplRequest) {
        // The following line is blocking call for SUPL connection and back. But it is fast enough
        ephemerisResponse = getSuplNavMessage(mReferenceLocation[0], mReferenceLocation[1]);
        if (!isEmptyNavMessage(ephemerisResponse)) {
          mFirstSuplRequestNeeded = false;
          mLastReceivedSuplMessageTimeMillis = currentTimeMillis();
        } else {
          return;
        }
      }

    }

    // some times the SUPL server returns less satellites than the visible ones, so remove those
    // visible satellites that are not returned by SUPL
    for (int i = 0; i < GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES; i++) {
      if (mUsefulSatellitesToReceiverMeasurements[i] != null){
        int n = getConstellationIndexPadding(mUsefulSatellitesToReceiverMeasurements[i].constellationType);
        if (!navMessageProtoContainsSvid(ephemerisResponse, i + 1 - n,
                mUsefulSatellitesToReceiverMeasurements[i].constellationType)) {
          mUsefulSatellitesToReceiverMeasurements[i] = null;
          mUsefulSatellitesToTowNs[i] = null;
        }
      }
    }


    // calculate the number of useful satellites
    int numberOfUsefulSatellites = 0;
    for (GpsMeasurement element : mUsefulSatellitesToReceiverMeasurements) {
      if (element != null) {
        numberOfUsefulSatellites++;
      }
    }
    if (numberOfUsefulSatellites < MINIMUM_NUMBER_OF_USEFUL_SATELLITES) Log.d(TAG, "Less than 6 satelllites detected!\n");
    if (numberOfUsefulSatellites >= MINIMUM_NUMBER_OF_USEFUL_SATELLITES) {
      // ignore first set of > 4 satellites as they often result in erroneous position
      if (!mFirstUsefulMeasurementSet) {
        // start with last known position and velocity of zero. Following the structure:
        // [X position, Y position, Z position, clock bias gps,
        //  X Velocity, Y Velocity, Z Velocity, clock bias rate, clock bias galileo]
        double[] positionVelocitySolutionEcef = GpsMathOperations.createAndFillArray(9, 0);
        double[] positionVelocityUncertaintyEnu = GpsMathOperations.createAndFillArray(6, 0);
        double[] pseudorangeResidualMeters
            = GpsMathOperations.createAndFillArray(
                GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES, Double.NaN
        );
        performPositionVelocityComputationEcef(
            mUserPositionVelocityLeastSquareCalculator,
            mUsefulSatellitesToReceiverMeasurements,
            mUsefulSatellitesToTowNs,
            mLargestTowNs,
            mArrivalTimeSinceGPSWeekNs,
            mDayOfYear1To366,
            mGpsWeekNumber,
            positionVelocitySolutionEcef,
            positionVelocityUncertaintyEnu,
            pseudorangeResidualMeters,
            ionoConfig);
        // convert the position solution from ECEF to latitude, longitude and altitude
        GeodeticLlaValues latLngAlt =
            Ecef2LlaConverter.convertECEFToLLACloseForm(
                positionVelocitySolutionEcef[0],
                positionVelocitySolutionEcef[1],
                positionVelocitySolutionEcef[2]);
        mPositionSolutionLatLngDeg[0] = Math.toDegrees(latLngAlt.latitudeRadians);
        mPositionSolutionLatLngDeg[1] = Math.toDegrees(latLngAlt.longitudeRadians);
        mPositionSolutionLatLngDeg[2] = latLngAlt.altitudeMeters;
        mPositionVelocityUncertaintyEnu[0] = positionVelocityUncertaintyEnu[0];
        mPositionVelocityUncertaintyEnu[1] = positionVelocityUncertaintyEnu[1];
        mPositionVelocityUncertaintyEnu[2] = positionVelocityUncertaintyEnu[2];
        System.arraycopy(
            pseudorangeResidualMeters,
            0 /*source starting pos*/,
            mPseudorangeResidualsMeters,
            0 /*destination starting pos*/,
            GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES /*length of elements*/
            );
        Log.d(TAG,
            "Position Uncertainty ENU Meters :"
                + mPositionVelocityUncertaintyEnu[0]
                + " "
                + mPositionVelocityUncertaintyEnu[1]
                + " "
                + mPositionVelocityUncertaintyEnu[2]);
        successCount++;
        if(ionoConfig==IonoConfig.IONO_NEQUICK){
          Log.d(TAG, "Latitude NeQuick");
          String logStr = Double.toString(mPositionSolutionLatLngDeg[0]) +
                  ";" + Double.toString(mPositionSolutionLatLngDeg[1]) +
                  ";" + Double.toString(mPositionSolutionLatLngDeg[2]) +
                  ";" + Integer.toString(successCount) +
                  ";" + Integer.toString(numberOfUsefulSatellites) +
                  ";" + currentTimeMillis() + "\n";
          gnssInsideLogger.appendLog(logStr);
          Log.d(
                  TAG,
                  "Latitude NeQuick, Longitude, Altitude: "
                          + mPositionSolutionLatLngDeg[0]
                          + " "
                          + mPositionSolutionLatLngDeg[1]
                          + " "
                          + mPositionSolutionLatLngDeg[2]);

        }else if(ionoConfig==IonoConfig.IONO_KLOBUCHAR){
          String logStr = Double.toString(mPositionSolutionLatLngDeg[0]) +
                  ";" + Double.toString(mPositionSolutionLatLngDeg[1]) +
                  ";" + Double.toString(mPositionSolutionLatLngDeg[2]) +
                  ";" + Integer.toString(successCount) +
                  ";" + Integer.toString(numberOfUsefulSatellites) +
                  ";" + currentTimeMillis() + "\n";
          gnssInsideLogger.appendLog(logStr);
          Log.d(TAG, "Latitude Klobuchar");
          Log.d(
                  TAG,
                  "Latitude Klobuchar, Longitude, Altitude: "
                          + mPositionSolutionLatLngDeg[0]
                          + " "
                          + mPositionSolutionLatLngDeg[1]
                          + " "
                          + mPositionSolutionLatLngDeg[2]);

        }
        EnuValues velocityEnu = Ecef2EnuConverter.convertEcefToEnu(
            positionVelocitySolutionEcef[4],
            positionVelocitySolutionEcef[5],
            positionVelocitySolutionEcef[6],
            latLngAlt.latitudeRadians,
            latLngAlt.longitudeRadians
        );

        mVelocitySolutionEnuMps[0] = velocityEnu.enuEast;
        mVelocitySolutionEnuMps[1] = velocityEnu.enuNorth;
        mVelocitySolutionEnuMps[2] = velocityEnu.enuUP;
        Log.d(
            TAG,
            "Velocity ENU Mps: "
                + mVelocitySolutionEnuMps[0]
                + " "
                + mVelocitySolutionEnuMps[1]
                + " "
                + mVelocitySolutionEnuMps[2]);
        mPositionVelocityUncertaintyEnu[3] = positionVelocityUncertaintyEnu[3];
        mPositionVelocityUncertaintyEnu[4] = positionVelocityUncertaintyEnu[4];
        mPositionVelocityUncertaintyEnu[5] = positionVelocityUncertaintyEnu[5];
        Log.d(TAG,
            "Velocity Uncertainty ENU Mps :"
                + mPositionVelocityUncertaintyEnu[3]
                + " "
                + mPositionVelocityUncertaintyEnu[4]
                + " "
                + mPositionVelocityUncertaintyEnu[5]);
      }
      mFirstUsefulMeasurementSet = false;
    } else {
      Log.d(
          TAG,
          "Less than four satellites with SNR above threshold visible ... "
              + "no position is calculated!");

      mPositionSolutionLatLngDeg = GpsMathOperations.createAndFillArray(3, Double.NaN);
      mVelocitySolutionEnuMps = GpsMathOperations.createAndFillArray(3, Double.NaN);
      mPseudorangeResidualsMeters =
          GpsMathOperations.createAndFillArray(
              GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES, Double.NaN
          );
    }
  }

  private int getConstellationIndexPadding(int constellationType){
    if(constellationType==GnssStatus.CONSTELLATION_GPS){
      return 0;
    }else if(constellationType==GnssStatus.CONSTELLATION_GALILEO){
      return GpsNavigationMessageStore.MAX_NUMBER_OF_GPS_SATELLITE;
    }
    return 0;
  }

  private boolean isEmptyNavMessage(EphemerisResponse ephemerisResponse) {
    if(ephemerisResponse.ionoProto == null)return true;
    if(ephemerisResponse.ephList.size() == 0)return true;
    return  false;
  }

  private boolean navMessageProtoContainsSvid(EphemerisResponse ephemerisResponse, int svid, int constellationType) {
//    List<GnssEphemeris> ephemeridesList =
//            new ArrayList<GnssEphemeris>(Arrays.asList(ephemerisResponse.ephList));

    for (GnssEphemeris ephProto : ephemerisResponse.ephList) {
      int constelType = GnssStatus.CONSTELLATION_UNKNOWN;
      if(ephProto instanceof GpsEphemeris) constelType = GnssStatus.CONSTELLATION_GPS;
      if(ephProto instanceof GalEphemeris) constelType = GnssStatus.CONSTELLATION_GALILEO;
      if(ephProto instanceof GloEphemeris) constelType = GnssStatus.CONSTELLATION_GLONASS;
      if (constelType == constellationType && ephProto.svid == svid) {
        return true;
      }
    }
    return false;
  }

  /**
   * Calculates ECEF least square position and velocity solutions from an array of {@link
   * GpsMeasurement} in meters and meters per second and store the result in {@code
   * positionVelocitySolutionEcef}
   */
  private void performPositionVelocityComputationEcef(
      UserPositionVelocityWeightedLeastSquare userPositionVelocityLeastSquare,
      GpsMeasurement[] usefulSatellitesToReceiverMeasurements,
      Long[] usefulSatellitesToTOWNs,
      long largestTowNs,
      double arrivalTimeSinceGPSWeekNs,
      int dayOfYear1To366,
      int gpsWeekNumber,
      double[] positionVelocitySolutionEcef,
      double[] positionVelocityUncertaintyEnu,
      double[] pseudorangeResidualMeters,
      IonoConfig ionoConfig)
      throws Exception {

    List<GpsMeasurementWithRangeAndUncertainty> usefulSatellitesToPseudorangeMeasurements =
        UserPositionVelocityWeightedLeastSquare.computePseudorangeAndUncertainties(
            Arrays.asList(usefulSatellitesToReceiverMeasurements),
            usefulSatellitesToTOWNs,
            largestTowNs);

    // calculate iterative least square position solution and velocity solutions
    userPositionVelocityLeastSquare.calculateUserPositionVelocityLeastSquare(
        ephemerisResponse,
        usefulSatellitesToPseudorangeMeasurements,
        arrivalTimeSinceGPSWeekNs * SECONDS_PER_NANO,
        gpsWeekNumber,
        dayOfYear1To366,
        positionVelocitySolutionEcef,
        positionVelocityUncertaintyEnu,
        pseudorangeResidualMeters,
        ionoConfig);

    Log.d(
        TAG,
        "Least Square Position Solution in ECEF meters: "
            + positionVelocitySolutionEcef[0]
            + " "
            + positionVelocitySolutionEcef[1]
            + " "
            + positionVelocitySolutionEcef[2]);
    Log.d(TAG, "Estimated gps Receiver clock offset in meters: " + positionVelocitySolutionEcef[3]);
    Log.d(TAG, "Estimated gal Receiver clock offset in meters: " + positionVelocitySolutionEcef[8]);

    Log.d(
        TAG,
        "Velocity Solution in ECEF Mps: "
            + positionVelocitySolutionEcef[4]
            + " "
            + positionVelocitySolutionEcef[5]
            + " "
            + positionVelocitySolutionEcef[6]);
    Log.d(TAG, "Estimated Receiver clock offset rate in mps: " + positionVelocitySolutionEcef[7]);







  }

  /**
   * Reads the navigation message from the SUPL server by creating a Stubby client to Stubby server
   * that wraps the SUPL server. The input is the time in nanoseconds since the GPS epoch at which
   * the navigation message is required and the output is a {@link EphemerisResponse}
   *
   * @throws IOException
   * @throws UnknownHostException
   */
  private EphemerisResponse getSuplNavMessage(long latE7, long lngE7)
      throws UnknownHostException, IOException {
    SuplConnectionRequest request =
            SuplConnectionRequest.builder()
                    .setServerHost(SUPL_SERVER_NAME)
                    .setServerPort(SUPL_SERVER_PORT)
                    .setSslEnabled(true)
                    .setMessageLoggingEnabled(true)
                    .setLoggingEnabled(true)
                    .build();
    SuplController suplController = new SuplController(request);
    // Try to call methods to access SUPL server and see if they report any exception
    suplController.sendSuplRequest(latE7, lngE7);
    EphemerisResponse ephResponse = suplController.generateEphResponse(latE7, lngE7);
    return ephResponse;
  }

  /**
   * Checks if we should continue using the navigation message from the SUPL server, or use the
   * navigation message from the device if we fully received it. If the navigation message read from
   * the receiver has all the visible satellite ephemerides, return false, otherwise, return true.
   */
//  private static boolean continueUsingNavMessageFromSupl(
//          GpsMeasurement[] usefulSatellitesToReceiverMeasurements,
//          GpsNavMessageProto hardwareGpsNavMessageProto) {
//    boolean useNavMessageFromSupl = true;
//    if (hardwareGpsNavMessageProto != null) {
//      ArrayList<GpsEphemerisProto> hardwareEphemeridesList=
//              new ArrayList<GpsEphemerisProto>(Arrays.asList(hardwareGpsNavMessageProto.ephemerids));
//      if (hardwareGpsNavMessageProto.iono != null) {
//        for (int i = 0; i < GpsNavigationMessageStore.MAX_NUMBER_OF_SATELLITES; i++) {
//          if (usefulSatellitesToReceiverMeasurements[i] != null) {
//            int prn = i + 1;
//            for (GpsEphemerisProto hardwareEphProtoFromList : hardwareEphemeridesList) {
//              if (hardwareEphProtoFromList.prn == prn) {
//                useNavMessageFromSupl = false;
//                break;
//              }
//              useNavMessageFromSupl = true;
//            }
//            if (useNavMessageFromSupl == true) {
//              break;
//            }
//          }
//        }
//      }
//    }
//    return useNavMessageFromSupl;
//  }

  /**
   * Parses a string array containing an updates to the navigation message and return the most
   * recent {@link GnssNavigationMessage}.
   */
  public void parseHwNavigationMessageUpdates(GnssNavigationMessage navigationMessage) {
    //CHANGE TO NOP
    return;


//    byte messagePrn = (byte) navigationMessage.getSvid();
//    byte messageType = (byte) (navigationMessage.getType() >> 8);
//    int subMessageId = navigationMessage.getSubmessageId();
//
//    byte[] messageRawData = navigationMessage.getData();
//    // parse only GPS navigation messages for now
//    if (messageType == 1) {
//      mGpsNavigationMessageStore.onNavMessageReported(
//          messagePrn, messageType, (short) subMessageId, messageRawData);
//      mHardwareGpsNavMessageProto = mGpsNavigationMessageStore.createDecodedNavMessage();
//    }
  }

  /** Sets a rough location of the receiver that can be used to request SUPL assistance data */
  public void setReferencePosition(int latE7, int lngE7, int altE7) {
    if (mReferenceLocation == null) {
      mReferenceLocation = new int[3];
    }
    mReferenceLocation[0] = latE7;
    mReferenceLocation[1] = lngE7;
    mReferenceLocation[2] = altE7;
  }

  /**
   * Converts the input from LLA coordinates to ECEF and set up the reference position of
   * {@code mUserPositionVelocityLeastSquareCalculator} to calculate a corrected residual.
   *
   * <p> Based on this input ground truth, true residuals can be computed. This is done by using
   * the high elevation satellites to compute the true user clock error and with the knowledge of
   * the satellite positions.
   *
   * <p> If no ground truth is set, no residual analysis will be performed.
   */
  public void setCorrectedResidualComputationTruthLocationLla
  (double[] groundTruthLocationLla) {
    if (groundTruthLocationLla == null) {
      mUserPositionVelocityLeastSquareCalculator
          .setTruthLocationForCorrectedResidualComputationEcef(null);
      return;
    }
    GeodeticLlaValues llaValues =
        new GeodeticLlaValues(
            Math.toRadians(groundTruthLocationLla[0]),
            Math.toRadians(groundTruthLocationLla[1]),
            Math.toRadians(groundTruthLocationLla[2]));
    mUserPositionVelocityLeastSquareCalculator.setTruthLocationForCorrectedResidualComputationEcef(
        Lla2EcefConverter.convertFromLlaToEcefMeters(llaValues));
  }

  /** Returns the last computed weighted least square position solution */
  public double[] getPositionSolutionLatLngDeg() {
    return mPositionSolutionLatLngDeg;
  }

  /** Returns the last computed Velocity solution */
  public double[] getVelocitySolutionEnuMps() {
    return mVelocitySolutionEnuMps;
  }

  /**
   * Returns the last computed position and velocity uncertainties in meters and meter per seconds,
   * respectively.
   */
  public double[] getPositionVelocityUncertaintyEnu() {
    return mPositionVelocityUncertaintyEnu;
  }

  /**
   * Returns the pseudorange residuals corrected by using clock bias computed from highest
   * elevationDegree satellites.
   */
  public double[] getPseudorangeResidualsMeters() {
    return mPseudorangeResidualsMeters;
  }
}
