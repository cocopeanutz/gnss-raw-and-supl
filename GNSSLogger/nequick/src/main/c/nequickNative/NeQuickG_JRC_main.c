/**
 * NeQuickG test program
 *
 * @author Angela Aragon-Angel (maria-angeles.aragon@ec.europa.eu)
 * @ingroup NeQuickG_JRC
 * @defgroup NeQuickG_JRC_main NeQuickG_JRC_main [NeQuickG_JRC_main]
 * @copyright Joint Research Centre (JRC), 2019<br>
 *  This software has been released as free and open source software
 *  under the terms of the European Union Public Licence (EUPL), version 1.<br>
 *  Questions? Submit your query at https://www.gsc-europa.eu/contact-us/helpdesk
 * @file
 */
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "NeQuickG_JRC.h"
#include "NeQuickG_JRC_exception.h"
#include "NeQuickG_JRC_input_data_std_stream.h"
#include "NeQuickG_JRC_input_data_stream.h"
#include "NeQuickG_JRC_macros.h"

#define NEQUICK_TEC_EXCEPTION -8
#ifdef FTR_UNIT_TEST
#define NEQUICK_UNIT_TEST_EXCEPTION -10
#endif
#define NEQUICK_G_JRC_TEC_EPSILON (5e-6)

static void to_std_output
  (NeQuickG_chandle nequick,
   double_t total_electron_content) {
  NeQuickG.input_data_to_std_output(nequick);
  printf(", TEC =  %12.5f\n", total_electron_content);
}

static
#if defined(__GNUC__) || defined(__GNUG__)
__attribute__((noreturn))
#endif
void usage(bool is_error) {
  printf("\nThis is NeQuickJRC version %.1f\n\n", NEQUICKG_VERSION);
  printf("Ionospheric Correction Algorithm for Galileo Single Frequency Users, 1.2 September 2016\n\n");
  printf("Usage: NeQuickJRC "
#ifndef FTR_MODIP_CCIR_AS_CONSTANTS
    "<modip file path>, <ccir files directory> "
#endif // !FTR_MODIP_CCIR_AS_CONSTANTS
    "[Options] \n\n");

  printf("Options: \n\n");

  printf(" -c "
    "<a0> <a1> <a2> "
    "<month> <UT> "
    "<station.longitude> <station.latitude> <station.height> "
    "<satellite.longitude> <satellite.latitude> <satellite.height>\n");
  printf("\tReturns the STEC in the standard output\n\n");

  printf(" -f "
    "<a0> <a1> <a2> "
    "<input_file> "
    "<output_file>\n"
    "\tReturns the STEC in the ouput file\n"
    "\tinput file format is:\n"
    "\t\t<month> <UT> "
    "<station.longitude> <station.latitude> <station.height> "
    "<satellite.longitude> <satellite.latitude> <satellite.height>\n"
    "\toutput file format is:\n"
    "\t\tinput data + modip(degrees) + STEC\n\n");

  printf(" -h prints this message and exits\n\n");

  printf(" -j "
    "<input_file>\n"
    "\tCalculates and checks the STEC against the value given in the file\n"
    "\tfile format is:\n"
    "\tfirst line: <a0> <a1> <a2>\n"
    "\tfollowing lines:\n"
    "\t\t<month> <UT> "
    "<station.longitude> <station.latitude> <station.height> "
    "<satellite.longitude> <satellite.latitude> <satellite.height> "
    "<STEC expected>\n\n");


  printf("\nParameter\tDescription                  \tunit: \n\n");
  printf("STEC       \tSlant Total Electron Content\tTECU (10^16 electrons/m2) \n");
  printf("month      \tJanuary = 1, ..., December = 12\t---\n");
  printf("UT         \tUT time                        \thours\n");
  printf("latitude\tWGS-84 ellipsoidal latitude\tdegrees\n");
  printf("longitude\tWGS-84 ellipsoidal latitude\tdegrees\n");
  printf("height   \tWGS-84 ellipsoidal height\tmeters\n");
  printf("a0       \tFirst ionospheric coefficient  \tsfu (10^-22 W/(m2*Hz)\n");
  printf("a1       \tSecond ionospheric coefficient \tsfu/degree\n");
  printf("a2       \tThird ionospheric coefficient  \tsfu/degree^2\n");

  if (is_error) {
    exit(NEQUICK_USAGE_EXCEPTION);
  } else {
    exit(1);
  }
}

static bool check_result(
  NeQuickG_chandle nequick,
  const double_t total_electron_content_expected,
  const double_t total_electron_content,
  clock_t begin) {

  clock_t end = clock();
  double_t time_spent = (double_t)(end - begin) / CLOCKS_PER_SEC;

  if (isnormal(total_electron_content_expected)) {
    bool success = THRESHOLD_COMPARE(
      total_electron_content,
      total_electron_content_expected,
      NEQUICK_G_JRC_TEC_EPSILON);

    if (!success) {
      to_std_output(nequick, total_electron_content);
      printf(
        "TEC is not the expected, \
          expected = %12.5f, \
          calculated = %12.5f\n",
        total_electron_content_expected,
        total_electron_content);
      return false;
    }
  }

  printf("TEC = %12.5f, time spent (s) = %f\n",
        total_electron_content, time_spent);
  return true;
}

static bool get_TEC(
  const NeQuickG_handle nequick,
  double_t * total_electron_content) {

  clock_t begin = clock();

  if (NeQuickG.get_total_electron_content(nequick, total_electron_content) !=
      NEQUICK_OK) {
    return false;
  }
}

int processData(
  const NeQuickG_handle handle,
  double_t* solarCoef,
  uint8_t month, double_t UTC,
  double_t longitude_degree,
  double_t latitude_degree,
  double_t height_meters,
  double_t satellite_longitude_degree,
  double_t satellite_latitude_degree,
  double_t satellite_height_meters,

  double_t * retElectronContent) {

    bool ret = true;
    setAllDatas(
        handle,
        solarCoef,
      month, UTC,
      longitude_degree,
      latitude_degree,
      height_meters,
      satellite_longitude_degree,
      satellite_latitude_degree,
      satellite_height_meters);
    if (!get_TEC(handle, retElectronContent)) {
      ret = false;
    };


  return (ret) ? NEQUICK_OK : NEQUICK_TEC_EXCEPTION;
}

int main() {

  volatile int ret = NEQUICK_OK;

  volatile NeQuickG_handle nequick = NEQUICKG_INVALID_HANDLE;

  NEQUICK_TRY {
    NeQuickG_handle nequick_;
    ret = NeQuickG.init(pModip_file, pCCIR_directory, &nequick_);
    if (ret != NEQUICK_OK) {
      NeQuick_exception_throw(
        NEQUICK_USAGE_EXCEPTION, NEQUICK_USAGE_BAD_FORMAT);
    }
    nequick = nequick_;
    double_t returnedElectronContent;
    double_t solarCoef[] = { 0.1, 0.2, 0.3 };
  ret = processData(
      nequick,
      solarCoef,    //double_t* solarCoef,
      11, 2,          //uint8_t month, double_t UTC,
      34,                   //double_t longitude_degree,
      44,                   //double_t latitude_degree,
      0,                 //double_t height_meters,
      40,                   //double_t satellite_longitude_degree,
      50,                   //double_t satellite_latitude_degree,
      20200000,                 //double_t satellite_height_meters,

      &returnedElectronContent);
    printf("electron Content: %lf", returnedElectronContent);
  } NEQUICK_CATCH(NEQUICK_USAGE_EXCEPTION) {
    // to satisfy lint a boolean is declared
    bool is_error = false;
    usage(is_error);
  } NEQUICK_CATCH_ALL_EXCEPTIONS {
    ret = g_NeQuick_exception_type;
  } NEQUICK_END;

  NEQUICK_TRY {
    NeQuickG.close(nequick);
  } NEQUICK_CATCH_ALL_EXCEPTIONS {
    ret = INT16_MIN;
  } NEQUICK_END;

  return ret;
}

#undef NEQUICK_UNIT_TEST_EXCEPTION
#undef NEQUICK_TEC_EXCEPTION
#undef NEQUICK_G_JRC_TEC_EPSILON
