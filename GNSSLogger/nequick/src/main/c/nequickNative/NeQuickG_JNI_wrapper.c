#include <jni.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <android/log.h>
#include "NeQuickG_JRC.h"

bool setupAndProcessData(  double_t* solarCoef,
                          uint8_t month, double_t UTC,
                          double_t longitude_degree,
                          double_t latitude_degree,
                          double_t height_meters,
                          double_t satellite_longitude_degree,
                          double_t satellite_latitude_degree,
                          double_t satellite_height_meters,
                          double_t *returnedElectronContent);

JNIEXPORT jdouble JNICALL
Java_com_gnssdipgroup_nequick_NeQuickHelper_nequickComputeNative(JNIEnv *env, jobject thiz,
                                                           jdoubleArray solar_coef, jint month,
                                                           jdouble utc, jdouble lng, jdouble lat,
                                                           jdouble height, jdouble sat_lng,
                                                           jdouble sat_lat, jdouble sat_height) {
    double_t returnedElectronContent;
    jdouble *solar_coef_array = (*env)->GetDoubleArrayElements(env, solar_coef, NULL);
    bool checkRet = setupAndProcessData(  solar_coef_array,
                              month, utc,
                              lng,
                              lat,
                              height,
                              sat_lng,
                              sat_lat,
                              sat_height,
                              &returnedElectronContent);



    if(checkRet == true)
        return returnedElectronContent;
    else {
        char errStr[50];
        snprintf(errStr, 50, "%d", checkRet);
        __android_log_write(ANDROID_LOG_ERROR, "nequickTag", errStr);//Or ANDROID_LOG_INFO, ...
        return -1;
    }
}