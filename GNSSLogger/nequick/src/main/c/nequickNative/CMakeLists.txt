# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)


# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add_library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

add_library( # Specifies the name of the library.
            nequickNative

            # Sets the library as a shared library.
            SHARED

            # Provides a relative path to your source file(s).
            NeQuickG_JNI_wrapper.c
        NeQuickG_JRC_entryPoint.c
        NeQuickG_JRC_exception.h
        NeQuickG_JRC_input_data_std_stream.h
        NeQuickG_JRC_input_data_stream.c
        NeQuickG_JRC_exception.c
        NeQuickG_JRC_input_data_std_stream.c
        NeQuickG_JRC_input_data_stream.h

        lib/NeQuickG_JRC.c
        lib/NeQuickG_JRC_coordinates.c
        lib/NeQuickG_JRC_electron_density.c
        lib/NeQuickG_JRC_Gauss_Kronrod_integration.c
        lib/NeQuickG_JRC_geometry.c
        lib/NeQuickG_JRC_input_data.c
        lib/NeQuickG_JRC_interpolate.c
        lib/NeQuickG_JRC_iono_E_layer.c
        lib/NeQuickG_JRC_iono_F1_layer.c
        lib/NeQuickG_JRC_iono_F2_layer.c
        lib/NeQuickG_JRC_iono_F2_layer_fourier_coefficients.c
        lib/NeQuickG_JRC_iono_profile.c
        lib/NeQuickG_JRC_math_utils.c

        lib/NeQuickG_JRC_MODIP.c
        lib/NeQuickG_JRC_MODIP_grid.c
        lib/NeQuickG_JRC_ray.c
        lib/NeQuickG_JRC_ray_slant.c
        lib/NeQuickG_JRC_ray_vertical.c
        lib/NeQuickG_JRC_solar.c
        lib/NeQuickG_JRC_solar_activity.c
        lib/NeQuickG_JRC_TEC_integration.c
        lib/NeQuickG_JRC_time.c
        lib/ITU_R_P_371_8.c

        lib/CCIR/NeQuickG_JRC_CCIR.c
        lib/CCIR/NeQuickG_JRC_ccir11.c
        lib/CCIR/NeQuickG_JRC_ccir12.c
        lib/CCIR/NeQuickG_JRC_ccir13.c
        lib/CCIR/NeQuickG_JRC_ccir14.c
        lib/CCIR/NeQuickG_JRC_ccir15.c
        lib/CCIR/NeQuickG_JRC_ccir16.c
        lib/CCIR/NeQuickG_JRC_ccir17.c
        lib/CCIR/NeQuickG_JRC_ccir18.c
        lib/CCIR/NeQuickG_JRC_ccir19.c
        lib/CCIR/NeQuickG_JRC_ccir20.c
        lib/CCIR/NeQuickG_JRC_ccir21.c
        lib/CCIR/NeQuickG_JRC_ccir22.c
          )
include_directories(
        lib
        lib/private
        lib/public
        lib
        ../../../androidTest/java
)
find_library( log-lib log )
target_link_libraries(nequickNative ${log-lib} )
add_definitions(-DFTR_MODIP_CCIR_AS_CONSTANTS=1)
