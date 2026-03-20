#cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(wavevisual)




# Waves
add_library(wavevisual SHARED
        WaveVisual.cc
)
target_link_libraries(wavevisual PUBLIC
#        Eigen3::Eigen
#        gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
#        gz-math${GZ_MATH_VER}
#        gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
#        gz-sim${GZ_SIM_VER}::core
        PUBLIC px4_gz_msgs
        PUBLIC ${GZ_SENSORS_TARGET}
        PUBLIC ${GZ_PLUGIN_TARGET}
        PUBLIC ${GZ_SIM_TARGET}
        PUBLIC ${GZ_TRANSPORT_TARGET}
        PUBLIC waves
)

