#cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(surface)


add_library(surface SHARED
        Surface.cc
)
target_link_libraries(surface PUBLIC

        PUBLIC px4_gz_msgs
        PUBLIC ${GZ_SENSORS_TARGET}
        PUBLIC ${GZ_PLUGIN_TARGET}
        PUBLIC ${GZ_SIM_TARGET}
        PUBLIC ${GZ_TRANSPORT_TARGET}
        PUBLIC waves
)


