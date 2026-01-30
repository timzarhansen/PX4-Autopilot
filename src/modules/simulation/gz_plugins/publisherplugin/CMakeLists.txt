#cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(publisherplugin)

add_library(publisherplugin SHARED
        PublisherPlugin.cc
)
target_link_libraries(publisherplugin PUBLIC
        PUBLIC px4_gz_msgs
        PUBLIC ${GZ_SENSORS_TARGET}
        PUBLIC ${GZ_PLUGIN_TARGET}
        PUBLIC ${GZ_SIM_TARGET}
        PUBLIC ${GZ_TRANSPORT_TARGET}
        waves
)
