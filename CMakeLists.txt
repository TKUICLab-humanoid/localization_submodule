cmake_minimum_required(VERSION 2.8.3)
project(localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  genmsg
  roscpp
  sensor_msgs
  std_msgs
  roslib
  message_generation
  tku_libs
  tku_msgs
)

find_package(OpenCV REQUIRED)
#set(TKU_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libtku_libs.so)
#set(STRATEGYBASE_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libstrategybase.so)
# add the resized image message
#add_message_files(DIRECTORY msg
#   FILES 
#)


#add_service_files(DIRECTORY srv FILES )

#generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES imageprocess
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add the publisher example
add_executable(localization
    src/LocalizationBase/LocalizationBase.cpp
    src/main/Localizationmain.cpp
    src/Drawing/Drawing.cpp
    src/ParticleFilter/ParticleFilter.cpp
)
target_link_libraries(localization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


