##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gnd_lssmap_maker)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs gnd_msgs gndlib gnd_rosutil )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gnd_lssmap_maker
  CATKIN_DEPENDS roscpp sensor_msgs gnd_msgs gndlib gnd_rosutil 
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##############################################################################
# Sources
##############################################################################

#add_library(gnd_lssmap_maker src/lib/gnd_lssmap_maker.cpp)
#target_link_libraries(gnd_lssmap_maker ${catkin_LIBRARIES})

#install(TARGETS gnd_lssmap_maker
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/gnd_lssmap_maker/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

##############################################################################
# Example
##############################################################################

add_executable(gnd_lssmap_maker src/gnd_lssmap_maker.cpp)
target_link_libraries(gnd_lssmap_maker ${catkin_LIBRARIES})
install(TARGETS gnd_lssmap_maker 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(gnd_lssmap_maker sensor_msgs_generate_messages_cpp gnd_msgs_generate_messages_cpp)

##############################################################################
# Test
##############################################################################

#if(CATKIN_ENABLE_TESTING)
#  find_package(catkin COMPONENTS rostest rosunit)
#  include_directories(${GTEST_INCLUDE_DIRS})
#  catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
#  if(TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  endif()
#endif()
