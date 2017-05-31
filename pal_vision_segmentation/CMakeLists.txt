cmake_minimum_required(VERSION 2.8.3)
project(pal_vision_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
)

## System dependencies are found with CMake's conventions
#find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/DisparitySegment.cfg
  cfg/DepthSegment.cfg
  cfg/HistogramSegment.cfg
  cfg/TemplateSegment.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pal_vision_segmentation
  CATKIN_DEPENDS geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/image_processing.cpp
    src/histogram.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(disparity_segmentation
    src/disparity_segmentation.cpp)
add_dependencies(disparity_segmentation
    ${PROJECT_NAME}_gencfg)
target_link_libraries(disparity_segmentation ${PROJECT_NAME})

add_executable(depth_segmentation
    src/depth_segmentation.cpp)
add_dependencies(depth_segmentation
    ${PROJECT_NAME}_gencfg)
target_link_libraries(depth_segmentation ${PROJECT_NAME})

add_executable(template_segmentation
    src/template_segmentation.cpp
    src/image_processing.cpp)
add_dependencies(template_segmentation
    ${PROJECT_NAME}_gencfg)
target_link_libraries(template_segmentation ${PROJECT_NAME})

add_executable(histogram_segmentation
    src/histogram_segmentation.cpp)
add_dependencies(histogram_segmentation
    ${PROJECT_NAME}_gencfg)
target_link_libraries(histogram_segmentation
    ${PROJECT_NAME}
    )
#    ${OpenCV_LIBS}

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS disparity_segmentation depth_segmentation template_segmentation histogram_segmentation ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 catkin_add_gtest(${PROJECT_NAME}-test test/histogram_segmentation_unit_test.cpp)
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 endif()
