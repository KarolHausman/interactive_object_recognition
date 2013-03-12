cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include_directories("/usr/include/vtk-5.6/")
#include_directories("~/ros_workspace/perception_new/pcl/include/pcl-1.7/")
#include_directories("/usr/include/vtk-5.6/")
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Eigen
#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

# VTK ####################
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
MESSAGE("vtk lib--------------------")
MESSAGE(${VTK_LIBRARY_DIRS})
#SET(VTK_LIBRARIES vtkalglib vtkCharts vtkCommon vtkDICOMParser vtkexoIIc vtkFiltering vtkftgl vtkGenericFiltering vtkGeovis vtkGraphics vtkHybrid vtkImaging vtkInfovis vtkIO vtkmetaio vtkParallel vtkproj4 vtkQtChart vtkRendering vtksqlite vtksys vtkverdict vtkViews vtkVolumeRendering vtkWidgets)
SET(VTK_LIBRARIES vtkCommon vtkFiltering boost_system)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_library(visualizer src/VisualizerImpl.cpp)
target_link_libraries(visualizer ${VTK_LIBRARIES})
#target_link_libraries(visualizer ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
