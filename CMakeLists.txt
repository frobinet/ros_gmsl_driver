cmake_minimum_required(VERSION 3.1)
project(gmsl_n_cameras)

set (CMAKE_CXX_STANDARD 11)

# FindDriveworks.cmake, ArchConfiguration.cmake, and LibFindMacros.cmake were needed for my setup they are taken from driveworks/samples/cmake/
# ArchConfiguration.cmake was the only file that needed small changes, remove the fatal error on line 17 and add the following lines in its place
         set(VIBRANTE TRUE)
         add_definitions(-DVIBRANTE)
# this is the path I placed the driveworks cmake files in
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# # set(glfw_INCLUDE_DIR /usr/local/include/)
# # set(glfw_LIBRARY /usr/local/lib/libglfw3.a)

set(drive_t186ref_linux_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/drive-t186ref-linux/include)
set(lodepng_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/linux-aarch64/lodepng-20150912/include)
set(lodepng_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/linux-aarch64/lodepng-20150912/lib)

link_directories(${lodepng_LIBRARY})

find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(GLEW REQUIRED)
find_package(Driveworks REQUIRED)
find_package(OpenCV REQUIRED )


find_package(X11 REQUIRED )
if(NOT X11_FOUND)
    message("ERROR: x11 not found")
endif(NOT X11_FOUND)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.1 REQUIRED 
			PATHS /usr/local )

  
# # # find_package(PkgConfig REQUIRED)
# # # set( CMAKE_PREFIX_PATH /usr/local/ )
# # # set( PKG_CONFIG_PATH /usr/local/lib/pkgconfig )
# # # pkg_search_module(glfw REQUIRED glfw3)

# SET THESE VARIABLES TO YOUR DRIVEWORKS LOCATIONS
set(Driveworks_INCLUDE_DIR /usr/local/driveworks/include/)
set(Driveworks_LIBRARY /usr/local/driveworks/lib/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cv_bridge 
  image_transport
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)
link_directories(
  ${Driveworks_LIBRARY})

MESSAGE("Driveworks include directory ${Driveworks_INCLUDE_DIR}")

# ros node name template
set(NODE_NAME ${PROJECT_NAME}_node)

add_executable(${NODE_NAME} src/main.cpp
  src/ProgramArguments.cpp
  src/WindowEGL.cpp
  src/WindowGLFW.cpp  
  src/cv_connection.cpp
  src/SampleFramework.cpp
  src/Log.cpp
)

target_link_libraries(${NODE_NAME} # See CMakeList.txt of dw_samples_framework
  ${catkin_LIBRARIES}
  nvmedia  
  ${GLEW_LIBRARY}
  driveworks
  GLESv2
  EGL
  drm
  ${CUDA_LIBRARY}
  ${OpenCV_LIBS} 
  
  glfw
  ${OPENGL_gl_LIBRARY}
	${X11_LIBRARIES}
	${X11_Xrandr_LIB}
	${X11_Xcursor_LIB}
	${X11_Xxf86vm_LIB}
	${X11_Xinerama_LIB}
	${X11_Xi_LIB}
  
  ## ${glfw_LIBRARY}
  
  #${glfw_LIBRARY_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Driveworks_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${GLEW_INCLUDE_DIR}
  ${drive_t186ref_linux_INCLUDE_DIR}
  ${lodepng_INCLUDE_DIR}
  
  ${glfw_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR} 
  ${X11_INCLUDE_DIR}
  
  ##${glfw_INCLUDE_DIR}
)
