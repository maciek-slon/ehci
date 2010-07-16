cmake_minimum_required (VERSION 2.6) 
find_package(OpenCV REQUIRED)
project (EHCI)
add_subdirectory (src) 
add_subdirectory (samples) 

option (BUILD_EXAMPLES "Build Samples" ON)
option (BUILD_OPENGL_EXAMPLES "Build OpenGL Samples" OFF)

#copy data files
ADD_CUSTOM_COMMAND(
    TARGET ehci
    POST_BUILD    
    COMMAND ${CMAKE_COMMAND} -E copy ARGS ${EHCI_SOURCE_DIR}/data ${EHCI_SOURCE_DIR}/bin/data
)







