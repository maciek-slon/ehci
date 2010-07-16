include_directories (${EHCI_SOURCE_DIR}/include) 

# Allow the developer to select if Dynamic or Static libraries are built
OPTION (BUILD_SHARED_LIBS "Build Shared Libraries" OFF)
# Set the LIB_TYPE variable to STATIC
SET (LIB_TYPE STATIC)
IF (BUILD_SHARED_LIBS)
  # User wants to build Dynamic Libraries, so change the LIB_TYPE variable to CMake keyword 'SHARED'
  SET (LIB_TYPE SHARED)
ENDIF (BUILD_SHARED_LIBS)

# Create a target for the library
ADD_LIBRARY(Ehci ${LIB_TYPE} ehci.cpp)

 
TARGET_LINK_LIBRARIES( Ehci ${OpenCV_LIBS} )
