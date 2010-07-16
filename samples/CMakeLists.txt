# ---------------------------------------------
#      Define executable targets
# ---------------------------------------------
MACRO(MY_DEFINE_EXAMPLE name srcs)
	add_executable(${name} ${srcs})
	set_target_properties(${name} PROPERTIES PROJECT_LABEL "(EXAMPLE) ${name}")
	add_dependencies(${name} Ehci)
	target_link_libraries(${name} ${OpenCV_LIBS} Ehci)

	if(WIN32)
		install(TARGETS ${name}
		RUNTIME DESTINATION "samples" COMPONENT main)
	endif()
ENDMACRO(MY_DEFINE_EXAMPLE)

project(samples)
include_directories (${EHCI_SOURCE_DIR}/include) 
if(CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
endif()


if (BUILD_EXAMPLES)
    
    MY_DEFINE_EXAMPLE(simple2d			simple2d.cpp)    
    MY_DEFINE_EXAMPLE(simple3d			simple3d.cpp)    
     
endif(BUILD_EXAMPLES)

FIND_PATH(GLUT_INCLUDE_DIR glut.h /usr/include/foo /usr/local/include/foo)
FIND_LIBRARY(GLUT_LIBRARY NAMES glut glut32 PATHS ${OPENGL_LIBRARY_DIR} ${GLUT_ROOT_PATH}/Release)

if (BUILD_OPENGL_EXAMPLES)
	
	find_package(OpenGL REQUIRED)
	
	MESSAGE(STATUS "Looking for GLUT")	
	find_package(GLUT REQUIRED)

	IF (GLUT_INCLUDE_DIR AND GLUT_LIBRARY)
		SET(GLUT_FOUND TRUE)
	ENDIF (GLUT_INCLUDE_DIR AND GLUT_LIBRARY)

	IF (GLUT_FOUND)
		MESSAGE(STATUS "Looking for GLUT - found : ${GLUT_INCLUDE_DIR}")
	ELSE (GLUT_FOUND)
		MESSAGE(FATAL_ERROR "Looking for GLUT - not found")		
			
	ENDIF (GLUT_FOUND)
	
	
	
	include_directories (${GLUT_INCLUDE_DIR})
	
	MY_DEFINE_EXAMPLE(boxView3d			boxView3d.cpp)   
	TARGET_LINK_LIBRARIES( boxView3d ${OpenCV_LIBS} ${OpenGL_LIBS} ${GLUT_LIBRARY})	
	MY_DEFINE_EXAMPLE(zoomHand			zoomHand.cpp)   
	TARGET_LINK_LIBRARIES(zoomHand ${OpenCV_LIBS} ${OpenGL_LIBS} ${GLUT_LIBRARY})	
	MY_DEFINE_EXAMPLE(6dofhand			6dofhand.cpp)   
	TARGET_LINK_LIBRARIES(6dofhand ${OpenCV_LIBS} ${OpenGL_LIBS} ${GLUT_LIBRARY})	
	MY_DEFINE_EXAMPLE(6dofhead			6dofhead.cpp)   
	TARGET_LINK_LIBRARIES(6dofhead ${OpenCV_LIBS} ${OpenGL_LIBS} ${GLUT_LIBRARY})	
	
	
endif(BUILD_OPENGL_EXAMPLES)

if (INSTALL_C_EXAMPLES AND NOT WIN32)
    file(GLOB SAMPLES *.c *.cpp *.jpg *.png *.data makefile.* build_all.sh *.dsp *.cmd )
    install(FILES ${SAMPLES}
            DESTINATION share/ehci/samples/c
            PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
endif ()
