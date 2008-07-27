SUBDIRS = data
AM_CPPFLAGS = -I../lib/ $(OPENCV_CFLAGS) $(GL_CFLAGS) $(GLUT_CFLAGS) 
AM_LDFLAGS = $(GL_LIBS) $(GLUT_LIBS) $(OPENCV_LIBS)

6dofheaddir = $(datadir)/@PACKAGE@/samples
6dofhead_PROGRAMS = 6dofhead 
6dofhead_SOURCES = 6dofhead.cpp 
6dofhead_LDADD = ../lib/libehci.la
6dofhead_DATA = 6dofhead.cpp


boxView3ddir = $(datadir)/@PACKAGE@/samples
boxView3d_PROGRAMS = boxView3d 
boxView3d_SOURCES = boxView3d.cpp 
boxView3d_LDADD = ../lib/libehci.la
boxView3d_DATA = boxView3d.cpp


