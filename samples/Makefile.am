SUBDIRS = data
AM_CPPFLAGS = -I../lib/ $(OPENCV_CFLAGS) $(GL_CFLAGS) $(GLUT_CFLAGS) 
AM_LDFLAGS = $(GL_LIBS) $(GLUT_LIBS) $(OPENCV_LIBS) -lglut
EXTRA_DIST = 6dof.py

pythonsamplesdir = $(datadir)/@PACKAGE@/samples
#pythonsamples_PROGRAMS = pythonsamples 
#pythonsamples_SOURCES = pythonsamples.cpp 
#pythonsamples_LDADD = ../lib/libehci.la
pythonsamples_DATA = 6dof.py



6dofhanddir = $(datadir)/@PACKAGE@/samples
6dofhand_PROGRAMS = 6dofhand 
6dofhand_SOURCES = 6dofhand.cpp 
6dofhand_LDADD = ../lib/libehci.la
6dofhand_DATA = 6dofhand.cpp


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

simple2ddir = $(datadir)/@PACKAGE@/samples
simple2d_PROGRAMS = simple2d 
simple2d_SOURCES = simple2d.cpp 
simple2d_LDADD = ../lib/libehci.la
simple2d_DATA = simple2d.cpp

simple3ddir = $(datadir)/@PACKAGE@/samples
simple3d_PROGRAMS = simple3d 
simple3d_SOURCES = simple3d.cpp 
simple3d_LDADD = ../lib/libehci.la
simple3d_DATA = simple3d.cpp

zoomHanddir = $(datadir)/@PACKAGE@/samples
zoomHand_PROGRAMS = zoomHand 
zoomHand_SOURCES = zoomHand.cpp 
zoomHand_LDADD = ../lib/libehci.la
zoomHand_DATA = zoomHand.cpp




