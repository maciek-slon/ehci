AM_CPPFLAGS = -I$(srcdir)/../lib -I /usr/local/include/opencv/
AM_LDFLAGS = -lGL -lglut `pkg-config opencv --libs`
bin_PROGRAMS = 6dofhead
6dofheaddir = $(bindir)/data
6dofhead_SOURCES = 6dofhead.cpp
6dofhead_LDADD = ../lib/libehci.la
6dofhead_DATA = data/haarcascade_frontalface_default.xml data/head.raw

  
