if PYEHCI

#EXTRA_DIST = ehci.py

BUILT_SOURCES = $(srcdir)/swig_wrap.cpp 
SWIG_SOURCES = ehci.i 

pkgpython_PYTHON = __init__.py ehci.py
pkgpyexec_LTLIBRARIES = _ehci.la
_ehci_la_SOURCES = $(srcdir)/swig_wrap.cpp $(SWIG_SOURCES)
_ehci_la_CPPFLAGS = $(SWIG_PYTHON_CPPFLAGS) $(PYTHON_CPPFLAGS) $(OPENCV_CFLAGS) $(GL_CFLAGS) $(GLUT_CFLAGS) -I$(top_srcdir)/lib
_ehci_la_LDFLAGS = -module $(GL_LIBS) $(GLUT_LIBS) $(OPENCV_LIBS)
_ehci_la_LIBADD = ../lib/libehci.la


$(srcdir)/swig_wrap.cpp: $(SWIG_SOURCES)
	$(SWIG) $(SWIG_PYTHON_OPT) -python -I$(top_srcdir)/src -o $@ $<



endif
