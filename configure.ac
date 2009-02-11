AC_INIT([ehci], [0.5], [http://code.google.com/p/ehci/issues/list],[ehci])

AC_CONFIG_SRCDIR([lib/ehci.cpp])

AC_CONFIG_AUX_DIR([.build-aux])

AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_CONFIG_HEADER([ehciconfig.h])



AC_PROG_CC
AM_DISABLE_STATIC
AC_LIBTOOL_WIN32_DLL
AM_PROG_LIBTOOL

AM_PROG_AS


###################################
# SWIG and Python related         #
###################################

AM_PATH_PYTHON(2.3)
AC_PROG_SWIG(1.3.21)
SWIG_ENABLE_CXX
SWIG_PYTHON
AC_PYTHON_DEVEL(>= '2.3')

# optionaly pyehci compiling

AC_ARG_ENABLE(pyehci,
[  --enable-pyehci    Turn on python bindings for EHCI],
[case "${enableval}" in
  yes) pyehci=true ;;
  no)  pyehci=false ;;
  *) AC_MSG_ERROR(bad value ${enableval} for --enable-pyehci) ;;
esac],[pyehci=false])
AM_CONDITIONAL(PYEHCI, test x$pyehci = xtrue)



###################################
# Checking for required libraries #
###################################

#In some distributions, the location /usr/local/lib/pkgconfig
#is not automatically checked for installed libraries
export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig"

PKG_CHECK_MODULES(OPENCV, opencv >= 1.0.0)


AC_SUBST(OPENCV_CFLAGS)
AC_SUBST(OPENCV_LIBS)
AX_CHECK_GL
AX_CHECK_GLUT


AC_CONFIG_FILES([Makefile ehci.pc lib/Makefile samples/Makefile samples/data/Makefile doc/Makefile
 doc/html/Makefile pyehci/Makefile])
AC_OUTPUT

