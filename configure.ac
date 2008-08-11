AC_INIT([ehci], [0.3], [http://code.google.com/p/ehci/issues/list],[ehci])

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
# Checking for required libraries #
###################################

#In some distributions, the location /usr/local/lib/pkgconfig
#is not automatically checked for installed libraries
export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig"

PKG_CHECK_MODULES(OPENCV, opencv >= 1.1.0)


AC_SUBST(OPENCV_CFLAGS)
AC_SUBST(OPENCV_LIBS)
AX_CHECK_GL
AX_CHECK_GLUT


AC_CONFIG_FILES([Makefile ehci.pc lib/Makefile samples/Makefile samples/data/Makefile doc/Makefile
 doc/html/Makefile])
AC_OUTPUT

