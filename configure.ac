AC_INIT([ehci], [0.3], [danielbaggio@gmail.com],[ehci])

AC_CONFIG_SRCDIR([lib/ehci.cpp])

AC_CONFIG_AUX_DIR([build-aux])

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

PKG_CHECK_MODULES(OPENCV, opencv >= 1.1.0)
AC_SUBST(OPENCV_CFLAGS)
AC_SUBST(OPENCV_LIBS)
AX_CHECK_GL
AX_CHECK_GLUT


AC_CONFIG_FILES([Makefile lib/Makefile samples/Makefile data/Makefile])
AC_OUTPUT

