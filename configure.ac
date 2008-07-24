AC_INIT([ehci], [1.0], [danielbaggio@gmail.com])
AC_CONFIG_AUX_DIR([build-aux])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
PKG_CHECK_MODULES(DEPS, opencv >= 1.1.0)
AC_SUBST(DEPS_CFLAGS)
AC_SUBST(DEPS_LIBS)


AC_CHECK_HEADERS(GL/gl.h GL/glu.h,,
         [AC_CHECK_HEADERS(OpenGL/gl.h OpenGL/glu.h,,
           [echo "OpenGL is mandatory";exit 1])]
)

if test `uname -s` != Darwin;
then
AC_CHECK_LIB(GL, glInitNames,,
     AC_CHECK_LIB(MesaGL,glInitNames,,
            echo "OpenGL is mandatory";exit 1)
)

dnl Check for GLU (Original or Mesa)
AC_CHECK_LIB(GLU, gluLookAt, ,
     AC_CHECK_LIB(MesaGLU,gluLookAt,,
     echo "incomplete OpenGL (no GLU)";exit 1)
)
fi


AC_CHECK_LIB([efence], [malloc], [EFENCELIB=-lefence])
AC_SUBST([EFENCELIB])


AC_PROG_LIBTOOL
AC_PROG_CC
AC_CONFIG_HEADERS([ehciconfig.h])
AC_CONFIG_FILES([Makefile lib/Makefile samples/Makefile])
AC_OUTPUT

