#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.68])
AC_INIT([interactive-collision], [0.1], [danielosmari@users.sourceforge.net])
AC_CONFIG_SRCDIR([src/main.cpp])
AC_CONFIG_HEADERS([config.h])

AM_INIT_AUTOMAKE

LT_INIT

# Checks for programs.
AC_PROG_CC
AC_PROG_CXX
PKG_PROG_PKG_CONFIG
AC_LANG(C++)

# Checks for libraries.
PKG_CHECK_MODULES(ODE, ode)
PKG_CHECK_MODULES(OPENGL, gl glu)
PKG_CHECK_MODULES(GLFW, libglfw, [have_glfw=true], [have_glfw=false])

AM_CONDITIONAL([GLFW], [test x$have_glfw = xtrue])
if test x$have_glfw = xtrue
then
    AC_DEFINE([USE_GLFW], [1], [define if GLFW is available])
fi

# TODO: when I get around implementing other backends...
# PKG_CHECK_MODULES(SDL, sdl, AC_DEFINE(HAVE_SDL))
# PKG_CHECK_MODULES(GLUT, glut, AC_DEFINE(HAVE_GLUT))

# Checks for header files.
AC_CHECK_HEADERS

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.
AC_CHECK_FUNCS([atexit])
AC_CHECK_FUNCS([sqrt])
AC_HEADER_STDBOOL

AC_CONFIG_SUBDIRS([deps/AntTweakBar])

AC_CONFIG_FILES([Makefile
                 deps/Makefile
                 src/Makefile])
AC_OUTPUT
