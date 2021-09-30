AM_CPPFLAGS = $(ODE_CFLAGS) $(OPENGL_CFLAGS) \
    -I${top_srcdir}/deps/AntTweakBar/include

AM_LDFLAGS = ../deps/AntTweakBar/src/libAntTweakBar.la \
            $(ODE_LIBS) $(OPENGL_LIBS)


if GLFW
AM_CPPFLAGS += $(GLFW_CFLAGS)
AM_LDFLAGS += $(GLFW_LIBS)
endif

bin_PROGRAMS = icollision

icollision_SOURCES = main.cpp \
                     camera.cpp camera.hpp

