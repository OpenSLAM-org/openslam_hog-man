-include ../../global.mk

OBJS  = primitives.o pose_graph_vis3d.o qgl_graph_viewer.o
OBJS += matrix4x4.o vrml_output.o vector4.o main_widget.o
APPS = graph_viewer

LDFLAGS  += -lqglviewer-qt4 -lGLU -lGL -l$(LIB_PREFIX)graph -lpthread
LDFLAGS+= $(QT_LIB)
CPPFLAGS+= $(QT_INCLUDE)
CPPFLAGS+= -I$(ROOTDIR)/EXTERNAL

-include ../../build_tools/Makefile.generic-shared-object
