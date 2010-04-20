-include ../../global.mk

OBJS  = primitives.o pose_graph_vis3d.o qgl_graph_viewer.o
OBJS += matrix4x4.o vrml_output.o vector4.o main_widget.o
APPS = graph_viewer

LDFLAGS  += -l$(LIB_PREFIX)graph -l$(LIB_PREFIX)qglviewer
LDFLAGS+= $(QT_LIB)
CPPFLAGS+= $(QT_INCLUDE)
CPPFLAGS+= -I$(ROOTDIR)/EXTERNAL

-include ../../build_tools/Makefile.generic-shared-object
