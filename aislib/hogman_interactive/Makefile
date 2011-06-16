-include ../../global.mk

OBJS= hogman_slam_interface.o

APPS= hogman_online

LDFLAGS+= -l$(LIB_PREFIX)parser
LDFLAGS+= -l$(LIB_PREFIX)interface
LDFLAGS+= -l$(LIB_PREFIX)graph_optimizer_hogman

CPPFLAGS+= -I$(ROOTDIR)/aislib

-include ../../build_tools/Makefile.generic-shared-object



