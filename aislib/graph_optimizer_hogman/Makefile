-include ../../global.mk

OBJS  =	csparse_helper.o

APPS  = hogman2d hogman3d


CPPFLAGS += -D"_MY_CAST_=reinterpret_cast"

#this is for safe type checking
#CPPFLAGS += -D"_MY_CAST_=dynamic_cast"

LIBS += -l$(LIB_PREFIX)graph_optimizer -l$(LIB_PREFIX)graph \
	-l$(LIB_PREFIX)stuff  -l$(LIB_PREFIX)math -l$(LIB_PREFIX)csparse

CPPFLAGS += -I$(ROOTDIR)/aislib

-include ../../build_tools/Makefile.generic-shared-object
