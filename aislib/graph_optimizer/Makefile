-include ../../global.mk

OBJS= graph_optimizer2d_aux.o

#this is for safe type checking
#CPPFLAGS += -D"_MY_CAST_=dynamic_cast"

LDFLAGS+=  -lm -l$(LIB_PREFIX)stuff -l$(LIB_PREFIX)graph 
CPPFLAGS+= 

-include ../../build_tools/Makefile.generic-shared-object
