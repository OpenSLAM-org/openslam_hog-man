-include ../../global.mk

OBJS = graph.o dijkstra.o posegraph3d.o loadEdges3d.o posegraph2d.o
 
LDFLAGS += -l$(LIB_PREFIX)stuff -l$(LIB_PREFIX)math
CPPFLAGS+= 

-include ../../build_tools/Makefile.generic-shared-object
