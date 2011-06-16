-include ../global.mk

SUBDIRS += stuff \
           math \
           graph \
           graph_optimizer \
           graph_optimizer_hogman

SUBDIRS += hogman_interactive

ifeq	($(LINUX),1)
SUBDIRS += graph_viewer
endif

-include ../build_tools/Makefile.subdirs
