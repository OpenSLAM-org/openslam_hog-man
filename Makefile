-include ./global.mk

SUBDIRS_EXTERNAL = EXTERNAL
SUBDIRS_AISLIB	 = aislib
	
SUBDIRS = 	$(SUBDIRS_EXTERNAL)\
		slam_parser $(SUBDIRS_AISLIB)

LDFLAGS+=
CPPFLAGS+=

-include ./build_tools/Makefile.subdirs
