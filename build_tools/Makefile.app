# Makefile generico per applicazione
#
# Variabili:
# APPS lista delle applicazioni
# OBJS lista degli oggetti
# QOBJS lista degli oggetti QT
# LIBS librerie
#
# Ogni applicazione viene linkata con tutti gli oggetti


ifeq ($(LINUX),1)
CPPFLAGS+=-DLINUX
endif

ifeq ($(WINDOWS),1)
CPPFLAGS+=-DWINDOWS
endif

APPLICATIONS= $(foreach a, $(APPS),$(BINDIR)/$(a))
CAPPLICATIONS= $(foreach a, $(CAPPS),$(BINDIR)/$(a))
all: $(APPLICATIONS) $(CAPPLICATIONS)

PACKAGE=$(notdir $(shell pwd))

.SECONDARY:		$(OBJS) $(COBJS) $(QOBJS)
.PHONY:			all clean copy doc

$(QOBJS): %.o: %.cpp moc_%.cpp
	@$(MESSAGE) "Compiling (QT) $@"
	@$(PRETTY) "$(CXX) $(CPPFLAGS) $(QT_INCLUDE) $(CXXFLAGS) -c $< -o $@"

moc_%.cpp: %.h
	@$(MESSAGE) "Generating MOC $@"
	@$(PRETTY) "$(MOC) -i $< -o $@"

# Generazione degli oggetti
%.o: %.cpp
	@$(MESSAGE) "Compiling $@"
	@$(PRETTY) "$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@"

# Generazione degli oggetti
%.o: %.c
	@$(MESSAGE) "Compiling $@"
	@$(PRETTY) "$(CC) $(CPPFLAGS) $(CCFLAGS) -c $< -o $@"

# Generazione delle applicazioni
$(BINDIR)/%: %.cpp $(OBJS) $(COBJS) $(QOBJS)
	@$(MESSAGE) "Linking application `basename $@` [c++]"
	@$(PRETTY) "$(CXX) $(CPPFLAGS) $(CXXFLAGS) $<  $(OBJS) $(QOBJS)  -L$(LIBDIR) $(LDFLAGS) $(LIBS) -o $@"

$(BINDIR)/%: %.c $(COBJS) $(QOBJS)
	@$(MESSAGE) "Linking application `basename $@` [c]"
	@$(PRETTY) "$(CC) $(CPPFLAGS) $(CFLAGS) $<  $(COBJS)  -L$(LIBDIR) $(LIBS) -o $@"


#Regole per la generazione delle dipendenze
#Regole per la generazione delle dipendenze
OBJDEPS= $(foreach module,$(basename $(OBJS)),$(module).d) $(foreach a, $(APPS),$(a).d)
COBJDEPS=$(foreach module,$(basename $(COBJS)),$(module).d)

$(OBJDEPS): %.d: %.cpp
	@$(MESSAGE) "Generating dependencies for $<"
	@$(PRETTY) "$(CXX) $(CPPFLAGS) -MM -MG $< -MF $@"

$(COBJDEPS): %.d: %.c
	@$(MESSAGE) "Generating dependencies for $<"
	@$(PRETTY) "$(CC)  $(CPPFLAGS) -MM -MG $< -MF $@"

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),copy)
ifneq ($(MAKECMDGOALS),dep)
-include $(OBJDEPS) $(COBJDEPS)
endif
endif
endif

doc:
	rm -rf doc/$(PACKAGE)
ifeq ($(strip $(DOCTITLE)),)
	kdoc -L doc -d doc/$(PACKAGE) -n "Package $(PACKAGE) (lib$(PACKAGE).so)" $(HEADERS)
else
	kdoc -L doc -d doc/$(PACKAGE) -n "$(DOCTITLE) (lib$(PACKAGE).so)" $(HEADERS)
endif

clean:
	@$(MESSAGE) "Cleaning $(PACKAGE)"
	@$(PRETTY) "rm -f *.d *.o  moc_*.cpp *.d core *~ table_*.cpp  gen_table*[^.][^c][^p][^p] $(APPLICATIONS) $(CAPPLICATIONS)"
	@$(PRETTY) "rm -rf doc/$(PACKAGE)"

copy:	clean
	tar -C .. -cvzf `date +../$(PACKAGE)-%d%b%y.tgz` $(PACKAGE)
