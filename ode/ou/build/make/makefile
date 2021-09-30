PRODUCTNAME = ou
MODE ?= DEVELOP
BITS ?= 32

include makefile.os

CXX = gcc
AR = ar
LD = gcc

CXXFLAGS = -fno-exceptions -fno-rtti -Wall -g
ARFLAGS = -r
LDFLAGS = -fno-exceptions -fno-rtti -g

ifeq ($(OS_PLATFORM), i386)
ifneq ($(BITS), 64)

CXXFLAGS += -malign-double 

endif
endif

ifeq ($(MODE), DEBUG)

CXXFLAGS += -D_DEBUG
CXXFLAGS += -O0

else
ifeq ($(MODE), RELEASE)

CXXFLAGS += -DNDEBUG
CXXFLAGS += -O3 -funroll-loops

else

CXXFLAGS += -O2

endif
endif

ifeq ($(BITS), 64)

CXXFLAGS += -m64
LDFLAGS += -m64
 
else

CXXFLAGS += -m32
LDFLAGS += -m32

endif


SRCEXT=.cpp
OBJEXT=$(OS_OBJ_EXT)

OUTNAME := $(PRODUCTNAME)
OUTDIR = ../../lib/
OUTEXT=$(OS_LIB_EXT)

TESTNAME := $(PRODUCTNAME)test
TESTDIR = ../../bin/
TESTEXT=$(OS_EXE_EXT)

REFINEDCXX = $(notdir $(CXX))
OUTINTDIR := ../../intermediate/$(REFINEDCXX)/
TESTINTDIR := ../../intermediate/$(REFINEDCXX)_$(TESTNAME)/


ifeq ($(MODE), DEBUG)

OUTINTDIR := $(OUTINTDIR)debug
TESTINTDIR := $(TESTINTDIR)debug

else
ifeq ($(MODE), RELEASE)

OUTINTDIR := $(OUTINTDIR)release
TESTINTDIR := $(TESTINTDIR)release

else

OUTINTDIR := $(OUTINTDIR)develop
TESTINTDIR := $(TESTINTDIR)develop

endif
endif

ifeq ($(BITS), 64)

OUTINTDIR := $(OUTINTDIR)64
TESTINTDIR := $(TESTINTDIR)64


OUTNAME := $(OUTNAME)64
TESTNAME := $(TESTNAME)64

endif

ifeq ($(MODE), DEBUG)

OUTNAME := $(OUTNAME)_debug
TESTNAME := $(TESTNAME)_debug

endif

ifeq ($(MODE), RELEASE)

OUTNAME := $(OUTNAME)_release
TESTNAME := $(TESTNAME)_release

endif

OUTINTDIR := $(OUTINTDIR)/
TESTINTDIR := $(TESTINTDIR)/

OUTPATH = $(OUTDIR)$(OS_LIB_PRE)$(OUTNAME)$(OUTEXT)
TESTPATH = $(TESTDIR)$(TESTNAME)$(TESTEXT)

ININCDIR = ../../include
INDIR = ../../src/ou/

INFILES = \
	atomic \
	customization \
	malloc \
	threadlocalstorage

INOBJS = $(addprefix $(OUTINTDIR), $(addsuffix $(OBJEXT), $(INFILES)))


TESTINCDIR = $(ININCDIR)
TESTLIBDIR = $(OUTDIR)
TESTLIBS = $(OUTNAME)
TESTSRCDIR = ../../test/

ifeq ($(OS_TYPE), Linux)
TESTLIBS += pthread
endif

ifeq ($(OS_TYPE), SUN)
TESTLIBS += pthread c
endif


TESTFILES = outest

TESTOBJS = $(addprefix $(TESTINTDIR), $(addsuffix $(OBJEXT), $(TESTFILES)))


.PHONY: develop debug release develop64 debug64 release64 all ou

all: develop

develop:
	$(MAKE) ou MODE=DEVELOP BITS=32

debug: 
	$(MAKE) ou MODE=DEBUG BITS=32
	
release: 
	$(MAKE) ou MODE=RELEASE BITS=32
	
develop64:
	$(MAKE) ou MODE=DEVELOP BITS=64

debug64: 
	$(MAKE) ou MODE=DEBUG BITS=64
	
release64: 
	$(MAKE) ou MODE=RELEASE BITS=64
	
ou: $(OUTPATH) $(TESTPATH)


.PHONY: mkoutintpath mkoutpath mktestintpath mktestpath

$(OUTPATH): mkoutintpath mkoutpath $(INOBJS)
	$(OS_VP)echo BUILDING $(notdir $(OUTPATH))...
	$(OS_VP)$(AR) $(ARFLAGS) $(OUTPATH) $(INOBJS)

$(TESTPATH): mktestintpath mktestpath $(OUTPATH) $(TESTOBJS)
	$(OS_VP)echo LINKING $(notdir $(TESTPATH))...
	$(OS_VP)$(LD) $(LDFLAGS) $(LDADDFLAGS) -o $(TESTPATH) \
		$(addprefix -I, $(TESTINCDIR) $(OS_INCLUDE_PATH)) \
		$(addprefix -L, $(TESTLIBDIR) $(OS_LIB_PATH)) \
		$(TESTOBJS) \
		$(addprefix -l, $(TESTLIBS))

$(OUTINTDIR)%$(OBJEXT): $(INDIR)%$(SRCEXT)
	$(OS_VP)echo Compiling $(notdir $@)...
	$(OS_VP)$(CXX) $(CXXFLAGS) $(CXXADDFLAGS) -c -o $@ \
		$(addprefix -I, $(ININCDIR) $(OS_INCLUDE_PATH)) \
		$< 

$(TESTINTDIR)%$(OBJEXT): $(TESTSRCDIR)%$(SRCEXT)
	$(OS_VP)echo Compiling $(notdir $@)...
	$(OS_VP)$(CXX) $(CXXFLAGS) $(CXXADDFLAGS) -c -o $@ \
		$(addprefix -I, $(TESTINCDIR) $(OS_INCLUDE_PATH)) \
		$< 


mkoutintpath:
	$(call FN_MKDIR, $(OUTINTDIR))

mktestintpath:
	$(call FN_MKDIR, $(TESTINTDIR))

mkoutpath:
	$(call FN_MKDIR, $(OUTDIR))

mktestpath:
	$(call FN_MKDIR, $(TESTDIR))


.PHONY: runtest

runtest: $(TESTPATH)
	$(TESTPATH)
