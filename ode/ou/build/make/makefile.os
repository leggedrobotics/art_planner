ifeq ($(OS), Windows_NT)

OS_TYPE = WIN32
OS_PLATFORM := i386
            
OS_OBJ_EXT = .obj
OS_LIB_EXT = .lib
OS_LIB_PRE = 
OS_EXE_EXT = .exe

FN_CONVERT_PATH = $(subst /,\,$(subst //,/,$(1)))

FN_RM = $(OS_VP)erase "$(strip $(call FN_CONVERT_PATH, $(1)))"
FN_MKDIR = $(OS_VP)if NOT EXIST "$(strip $(call FN_CONVERT_PATH, $(1)))" mkdir "$(strip $(call FN_CONVERT_PATH, $(1)))"

OS_INCLUDE_PATH = $(GCC_PATH)/include $(GCC_PATH)/include/win32
OS_LIB_PATH += $(GCC_PATH)/lib $(GCC_PATH)/lib/win32
OS_VP=@

else

OS_OBJ_EXT = .o
OS_LIB_EXT = .a
OS_LIB_PRE = lib
OS_EXE_EXT = 

OS_TYPE = $(subst SunOS,SUN,$(shell uname))
OS_PLATFORM := $(shell uname -p)
OS_PLATFORM := $(subst x86_64,i386,$(OS_PLATFORM))
OS_PLATFORM := $(subst i586,i386,$(OS_PLATFORM))
OS_PLATFORM := $(subst i686,i386,$(OS_PLATFORM))

FN_RM = $(OS_VP)rm -rf $(1)
FN_MKDIR = $(OS_VP)test -d $(1) || mkdir -p $(patsubst %/,%,$(1))

OS_INCLUDE_PATH = /include /usr/include
OS_LIB_PATH = /lib /usr/lib
OS_VP=@

endif

