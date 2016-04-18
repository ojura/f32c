################ Build options #######################################

NAME		:= ustl
MAJOR		:= 2
MINOR		:= 3

#DEBUG		:= 1
BUILD_SHARED	:= 1
#BUILD_STATIC	:= 1
NOLIBSTDCPP	:= 1

################ Programs ############################################

CXX		:= g++
LD		:= g++
AR		:= ar
RANLIB		:= ranlib
DOXYGEN		:= doxygen
INSTALL		:= install

INSTALLDATA	:= ${INSTALL} -D -p -m 644
INSTALLLIB	:= ${INSTALLDATA}
RMPATH		:= rmdir -p --ignore-fail-on-non-empty

################ Destination #########################################

INCDIR		:= /usr/local/include
LIBDIR		:= /usr/local/lib

################ Compiler options ####################################

WARNOPTS	:= -Wall -Wextra -Woverloaded-virtual -Wpointer-arith\
		-Wshadow -Wredundant-decls -Wcast-qual
TGTOPTS		:= -std=c++11 -march=native
INLINEOPTS	:= -fvisibility-inlines-hidden -fno-threadsafe-statics -fno-enforce-eh-specs

CXXFLAGS	:= ${WARNOPTS} ${TGTOPTS} -fPIC
LDFLAGS		:=
LIBS		:=
ifdef DEBUG
    CXXFLAGS	+= -O0 -g
    LDFLAGS	+= -rdynamic
else
    CXXFLAGS	+= -Os -g0 -DNDEBUG=1 -fomit-frame-pointer ${INLINEOPTS}
    LDFLAGS	+= -s -Wl,-gc-sections
endif
ifdef NOLIBSTDCPP
    LD		:= /usr/bin/gcc
    STAL_LIBS	:= -lsupc++
    LIBS	:= ${STAL_LIBS}
endif
BUILDDIR	:= /tmp/juraj/make/${NAME}
O		:= .o/

slib_lnk	= lib$1.so
slib_son	= lib$1.so.${MAJOR}
slib_tgt	= lib$1.so.${MAJOR}.${MINOR}
slib_flags	= -shared -Wl,-soname=$1
