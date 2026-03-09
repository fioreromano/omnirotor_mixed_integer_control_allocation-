#
# Copyright (c) 2015-2024 LAAS/CNRS
# All rights reserved.
#
# Redistribution and use  in source  and binary  forms,  with or without
# modification, are permitted provided that the following conditions are
# met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
#					Anthony Mallet on Tue Aug 11 2015
#          		Youssef Aboudorra on Fri Jul 05 2024
#
lib_LTLIBRARIES =

# omnimorph codels library
lib_LTLIBRARIES += libomnimorph_codels.la

libomnimorph_codels_la_SOURCES  =	omnimorph_c_types.h
libomnimorph_codels_la_SOURCES +=	omnimorph_codels.c
libomnimorph_codels_la_SOURCES +=	omnimorph_main_codels.c
libomnimorph_codels_la_SOURCES +=	controller.cc
libomnimorph_codels_la_SOURCES +=	geometry.cc
libomnimorph_codels_la_SOURCES +=	codels.h

libomnimorph_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libomnimorph_codels_la_LIBADD   =	$(requires_LIBS)
libomnimorph_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libomnimorph_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libomnimorph_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl mappings
BUILT_SOURCES=	omnimorph_c_types.h
CLEANFILES=	${BUILT_SOURCES}

omnimorph_c_types.h: ${top_srcdir}/omnimorph.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/omnimorph_c_types.h.d
