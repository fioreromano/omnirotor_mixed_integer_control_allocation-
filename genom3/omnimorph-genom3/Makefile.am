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
ACLOCAL_AMFLAGS=	-I autoconf

AM_CPPFLAGS = -I/usr/local/include -I/home/fiore/openrobots/include
AM_CXXFLAGS = -std=c++17 -O2
# IDL source files
idldir=		$(datadir)/idl/omnimorph
nobase_dist_idl_DATA= omnimorph.gen

pkgconfigdir=	$(libdir)/pkgconfig
pkgconfig_DATA=
pkgconfig_DATA+= omnimorph-genom3.pc

# we don't want generated templates in the distribution
#
DIST_SUBDIRS=		codels
SUBDIRS=		${DIST_SUBDIRS}

# recursion into templates directories configured with --with-templates
#
SUBDIRS+=		${AG_TEMPLATES_SUBDIRS}

distclean-local:
	-rm -rf ${AG_TEMPLATES_SUBDIRS}

# a rule to invoke skeleton merge mode
#
merge: merge-interactive
merge-%:
	cd ${top_srcdir} && ${GENOM3}  \
	  skeleton -l 'c' -m $* omnimorph.gen

# documentation
dist_doc_DATA=	README.html README.adoc

%.html: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTOR_ARGS) -b html5 -o $@ $<
