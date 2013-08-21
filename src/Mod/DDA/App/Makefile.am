
lib_LTLIBRARIES=libDDA.la DDA.la

libDDA_la_SOURCES=\
		AppDDAPy.cpp \
		PreCompiled.cpp \
		PreCompiled.h

includedir = @includedir@/Mod/DDA/App

# the library search path.
libDDA_la_LDFLAGS = -L../../../Base -L../../../App $(all_libraries) \
		-version-info @LIB_CURRENT@:@LIB_REVISION@:@LIB_AGE@
libDDA_la_CPPFLAGS = -DDDAAppExport=

libDDA_la_LIBADD   = \
		@BOOST_SYSTEM_LIB@ \
		-l@PYTHON_LIB@ \
		-lxerces-c \
		-lFreeCADBase \
		-lFreeCADApp

#--------------------------------------------------------------------------------------
# Loader of libDDA

DDA_la_SOURCES=\
		AppDDA.cpp

# the library search path.
DDA_la_LDFLAGS = $(libDDA_la_LDFLAGS) -module -avoid-version
DDA_la_CPPFLAGS = $(libDDA_la_CPPFLAGS)

DDA_la_LIBADD   = \
		$(libDDA_la_LIBADD) \
		-lDDA

DDA_la_DEPENDENCIES = libDDA.la

#--------------------------------------------------------------------------------------

# set the include path found by configure
AM_CXXFLAGS = -I$(top_srcdir)/src -I$(top_builddir)/src $(all_includes)

libdir = $(prefix)/Mod/DDA

EXTRA_DIST = \
		CMakeLists.txt
