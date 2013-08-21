
lib_LTLIBRARIES=libDDAGui.la DDAGui.la

BUILT_SOURCES=

libDDAGui_la_SOURCES=\
		AppDDAGuiPy.cpp \
		Command.cpp \
		PreCompiled.cpp \
		PreCompiled.h \
		Workbench.cpp \
		Workbench.h

includedir = @includedir@/Mod/DDA/Gui

# the library search path.
libDDAGui_la_LDFLAGS = -L../../../Base -L../../../App -L../../../Gui -L../App \
		$(QT_LIBS) $(all_libraries) \
		$(sim_ac_coin_ldflags) $(sim_ac_coin_libs) \
		$(sim_ac_soqt_ldflags) $(sim_ac_soqt_libs) \
		-version-info @LIB_CURRENT@:@LIB_REVISION@:@LIB_AGE@

libDDAGui_la_CPPFLAGS = -DDDAAppExport= -DDDAGuiExport=

libDDAGui_la_LIBADD   = \
		@BOOST_SYSTEM_LIB@ \
		-l@PYTHON_LIB@ \
		-lxerces-c \
		-lFreeCADBase \
		-lFreeCADApp \
		-lFreeCADGui \
		-lDDA

#--------------------------------------------------------------------------------------
# Loader of libDDAGui

DDAGui_la_SOURCES=\
		AppDDAGui.cpp

# the library search path.
DDAGui_la_LDFLAGS = $(libDDAGui_la_LDFLAGS) -module -avoid-version
DDAGui_la_CPPFLAGS = $(libDDAGui_la_CPPFLAGS)

DDAGui_la_LIBADD   = \
		$(libDDAGui_la_LIBADD) \
		-lDDAGui

DDAGui_la_DEPENDENCIES = libDDAGui.la

#--------------------------------------------------------------------------------------

# rule for Qt MetaObject Compiler:
moc_%.cpp: %.h
	$(QT_MOC) $< -o $(@F)

# rule for Qt MetaObject Compiler:
%.moc: %.h
	$(QT_MOC) $< -o $(@F)

# rules for Qt User Interface Compiler:
ui_%.h: %.ui
	$(QT_UIC) $< -o $(@F)

# rules for Qt Resource Compiler:
qrc_%.cpp: Resources/%.qrc
	$(QT_RCC) -name $(*F) $< -o $(@F)

# set the include path found by configure
AM_CXXFLAGS = -I$(top_srcdir)/src -I$(top_builddir)/src $(all_includes) $(QT_CXXFLAGS) \
		-I$(sim_ac_coin_includedir) -I$(sim_ac_soqt_includedir)

libdir = $(prefix)/Mod/DDA

CLEANFILES = $(BUILT_SOURCES)

EXTRA_DIST = \
		CMakeLists.txt \
		Resources/DDA.qrc

