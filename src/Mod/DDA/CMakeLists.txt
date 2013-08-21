
add_subdirectory(App)
if(FREECAD_BUILD_GUI)
    add_subdirectory(Gui)
endif(FREECAD_BUILD_GUI)


SET(DDA_SRCS
	Base.py
	boost_python-vc90-mt-1_39.dll
	checkConcave_and_triangulate.py
	CMakeLists.txt
	dc.ps
	dc_Ff.c
	DDA-preferences.ui
	DDA.dox
	DDACalcTools.pyd
	DDADatabase.py
	DDADisplay.py
	DDAGui.pyd
	DDAPanel.py
	DDAShapes.py
	DDAToolbars.py
	DDATools.py
	DDA_rc.py
	DFCalculation.py
	df_Ff.c
	DGPlayer.py
	dg_ff.c
	dl.ps
	drawGui.py
	fcgeo.py
	fcvec.py
	Ff.c
	Graph.py
	Init.py
	InitGui.py
	interfaceTools.py
	LICENCE.lgpl.txt
	loadDataTools.py
	Makefile.am
	PyDDA.pyd
	PyTriangulation.pyd
	storeScene.py
	testInterfaceTools.py
	test_rc.py
	TrackerTools.py
	ui_DC.py
	ui_DF.py
	ui_DFCalcProcess.py
	ui_DL.py
	ui_tunnelBolts.py
	ui_TunnelSelection.py
	UserGuide.docx
	WorkingPlane1.py
	__init__.py

)

SOURCE_GROUP("" FILES ${Draft_SRCS})

SET(DDALibs_SRCS
    draftlibs/dxfColorMap.py
    draftlibs/dxfImportObjects.py
    draftlibs/dxfLibrary.py
    draftlibs/dxfReader.py
    draftlibs/__init__.py
)
SOURCE_GROUP("draftlibs" FILES ${DraftLibs_SRCS})

SET(all_files ${DDA_SRCS} ${DDALibs_SRCS})

ADD_CUSTOM_TARGET(DDA ALL
    SOURCES ${all_files}
)

fc_copy_sources(DDA "${CMAKE_BINARY_DIR}/Mod/DDA" ${all_files})


INSTALL(
    FILES
        ${DDALibs_SRCS}
    DESTINATION
        Mod/DDA/draftlibs
)
INSTALL(
    FILES
        ${DDA_SRCS}
    DESTINATION
        Mod/DDA
)
