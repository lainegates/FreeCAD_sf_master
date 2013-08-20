
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
    ${COIN3D_INCLUDE_DIR}
    ${QT_INCLUDE_DIR}
    ${ZLIB_INCLUDE_DIR}
    ${SOQT_INCLUDE_DIR}
    ${PYTHON_INCLUDE_PATH}
    ${XERCESC_INCLUDE_DIR}
)

set(DDAGui_LIBS
    DDA
    FreeCADGui
)

set(DDAGui_MOC_HDRS
    ddapostplayer.h
)
qt4_wrap_cpp(DDAGui_MOC_SRCS ${DDAGui_MOC_HDRS})
SOURCE_GROUP("Moc" FILES ${DDAGui_MOC_SRCS})

set(DDAGui_UIC_SRCS
    ddapostplayer.ui
)
qt4_wrap_ui(DDAGui_UIC_HDRS ${DDAGui_UIC_SRCS})
SET(DDAGui_DLG_SRCS
    ${DDAGui_UIC_HDRS}
    ddapostplayer.ui
    ddapostplayer.cpp
    ddapostplayer.h
)
SOURCE_GROUP("Dialogs" FILES ${DDAGui_DLG_SRCS})

qt4_add_resources(DDA_QRC_SRCS Resources/DDA.qrc)

SET(DDAGui_SRCS
    ${DDA_QRC_SRCS}
	${DDAGui_DLG_SRCS}
	${DDAGui_MOC_SRCS}
    AppDDAGui.cpp
    AppDDAGuiPy.cpp
    Command.cpp
    PreCompiled.cpp
    PreCompiled.h
    Workbench.cpp
    Workbench.h
	FrameMaker.cpp
	FrameMaker.h
	FrameReader.cpp
	FrameReader.h
	GraphTools.cpp
	GraphTools.h
	Triangulate.cpp
	Triangulate.h
)

add_library(DDAGui SHARED ${DDAGui_SRCS})
target_link_libraries(DDAGui ${DDAGui_LIBS})

fc_target_copy_resource(DDAGui 
    ${CMAKE_SOURCE_DIR}/src/Mod/DDA
    ${CMAKE_BINARY_DIR}/Mod/DDA
    InitGui.py)

if(MSVC)
    set_target_properties(DDAGui PROPERTIES SUFFIX ".pyd")
    set_target_properties(DDAGui PROPERTIES DEBUG_OUTPUT_NAME "DDAGui_d")
    set_target_properties(DDAGui PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/DDA)
    set_target_properties(DDAGui PROPERTIES PREFIX "../")
elseif(MINGW)
    set_target_properties(DDAGui PROPERTIES SUFFIX ".pyd")
    set_target_properties(DDAGui PROPERTIES DEBUG_OUTPUT_NAME "DDAGui_d")
    set_target_properties(DDAGui PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/DDA)
    set_target_properties(DDAGui PROPERTIES PREFIX "")
else(MSVC)
    set_target_properties(DDAGui PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/DDA)
    set_target_properties(DDAGui PROPERTIES PREFIX "")
    set_target_properties(DDAGui PROPERTIES INSTALL_RPATH ${INSTALL_RPATH})
endif(MSVC)

install(TARGETS DDAGui DESTINATION lib)
