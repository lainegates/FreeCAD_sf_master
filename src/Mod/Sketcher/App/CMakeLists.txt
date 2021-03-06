if(MSVC)
    add_definitions(-DFCAppSketcher -DHAVE_ACOSH -DHAVE_ASINH -DHAVE_ATANH)
else(MSVC)
    add_definitions(-DHAVE_LIMITS_H -DHAVE_CONFIG_H)
endif(MSVC)

include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_BINARY_DIR}/..
    ${Boost_INCLUDE_DIRS}
    ${OCC_INCLUDE_DIR}
    ${ZLIB_INCLUDE_DIR}
    ${PYTHON_INCLUDE_PATH}
    ${XERCESC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)
link_directories(${OCC_LIBRARY_DIR})

set(Sketcher_LIBS
    Part
    FreeCADApp
)

generate_from_xml(SketchObjectSFPy)
generate_from_xml(SketchObjectPy)
generate_from_xml(ConstraintPy)
generate_from_xml(SketchPy)


SET(Properties_SRCS
    PropertyConstraintList.cpp
    PropertyConstraintList.h
)
SOURCE_GROUP("Properties" FILES ${Properties_SRCS})


SET(Features_SRCS
    SketchObjectSF.cpp
    SketchObjectSF.h
    SketchObject.cpp
    SketchObject.h
)
SOURCE_GROUP("Features" FILES ${Features_SRCS})

SET(Datatypes_SRCS
    Constraint.cpp
    Constraint.h
    Sketch.cpp
    Sketch.h
)
SOURCE_GROUP("Datatypes" FILES ${Datatypes_SRCS})

SET(Python_SRCS
    SketchObjectSFPy.xml
    SketchObjectSFPyImp.cpp
    SketchObjectPy.xml
    SketchObjectPyImp.cpp
    ConstraintPyImp.cpp
    ConstraintPy.xml
    SketchPy.xml
    SketchPyImp.cpp
)
SOURCE_GROUP("Python" FILES ${Python_SRCS})


SET(FreeGCS_SRCS
    freegcs/qp_eq.h
    freegcs/GCS.cpp
    freegcs/GCS.h
    freegcs/Util.h
    freegcs/Geo.h
    freegcs/Constraints.cpp
    freegcs/Constraints.h
    freegcs/SubSystem.cpp
    freegcs/SubSystem.h
    freegcs/qp_eq.cpp
    freegcs/qp_eq.h
)
SOURCE_GROUP("FreeGCS" FILES ${FreeGCS_SRCS})

SET(SketchModule_SRCS
    AppSketcher.cpp
    AppSketcherPy.cpp
    PreCompiled.cpp
    PreCompiled.h
)
SOURCE_GROUP("Module" FILES ${SketchModule_SRCS})

SET(Sketcher_SRCS
    ${Features_SRCS}
    ${FreeGCS_SRCS}
    ${SketchModule_SRCS}
    ${Python_SRCS}
    ${Properties_SRCS}
    ${Datatypes_SRCS}
)

SET(Sketcher_Scripts
    Init.py
    SketcherExample.py
	TestSketcherApp.py
)

add_library(Sketcher SHARED ${Sketcher_SRCS})
target_link_libraries(Sketcher ${Sketcher_LIBS})


fc_target_copy_resource(Sketcher 
    ${CMAKE_SOURCE_DIR}/src/Mod/Sketcher
    ${CMAKE_BINARY_DIR}/Mod/Sketcher
    ${Sketcher_Scripts})

if(MSVC)
    set_target_properties(Sketcher PROPERTIES SUFFIX ".pyd")
    set_target_properties(Sketcher PROPERTIES DEBUG_OUTPUT_NAME "Sketcher_d")
    set_target_properties(Sketcher PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/Sketcher)
    set_target_properties(Sketcher PROPERTIES PREFIX "../")
elseif(MINGW)
    set_target_properties(Sketcher PROPERTIES SUFFIX ".pyd")
    set_target_properties(Sketcher PROPERTIES DEBUG_OUTPUT_NAME "Sketcher_d")
    set_target_properties(Sketcher PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/Sketcher)
    set_target_properties(Sketcher PROPERTIES PREFIX "")
else(MSVC)
    set_target_properties(Sketcher PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/Sketcher)
    set_target_properties(Sketcher PROPERTIES PREFIX "")
    set_target_properties(Sketcher PROPERTIES INSTALL_RPATH ${INSTALL_RPATH})
endif(MSVC)

INSTALL(TARGETS Sketcher DESTINATION lib)
