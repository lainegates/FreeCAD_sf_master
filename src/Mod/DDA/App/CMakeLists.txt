
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${OCC_INCLUDE_DIR}
    ${PYTHON_INCLUDE_PATH}
    ${ZLIB_INCLUDE_DIR}
    ${XERCESC_INCLUDE_DIR}
)

set(DDA_LIBS
    FreeCADApp
)

SET(DDA_SRCS
    AppDDA.cpp
    AppDDAPy.cpp
    PreCompiled.cpp
    PreCompiled.h
)

add_library(DDA SHARED ${DDA_SRCS})
target_link_libraries(DDA ${DDA_LIBS})


fc_target_copy_resource(DDA 
    ${CMAKE_SOURCE_DIR}/src/Mod/DDA
    ${CMAKE_BINARY_DIR}/Mod/DDA
    Init.py)

if(MSVC)
    set_target_properties(DDA PROPERTIES SUFFIX ".pyd")
    set_target_properties(DDA PROPERTIES DEBUG_OUTPUT_NAME "DDA_d")
    set_target_properties(DDA PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/DDA)
    set_target_properties(DDA PROPERTIES PREFIX "../")
elseif(MINGW)
    set_target_properties(DDA PROPERTIES SUFFIX ".pyd")
    set_target_properties(DDA PROPERTIES DEBUG_OUTPUT_NAME "DDA_d")
    set_target_properties(DDA PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/DDA)
    set_target_properties(DDA PROPERTIES PREFIX "")
else(MSVC)
    set_target_properties(DDA PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Mod/DDA)
    set_target_properties(DDA PROPERTIES PREFIX "")
    set_target_properties(Fem PROPERTIES INSTALL_RPATH ${INSTALL_RPATH})
endif(MSVC)

install(TARGETS DDA DESTINATION lib)
