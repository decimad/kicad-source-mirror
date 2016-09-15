include_directories(
    "${CMAKE_SOURCE_DIR}/lib_dxf"
    "${CMAKE_SOURCE_DIR}/utils/idftools"
    ${OPENGL_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
  )

link_directories(
    "${CMAKE_BINARY_DIR}/lib_dxf"
  )

add_library( idf3 STATIC
    idf_helpers.cpp idf_common.cpp idf_outlines.cpp
    idf_parser.cpp vrml_layer.cpp )

if( MSVC )
    set_target_properties( idf3 PROPERTIES FOLDER libs )
endif()

add_executable( idfcyl idf_cylinder.cpp )

if( MSVC )
    set_target_properties( idfcyl PROPERTIES FOLDER tools )
endif()

add_executable( idfrect idf_rect.cpp )

if( MSVC )
    set_target_properties( idfrect PROPERTIES FOLDER tools )
endif()

add_executable( dxf2idf dxf2idfmain.cpp dxf2idf.cpp )

if( MSVC )
    set_target_properties( dxf2idf PROPERTIES FOLDER tools )
endif()

add_executable( idf2vrml idf2vrml.cpp )

if( MSVC )
    set_target_properties( idf2vrml PROPERTIES FOLDER tools )
endif()

target_link_libraries( dxf2idf lib_dxf idf3 ${wxWidgets_LIBRARIES} )

target_link_libraries( idf2vrml idf3 ${OPENGL_LIBRARIES} ${wxWidgets_LIBRARIES} )

if( APPLE )
    # puts binaries into the *.app bundle while linking
    set_target_properties( idfcyl idfrect dxf2idf idf2vrml PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${OSX_BUNDLE_BUILD_BIN_DIR}
        )
else()
    install( TARGETS idfcyl idfrect dxf2idf idf2vrml
        DESTINATION ${KICAD_BIN}
        COMPONENT binary )
endif()
