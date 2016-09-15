
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/pcbnew
    ${Boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )


add_executable( container_test
    EXCLUDE_FROM_ALL
    container_test.cpp
    )

set_target_properties( container_test PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD 1 )

if( MSVC )
    set_target_properties( container_test PROPERTIES FOLDER various )
endif()

target_link_libraries( container_test
    common
    polygon
    bitmaps
    ${wxWidgets_LIBRARIES}
    )

add_executable( test-nm-biu-to-ascii-mm-round-tripping
    EXCLUDE_FROM_ALL
    test-nm-biu-to-ascii-mm-round-tripping.cpp
    )

set_target_properties( test-nm-biu-to-ascii-mm-round-tripping PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD 1 )

if( MSVC )
    set_target_properties( test-nm-biu-to-ascii-mm-round-tripping PROPERTIES FOLDER various )
endif()

add_executable( property_tree
    EXCLUDE_FROM_ALL
    property_tree.cpp
    ../common/richio.cpp
    ../common/dsnlexer.cpp
    ../common/ptree.cpp
    )
target_link_libraries( property_tree
    ${wxWidgets_LIBRARIES}
    )

set_target_properties( property_tree PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD 1 )

if( MSVC )
    set_target_properties( property_tree PROPERTIES FOLDER various )
endif()
