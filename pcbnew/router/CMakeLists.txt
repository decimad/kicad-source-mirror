include_directories( BEFORE ${INC_BEFORE} )

include_directories(
    ./
    ../
    ../../include
    ../../3d-viewer
    ../../pcbnew
    ../../polygon
    ${INC_AFTER}
)

set( PCBNEW_PNS_SRCS
    time_limit.cpp
    pns_kicad_iface.cpp
    pns_algo_base.cpp
    pns_diff_pair.cpp
    pns_diff_pair_placer.cpp
    pns_dp_meander_placer.cpp
    pns_dragger.cpp
    pns_item.cpp
    pns_itemset.cpp
    pns_line.cpp
    pns_line_placer.cpp
    pns_logger.cpp
    pns_meander.cpp
    pns_meander_placer.cpp
    pns_meander_placer_base.cpp
    pns_meander_skew_placer.cpp
    pns_node.cpp
    pns_optimizer.cpp
    pns_router.cpp
    pns_routing_settings.cpp
    pns_shove.cpp
    pns_sizes_settings.cpp
    pns_solid.cpp
    pns_tool_base.cpp
    pns_topology.cpp
    pns_tune_status_popup.cpp
    pns_utils.cpp
    pns_via.cpp
    pns_walkaround.cpp
    router_preview_item.cpp
    router_tool.cpp
    length_tuner_tool.cpp
)

set( PCBNEW_PNS_HEADERS
    direction.h
    length_tuner_tool.h
    pns_algo_base.h
    pns_debug_decorator.h
    pns_diff_pair.h
    pns_diff_pair_placer.h
    pns_dp_meander_placer.h
    pns_dragger.h
    pns_index.h
    pns_item.h
    pns_itemset.h
    pns_joint.h
    pns_kicad_iface.h
    pns_layerset.h
    pns_line.h
    pns_line_placer.h
    pns_logger.h
    pns_meander.h
    pns_meander_placer.h
    pns_meander_placer_base.h
    pns_meander_skew_placer.h
    pns_node.h
    pns_optimizer.h
    pns_placement_algo.h
    pns_router.h
    pns_routing_settings.h
    pns_segment.h
    pns_shove.h
    pns_sizes_settings.h
    pns_solid.h
    pns_tool_base.h
    pns_topology.h
    pns_tune_status_popup.h
    pns_utils.h
    pns_via.h
    pns_walkaround.h
    range.h
    ranged_num.h
    router_preview_item.h
    router_tool.h
    time_limit.h
)

add_library( pnsrouter STATIC ${PCBNEW_PNS_SRCS} ${PCBNEW_PNS_HEADERS} )

if( MSVC )
    set_target_properties( pnsrouter PROPERTIES FOLDER libs )
endif()
