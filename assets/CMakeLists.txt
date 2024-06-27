add_custom_target(assets
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/geos.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/geos.gltf
        ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/geos.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/geos.gltf
)
