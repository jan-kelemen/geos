add_executable(geos)

target_sources(geos
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/application.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/application.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/camera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/camera.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/geos.m.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mesh.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mesh.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/physics.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/physics.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/scene.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/scene.hpp
)

target_include_directories(geos
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(geos
    PRIVATE
        cppext
        imgui_impl
        vkrndr
    PRIVATE
        boost::boost
        Bullet::Bullet
        EnTT::EnTT
        glm::glm
        SDL2::SDL2main
        spdlog::spdlog
    PRIVATE
        project-options
)

add_dependencies(geos shaders)

add_custom_target(shaders
    DEPENDS
        ${CMAKE_CURRENT_BINARY_DIR}/scene.frag.spv
        ${CMAKE_CURRENT_BINARY_DIR}/scene.vert.spv
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/scene.vert.spv
    COMMAND 
        ${GLSLC_EXE} ${CMAKE_CURRENT_SOURCE_DIR}/shaders/scene.vert -o ${CMAKE_CURRENT_BINARY_DIR}/scene.vert.spv
    DEPENDS 
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders/scene.vert
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/scene.frag.spv
    COMMAND 
        ${GLSLC_EXE} ${CMAKE_CURRENT_SOURCE_DIR}/shaders/scene.frag -o ${CMAKE_CURRENT_BINARY_DIR}/scene.frag.spv
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders/scene.frag
)

set_property(TARGET geos 
    PROPERTY 
        VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/physics.cpp 
    PROPERTIES
        SKIP_LINTING ON
)
