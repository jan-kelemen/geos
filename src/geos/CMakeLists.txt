add_executable(geos)

target_sources(geos
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/geos.m.cpp
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
        glm::glm
        SDL2::SDL2main
        spdlog::spdlog
    PRIVATE
        project-options
)

set_property(TARGET geos 
    PROPERTY 
        VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

