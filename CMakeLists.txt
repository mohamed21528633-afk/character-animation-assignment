cmake_minimum_required(VERSION 3.20)

project(student_character_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(N8RO_RELEASE "C:/N8RO" CACHE PATH "N8RO release root")

add_library(student_character_plugin SHARED
    src/StudentController.cpp
)

target_include_directories(student_character_plugin PRIVATE
    include
    "${N8RO_RELEASE}/include/astlib"
    "${N8RO_RELEASE}/include/astsim"
    "${N8RO_RELEASE}/include/astdb"
    "${N8RO_RELEASE}/include/n8ro-db"
)

target_compile_definitions(student_character_plugin PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    ARKHEON_ASTLIB_EXPORTS
)

target_link_directories(student_character_plugin PRIVATE
    "${N8RO_RELEASE}/lib"
)

target_link_libraries(student_character_plugin PRIVATE
    arkheon-astlib
    arkheon-astdb
    n8ro-db
    arkheon-astsim
)

set_target_properties(student_character_plugin PROPERTIES
    OUTPUT_NAME "character_plugin_230201904"
    PREFIX ""
)
