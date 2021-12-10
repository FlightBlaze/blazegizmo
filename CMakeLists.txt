cmake_minimum_required (VERSION 3.10.2)
set (CMAKE_CXX_STANDARD 17)

project ("blazegizmo")

set (BLAZEGIZMO_SOURCES
    "include/blazegizmo.hh"
    "source/blazegizmo.cc")

set (BLAZEVG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../blazevg/" )
set (GLM_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../glm/" )

if (NOT TARGET blazevg)
    add_subdirectory (${BLAZEVG_PATH} "${CMAKE_CURRENT_BINARY_DIR}/blazevg/")
endif()

if (NOT TARGET glm)
    add_subdirectory (${GLM_PATH} "${CMAKE_CURRENT_BINARY_DIR}/glm/")
endif()

add_library (blazegizmo
             ${BLAZEGIZMO_SOURCES})

target_include_directories (blazegizmo PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include/"
        "${BLAZEVG_PATH}/include/"
        ${GLM_PATH})

target_link_libraries (blazegizmo blazevg glm)
