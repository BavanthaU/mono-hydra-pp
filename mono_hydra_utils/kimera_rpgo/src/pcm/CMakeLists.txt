target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/graph_utils.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/geometry_utils.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pcm.cpp
)
add_subdirectory(max_clique_finder)
