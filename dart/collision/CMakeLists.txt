# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision headers" ${hdrs})
# TODO: remove below line once the files are completely removed.
list(REMOVE_ITEM header_names "Option.hpp" "Result.hpp")
set(
  header_names
  ${header_names}
  dart/dart.hpp
)
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/collision.hpp"
  "dart/collision/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/collision.hpp
  DESTINATION include/dart/collision
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/collision/detail
  COMPONENT headers
)

# Add subdirectories
add_subdirectory(dart)

dart_format_add(
  RaycastOption.hpp
  RaycastOption.cpp
  RaycastResult.hpp
  RaycastResult.cpp
  SmartPointer.hpp
)
