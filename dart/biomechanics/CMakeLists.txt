# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "biomechanics headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/biomechanics.hpp"
  "dart/biomechanics/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/biomechanics.hpp
  DESTINATION include/dart/biomechanics
  COMPONENT headers
)