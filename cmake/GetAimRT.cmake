include(FetchContent)

message(STATUS "get aimrt ...")

FetchContent_Declare(
  aimrt #
  GIT_REPOSITORY https://github.com/AimRT/AimRT.git
  GIT_TAG v0.9.1)

FetchContent_GetProperties(aimrt)
if(NOT aimrt_POPULATED)
  set(AIMRT_BUILD_TESTS
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_EXAMPLES
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_DOCUMENT
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_RUNTIME
      ON
      CACHE BOOL "")
  set(AIMRT_BUILD_CLI_TOOLS
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_WITH_PROTOBUF
      ON
      CACHE BOOL "")
  set(AIMRT_USE_LOCAL_PROTOC_COMPILER
      OFF
      CACHE BOOL "")
  set(AIMRT_USE_PROTOC_PYTHON_PLUGIN
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_WITH_ROS2
      ON
      CACHE BOOL "")
  set(AIMRT_BUILD_NET_PLUGIN
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_ROS2_PLUGIN
      ON
      CACHE BOOL "")
  set(AIMRT_BUILD_MQTT_PLUGIN
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_ZENOH_PLUGIN
      OFF
      CACHE BOOL "")
  set(AIMRT_BUILD_ICEORYX_PLUGIN
      OFF
      CACHE BOOL "")

  FetchContent_MakeAvailable(aimrt)

  # libunifex (AimRT dependency) uses directory-level:
  #   add_compile_options(-Wall -Wextra -pedantic -Werror)
  # in cmake/unifex_env.cmake. On systems with newer liburing headers that use
  # anonymous structs and zero-size arrays (valid in C, not C++ pedantic),
  # this breaks compilation.
  #
  # Fix: target_compile_options are appended AFTER directory-level options in
  # GCC's command line. -Wno-error=pedantic overrides -Werror for pedantic
  # warnings specifically. -Wno-pedantic suppresses the warning entirely.
  # Both flags together guarantee the fix regardless of GCC flag precedence.
  if(TARGET unifex)
    target_compile_options(unifex PRIVATE -Wno-pedantic -Wno-error=pedantic)
  endif()
endif()
