if(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
  # using Visual Studio C++
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /fp:fast /Ox /Ob2 /Oi /Ot /GT /std:c17 /experimental:c11atomics /experimental:module /Zm2000 /Zc:twoPhase- /D_HAS_STD_BYTE=0 /D_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING /D_HAS_STD_BOOLEAN=0 /bigobj /openmp:llvm")
endif()


add_executable(KSuE3_velocity_control KSuE3_velocity_control.c)
target_link_libraries(KSuE3_velocity_control soem)
install(TARGETS KSuE3_velocity_control DESTINATION bin)
