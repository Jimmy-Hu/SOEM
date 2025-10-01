if(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
  # using Visual Studio C++
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /fp:fast /Ox /Ob2 /Oi /Ot /GT /std:c17 /experimental:c11atomics /experimental:module /Zm2000 /Zc:twoPhase- /D_HAS_STD_BYTE=0 /D_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING /D_HAS_STD_BOOLEAN=0 /bigobj /openmp:llvm")
endif()


add_executable(driver_reset_objects driver_reset_objects.c)
target_link_libraries(driver_reset_objects soem)
install(TARGETS driver_reset_objects DESTINATION bin)
