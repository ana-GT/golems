cmake_minimum_required( VERSION 2.8 )

project( trajectories )

add_library( tkunz Path.cpp Trajectory.cpp Path.h Trajectory.h )
set_target_properties( tkunz PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )