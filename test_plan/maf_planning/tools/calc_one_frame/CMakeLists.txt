add_subdirectory (thirdparty)
set (CALC_ONE_FRAME_SRCS main.cpp)

set (THREADS_PREFER_PTHREAD_FLAG ON)
find_package (Threads REQUIRED)

add_subdirectory (msg_cache)
add_subdirectory (calc_one_frame_npp)

add_executable (fpp_sim ${CALC_ONE_FRAME_SRCS})
target_link_libraries (
  fpp_sim
  calc_one_frame_npp
  glog
  gflags
  Threads::Threads
  absl::failure_signal_handler
  ${CONAN_LIBS}
)
install(TARGETS fpp_sim DESTINATION bin)
