if(CSL_ENABLE_IPOPT)
  add_executable(ipopt_convex_client tests/installed/ipopt_convex_client.cpp)
  target_link_libraries(ipopt_convex_client PRIVATE CSL::ipopt_convex)
  add_test(NAME ipopt_convex_client COMMAND ipopt_convex_client)
  set(ipopt_test_work "${CMAKE_CURRENT_BINARY_DIR}/ipopt-client-options")
  file(MAKE_DIRECTORY "${ipopt_test_work}")
  file(WRITE "${ipopt_test_work}/ipopt.opt" "max_iter 250\n")
  set_tests_properties(ipopt_convex_client PROPERTIES
    WORKING_DIRECTORY "${ipopt_test_work}"
    ENVIRONMENT "OPENBLAS_NUM_THREADS=1;OMP_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
