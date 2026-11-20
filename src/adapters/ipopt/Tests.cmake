if(CSL_ENABLE_IPOPT)
  add_executable(ipopt_convex_client tests/installed/ipopt_convex_client.cpp)
  target_link_libraries(ipopt_convex_client PRIVATE CSL::ipopt_convex)
  add_test(NAME ipopt_convex_client COMMAND ipopt_convex_client)
  set_tests_properties(ipopt_convex_client PROPERTIES
    ENVIRONMENT "OPENBLAS_NUM_THREADS=1;OMP_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
