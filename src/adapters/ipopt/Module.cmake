if(CSL_ENABLE_IPOPT)
  csl_add_library(csl_ipopt_convex STATIC src/adapters/convex/ipopt.cpp)
  target_link_libraries(csl_ipopt_convex PUBLIC CSL::convex CSL::registry PRIVATE Ipopt::Ipopt)
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindIpopt.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/CSL")
endif()
