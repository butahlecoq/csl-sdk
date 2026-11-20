find_path(Ipopt_INCLUDE_DIR NAMES IpStdCInterface.h
  PATH_SUFFIXES include/coin-or include/coin coin-or coin include)
find_library(Ipopt_LIBRARY NAMES ipopt libipopt)

if(Ipopt_INCLUDE_DIR AND EXISTS "${Ipopt_INCLUDE_DIR}/IpoptConfig.h")
  file(STRINGS "${Ipopt_INCLUDE_DIR}/IpoptConfig.h" _ipopt_version
    REGEX "^#define IPOPT_VERSION \"[0-9]+\\.[0-9]+\\.[0-9]+\"$")
  if(_ipopt_version)
    string(REGEX REPLACE "^.*\"([0-9.]+)\"$" "\\1" Ipopt_VERSION "${_ipopt_version}")
  endif()
endif()
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ipopt
  REQUIRED_VARS Ipopt_LIBRARY Ipopt_INCLUDE_DIR Ipopt_VERSION VERSION_VAR Ipopt_VERSION)
mark_as_advanced(Ipopt_LIBRARY Ipopt_INCLUDE_DIR)
if(Ipopt_FOUND AND NOT TARGET Ipopt::Ipopt)
  add_library(Ipopt::Ipopt UNKNOWN IMPORTED)
  set_target_properties(Ipopt::Ipopt PROPERTIES
    IMPORTED_LOCATION "${Ipopt_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${Ipopt_INCLUDE_DIR}")
endif()
