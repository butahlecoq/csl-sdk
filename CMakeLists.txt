cmake_minimum_required(VERSION 3.31)
project(CSL VERSION 0.1.1 LANGUAGES C CXX)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)
option(CSL_DEPENDENCY_PROBES "Build the isolated pinned native dependency probes" OFF)

option(CSL_BUILD_NOMINAL_TUBE_CAPTURE "Build the source-bound closed-H nominal tube capture tool" OFF)
option(CSL_ENABLE_CASADI "Build the optional native C++ CasADi/IPOPT history optimizers" OFF)
option(CSL_BUILD_HISTORY_MEMBERSHIP_2D_EXAMPLE "Build the source-cost history membership 2D example" OFF)
if(CSL_ENABLE_CASADI)
  include(cmake/CasadiDependencies.cmake)
endif()
option(CSL_ENABLE_HIGHS "Build the native HiGHS LP/convex adapters" ON)
option(CSL_ENABLE_QHULL "Build the native Qhull converter (requires HiGHS)" ${CSL_ENABLE_HIGHS})
if(CSL_ENABLE_QHULL AND NOT CSL_ENABLE_HIGHS)
  message(FATAL_ERROR "CSL_ENABLE_QHULL requires CSL_ENABLE_HIGHS for its default LP provider")
endif()
if(CSL_DEPENDENCY_PROBES AND (NOT BUILD_TESTING OR NOT CSL_ENABLE_HIGHS OR NOT CSL_ENABLE_QHULL))
  message(FATAL_ERROR "Dependency probes require tests and both native adapters")
endif()
find_package(Threads REQUIRED)
include(cmake/ModulePolicy.cmake)
include(cmake/JsonDependencies.cmake)

# Include module-owned declarations in the project scope so dependency pins and
# package state remain shared. The same order is used for focused test ownership.
set(csl_modules contracts numeric optimization registry sets model geometry problem controllers
  reachability synthesis verification simulation input execution artifacts adapters/highs adapters/polyhedral application experiments)
foreach(module IN LISTS csl_modules)
  include(src/${module}/Module.cmake)
endforeach()
if(CSL_BUILD_NOMINAL_TUBE_CAPTURE)
  if(NOT CSL_ENABLE_HIGHS OR NOT CSL_ENABLE_QHULL)
    message(FATAL_ERROR "The nominal H tube capture tool requires the native HiGHS and Qhull adapters")
  endif()
  add_executable(nominal_tube_capture tools/nominal_tube_capture.cpp)
  target_link_libraries(nominal_tube_capture PRIVATE
    CSL::nominal_tube CSL::affine_sum CSL::qhull_conversion CSL::highs_lp)
  target_include_directories(nominal_tube_capture SYSTEM PRIVATE ${csl_json_SOURCE_DIR}/include)
  target_compile_features(nominal_tube_capture PRIVATE cxx_std_20)
  set_target_properties(nominal_tube_capture PROPERTIES CXX_EXTENSIONS OFF)
endif()
add_executable(csl_module_composition_catalogue tools/module_composition_catalogue.cpp)
target_link_libraries(csl_module_composition_catalogue PRIVATE
  CSL::hull CSL::affine_sum CSL::phase_one CSL::strategy_ts01 CSL::finite_action_nominal CSL::approximation
  CSL::quickhull_2d CSL::legacy_btulshor CSL::generator_adapter CSL::terminal_minimizers)
target_include_directories(csl_module_composition_catalogue PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/support)
if(CSL_ENABLE_HIGHS)
  target_link_libraries(csl_module_composition_catalogue PRIVATE CSL::highs_lp)
  target_link_libraries(csl_module_composition_catalogue PRIVATE CSL::highs_convex)
  target_compile_definitions(csl_module_composition_catalogue PRIVATE CSL_CATALOGUE_HAS_HIGHS=1)
  target_compile_definitions(csl_module_composition_catalogue PRIVATE CSL_CATALOGUE_HAS_HIGHS_CONVEX=1)
else()
  target_compile_definitions(csl_module_composition_catalogue PRIVATE CSL_CATALOGUE_HAS_HIGHS=0)
  target_compile_definitions(csl_module_composition_catalogue PRIVATE CSL_CATALOGUE_HAS_HIGHS_CONVEX=0)
endif()
if(CSL_ENABLE_QHULL)
  target_link_libraries(csl_module_composition_catalogue PRIVATE CSL::qhull_conversion)
  target_compile_definitions(csl_module_composition_catalogue PRIVATE CSL_CATALOGUE_HAS_QHULL=1)
else()
  target_compile_definitions(csl_module_composition_catalogue PRIVATE CSL_CATALOGUE_HAS_QHULL=0)
endif()
set_target_properties(csl_module_composition_catalogue PROPERTIES CXX_EXTENSIONS OFF)

if(CSL_ENABLE_HIGHS AND CSL_ENABLE_QHULL)
  add_executable(csl_strategy_a_each_choice_pipeline tools/strategy_a_each_choice_pipeline.cpp)
  target_link_libraries(csl_strategy_a_each_choice_pipeline PRIVATE
    CSL::strategy_a CSL::affine_sum CSL::polyhedral CSL::qhull_conversion CSL::highs_lp CSL::highs_convex)
  target_compile_definitions(csl_strategy_a_each_choice_pipeline PRIVATE
    CSL_PIPELINE_HAS_HIGHS=1 CSL_PIPELINE_HAS_QHULL=1)
  set_target_properties(csl_strategy_a_each_choice_pipeline PROPERTIES CXX_EXTENSIONS OFF)
endif()

add_executable(csl_fixed_tail_inner_region tools/fixed_tail_inner_region.cpp)
target_link_libraries(csl_fixed_tail_inner_region PRIVATE CSL::inner_template)
target_include_directories(csl_fixed_tail_inner_region SYSTEM PRIVATE ${csl_json_SOURCE_DIR}/include)
target_compile_features(csl_fixed_tail_inner_region PRIVATE cxx_std_20)
set_target_properties(csl_fixed_tail_inner_region PROPERTIES CXX_EXTENSIONS OFF)

add_executable(csl_closed_loop_native_policy tools/closed_loop_native_policy.cpp)
target_link_libraries(csl_closed_loop_native_policy PRIVATE CSL::finite_action_nominal CSL::polyhedral
  CSL::affine_sum CSL::phase_one CSL::history_cost CSL::feedback_geometry)
if(CSL_ENABLE_CASADI)
  target_link_libraries(csl_closed_loop_native_policy PRIVATE CSL::casadi_fixed_tail)
  target_compile_definitions(csl_closed_loop_native_policy PRIVATE CSL_NATIVE_HAS_CASADI=1)
endif()

if(CSL_ENABLE_HIGHS)
  target_link_libraries(csl_closed_loop_native_policy PRIVATE CSL::highs_lp CSL::highs_convex)
  target_compile_definitions(csl_closed_loop_native_policy PRIVATE CSL_NATIVE_HAS_HIGHS=1)
else()
  target_compile_definitions(csl_closed_loop_native_policy PRIVATE CSL_NATIVE_HAS_HIGHS=0)
endif()
if(CSL_ENABLE_HIGHS AND CSL_ENABLE_QHULL)
  target_link_libraries(csl_closed_loop_native_policy PRIVATE CSL::qhull_conversion)
endif()
target_compile_definitions(csl_closed_loop_native_policy PRIVATE
  CSL_NATIVE_HAS_QHULL=$<BOOL:${CSL_ENABLE_QHULL}>
  CSL_NATIVE_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
target_include_directories(csl_closed_loop_native_policy SYSTEM PRIVATE ${csl_json_SOURCE_DIR}/include)
set_target_properties(csl_closed_loop_native_policy PROPERTIES CXX_EXTENSIONS OFF)
if(CSL_ENABLE_CASADI)
  add_executable(history_optimizer_2d_example tools/history_optimizer_2d_example.cpp)
  target_link_libraries(history_optimizer_2d_example PRIVATE CSL::casadi_superscs_history_optimizer)
  set_target_properties(history_optimizer_2d_example PROPERTIES CXX_EXTENSIONS OFF)
  csl_strict_warnings(history_optimizer_2d_example)
endif()
if(CSL_BUILD_HISTORY_MEMBERSHIP_2D_EXAMPLE)
  if(NOT CSL_ENABLE_CASADI)
    message(FATAL_ERROR "The history membership example requires CSL_ENABLE_CASADI")
  endif()
  execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE CSL_HISTORY_MEMBERSHIP_GIT_RESULT)
  string(LENGTH "${CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT}"
    CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT_LENGTH)
  if(NOT CSL_HISTORY_MEMBERSHIP_GIT_RESULT EQUAL 0 OR
     NOT CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT_LENGTH EQUAL 40 OR
     NOT CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT MATCHES "^[0-9a-f]+$")
    message(FATAL_ERROR "The history membership example requires a Git source commit")
  endif()
  if(NOT casadi_VERSION)
    message(FATAL_ERROR "The history membership example requires a versioned CasADi package")
  endif()
  set(CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT "${CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT}"
    CACHE INTERNAL "Source commit compiled into the history membership example" FORCE)
  set(CSL_HISTORY_MEMBERSHIP_COMPILER "${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
    CACHE INTERNAL "Compiler identity for the history membership example" FORCE)
  set(CSL_HISTORY_MEMBERSHIP_CASADI_VERSION "${casadi_VERSION}"
    CACHE INTERNAL "CasADi version for the history membership example" FORCE)
  add_executable(history_membership_2d_example tools/history_membership_2d_example.cpp)
  target_link_libraries(history_membership_2d_example PRIVATE CSL::casadi_history_optimizer)
  target_compile_definitions(history_membership_2d_example PRIVATE
    CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT="${CSL_HISTORY_MEMBERSHIP_SOURCE_COMMIT}"
    CSL_HISTORY_MEMBERSHIP_COMPILER="${CSL_HISTORY_MEMBERSHIP_COMPILER}"
    CSL_HISTORY_MEMBERSHIP_CASADI_VERSION="${CSL_HISTORY_MEMBERSHIP_CASADI_VERSION}")
  set_target_properties(history_membership_2d_example PROPERTIES CXX_EXTENSIONS OFF)
  csl_strict_warnings(history_membership_2d_example)
endif()
include(cmake/Package.cmake)
if(BUILD_TESTING)
  include(cmake/Dependencies.cmake)
  include(cmake/TestCatalog.cmake)
  foreach(module IN LISTS csl_modules)
    include(src/${module}/Tests.cmake)
  endforeach()
  include(cmake/InstalledTests.cmake)
endif()
