# CSL C++ SDK

Compute reachable sets and try constrained feedback controllers on discrete-time models.
The first example uses a two-state plant to show when the next observation changes the control.

This repository is the SDK extraction candidate for [controlled-systems-lab #1082](https://github.com/butahlecoq/controlled-systems-lab/issues/1082). The research repository hosts the [learning book](https://butahlecoq.github.io/controlled-systems-lab/learn/), numerical reports, and the established release series. The SDK source baseline is [cde5412d](https://github.com/butahlecoq/controlled-systems-lab/tree/cde5412d5ed9e352d8e0fdb9bb54433a3bd94305).

## Quick start

Requires Git, CMake/CTest 3.31 or newer and a C++20 compiler. This extraction runs the default HiGHS and Qhull profile on Ubuntu in CI. Put CMake on PATH; on Windows a Visual Studio developer shell with its CMake component installed provides the tools. Run from the repository root and stop on any nonzero exit. The first configure downloads the existing pinned dependencies; no Python is needed with `BUILD_TESTING=OFF`. Installation is a local development check, not a binary distribution license grant.

These commands work in PowerShell and a POSIX shell; `$PWD` supplies an absolute install prefix. `--config Release` selects Release for multi-configuration generators, and `CMAKE_BUILD_TYPE` selects it for single-configuration generators.

```sh
cmake -S . -B build/quickstart -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/quickstart --config Release --parallel 4
cmake --install build/quickstart --config Release --prefix build/quickstart/install
cmake -S examples -B build/examples -DCMAKE_BUILD_TYPE=Release "-DCMAKE_PREFIX_PATH=$PWD/build/quickstart/install"
cmake --build build/examples --config Release --target discrete_mpc_2d --parallel 4
ctest --test-dir build/examples -C Release -R '^discrete_mpc_2d_matched_open_loop_and_feedback$' --no-tests=error -V
```

Expect one passing test. In the published example, the first control coordinate
differs for the first time at stage 4. It is 0 for the open-loop plan and about
1.05 for observed-state MPC. The state and control histories are in
`build/examples/discrete-mpc-2d-test-output/episode.csv`. This is a numerical
check on the example's eight-stage task.

To try another forecast or disturbance, edit `make_task` in the
[example source](examples/discrete_mpc_2d.cpp). To supply your own
model and bounds, use the public
[`NominalMpcTask` fields](include/csl/controllers/discrete_mpc.hpp). The source
data section shows the model and bounds used in this
[example page](https://butahlecoq.github.io/controlled-systems-lab/examples/discrete-mpc-2d.html).

### More examples

Build and run the full installed example suite when you want to explore more
operations.

```sh
cmake --build build/examples --config Release --parallel 4
ctest --test-dir build/examples -C Release --output-on-failure --no-tests=error
```

## Use CSL in a CMake project

```cmake
find_package(CSL 0.1.0 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE CSL::controllers CSL::model_catalogue CSL::highs_convex)
```

The [example CMake project](examples/CMakeLists.txt) uses the installed targets.
The [native source modules](src), [public headers](include/csl), [schemas](schemas),
and [tests](tests) accompany the SDK. The [research repository](https://github.com/butahlecoq/controlled-systems-lab) owns experiment drivers, comparison datasets, teaching pages and historical evidence.
