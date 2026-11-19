# CSL C++ SDK

Compute reachable sets and try constrained feedback controllers on discrete-time models.
The first example uses a two-state plant to show when the next observation changes the control.

Use this repository for the C++ library, its CMake package and runnable examples. Download versioned source from [SDK releases](https://github.com/butahlecoq/csl-sdk/releases). The research project hosts the [illustrated learning book](https://butahlecoq.github.io/controlled-systems-lab/learn/), [numerical reports](https://butahlecoq.github.io/controlled-systems-lab/archive.html) and [historical releases](https://github.com/butahlecoq/controlled-systems-lab/releases/tag/v0.1.0).

## Quick start

Build an observed-state controller, run it beside an open-loop plan, and inspect the first step at which their controls differ. You need Git, CMake/CTest 3.31 or newer and a C++20 compiler. On Windows, use a Visual Studio developer shell with CMake installed. The first configure downloads the pinned HiGHS and Qhull dependencies. CI exercises this default profile on Ubuntu.

Run in PowerShell or a POSIX shell. The installation prefix stays inside your checkout. Installation is a local development check, not a binary distribution license grant.

```sh
git clone https://github.com/butahlecoq/csl-sdk.git
cd csl-sdk
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

## Discover and check a reachability example

If you are exploring the SDK yourself or delegating to a coding agent, start
with the installed example build from the quick start. CTest can describe its
registered tests as JSON. The native catalogue lists the stable IDs of its
reachability cases.

```sh
cmake --build build/examples --config Release --target native_model_catalogue --parallel 4
ctest --test-dir build/examples -C Release --show-only=json-v1
ctest --test-dir build/examples -C Release -R '^native_catalogue_source_cases$' --no-tests=error --output-on-failure
```

With a single-configuration build, run the catalogue from the checkout root.
With Visual Studio, the executable is `build/examples/Release/native_model_catalogue.exe`.
Give each export a new directory.

```sh
build/examples/native_model_catalogue --list
build/examples/native_model_catalogue build/slide33
```

The export covers the seven- and eight-transition variants of a three-state
model with one control. Read `build/slide33/full-state-v-points.csv`. For case
`bulaev-2016-slide33/illustrated-seven/1`, select stage 7 and take the minimum
of `x1`. It should be **−63.5**. You can check that number directly. The first
coordinate starts as low as −0.5 and changes by `3u` per step. Applying the
lowest allowed control, −3, for seven steps gives −0.5 + 7 × 3 × (−3) = −63.5.
Open `build/slide33/native-model-catalogue-screen.svg` to see the projections
from the same run. When sharing a result, include the SDK commit, case ID and
stage. [Illustrated example](https://butahlecoq.github.io/controlled-systems-lab/examples/native-model-catalogue.html)

For module substitutions, the research project's
[composition inventory](https://github.com/butahlecoq/controlled-systems-lab/blob/main/docs/catalogue/module-composition-inventory-v1.json)
records module IDs, build-dependent classifications and limitations at its
audited revision.

## Use CSL in a CMake project

```cmake
find_package(CSL 0.1.1 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE CSL::controllers CSL::model_catalogue CSL::highs_convex)
```

The [example CMake project](examples/CMakeLists.txt) uses the installed targets.
The [native source modules](src), [public headers](include/csl), [schemas](schemas),
and [tests](tests) accompany the SDK. The [research repository](https://github.com/butahlecoq/controlled-systems-lab) owns experiment drivers, comparison datasets, teaching pages and historical evidence.

The original SDK extraction and source baseline are recorded in [research issue #1082](https://github.com/butahlecoq/controlled-systems-lab/issues/1082).
