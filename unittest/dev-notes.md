# Unit Testing

## General Concepts

We use `gtest` from Google to unit test our code. The test framework is discussed further in [their documentation](http://google.github.io/googletest/). 

This testing infrastructure is built into the overall build process. There is only one cmake system for all of `AAESim` and all test code. 

Manually run all tests using from the root of the `AAESim` project space. Follow these steps:

```bash
cd build
cmake ..
make run_tests
```

The above command is specified in [unittest.cmake](./unittest.cmake). As explained below, it actually runs several test binaries in series, each one producing JUNIT XML results. See the cmake setup for details.

## Development Principles

* Test source code is organized according to the library that is to be tested. All developers are responsible for keeping the test source code organized.
* Test source code files should be named in a very clear manner for what is being tested in the code.
   * Ex: `bada_factory_tests.cpp`
   * A good pattern to follow is `<lower_case_class_name>_tests.cpp`, but variations are acceptable so long as the intent is clear.
* Unit test methods must be small (as much as possible). A test should verify a specific functionality of the code and assert on a clear pass/fail result (or several). 
* All of these tests are intended to be _small_ tests that should run rapidly and give rapid feedback on code validity. Do not write regression tests here (legacy exception: `/src/Simulation/aircraft_simulation_rapid_regression.cpp`)
* Each library produced by `AAESim` has a corresponding test library. 
   * Ex: library `libpub` has a `public_test` binary that tests just that library. You can run those tests by using `make run_public_test`.
* Each test library links only the `AAESim` library it is intended to test (and also `gtest`).
   * Ex: test binary `bada_test` links against library `libbada`. See [this example](./src/Bada/bada.cmake).

## Test Targets

Each library in `AAESim` has a corresponding test binary that can be run by itself. Each test binary only links the library that it needs to test. For example, test binary `bada_test` only links `libbada`. This structure allows isolated testing to occur for each library. 

Below is the structure of the test build system.

| AAESim Library | Test Binary | Make Target for Test Binary | Make Target to Run Test Binary |
| --- | --- | --- | --- |
| `Public/lib/libpub` | `unittest/bin/public_test` | `public_test` | `run_public_test` |
| `Bada/lib/libbada` | `unittest/bin/bada_test` | `bada_test` | `run_bada_test` |
| `Avionics/lib/libavionics` | uses `simulation_test` | --- | --- |
| `IntervalManagement/lib/libimalgs` | `unittest/bin/imalgs_test` | `imalgs_test` | `run_imalgs_test` |
| `AAESim/lib/libcore` | `unittest/bin/simulation_test` | `simulation_test` | `run_simulation_test` |
| `AircraftDynamicsTestFramework/lib/libframework` | `unittest/bin/fmacm_test` | `fmacm_test` | `run_fmacm_test` |

To get JUNIT XML output from any one test binary, use the `run_*` make target. To run all test binaries and get JUNIT output from each, use the make target `run_tests`. 