# FMACM

> This is the copyright work of The MITRE Corporation, and was produced
for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
is subject to Federal Aviation Administration Acquisition Management
System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
(Oct. 1996).  No other use other than that granted to the U. S.
Government, or to those acting on behalf of the U. S. Government,
under that Clause is authorized without the express written
permission of The MITRE Corporation. For further information, please
contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
McLean, VA  22102-7539, (703) 983-6000.
>
> Copyright 2020 The MITRE Corporation. All Rights Reserved.
> Approved for Public Release; Distribution Unlimited. 15-1482

This project contains content developed by The MITRE Corporation. If this code is used in a deployment or embedded within another project, it is requested that you send an email to [opensource@mitre.org](mailto:opensource@mitre.org) in order to let us know where this software is being used.

![Apache 2.0](https://img.shields.io/badge/license-Apache%20License%202.0-blue?style=for-the-badge)

![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

![Rocky Linux](https://img.shields.io/badge/-Rocky%20Linux-%2310B981?style=for-the-badge&logo=rockylinux&logoColor=white)
![macOS](https://img.shields.io/badge/mac%20os-000000?style=for-the-badge&logo=macos&logoColor=F0F0F0)

## Licensing

[Apache 2.0](https://github.com/mitre/FMACM/blob/master/LICENSE)

## Documentation

1. [Technical documentation](https://www.mitre.org/publications/technical-papers/derivation-of-a-point-mass-aircraft-model-used-for-fast-time) that shows the models and their derivations is provided by MITRE.
2. Official [RTCA](https://www.rtca.org) standards documentation is available in DO-328B and DO-361A. Please contact [RTCA](https://www.rtca.org) for more information.

## Licensing questions

Any questions related to MITRE Open Source technologies may be emailed to [opensource@mitre.org](mailto:opensource@mitre.org).

## Developer Notes

### Aircraft Performance

This code needs aircraft performance data in order to propogate the dynamics models. [EUROCONTROL's BADA v3](https://eurocontrol.int/services/bada) as an example of one such data set and is what MITRE uses internally with this simulation. Our BADA functionality and code cannot be publicly released due to licensing restrictions. You should provide your own implementation by extending [include/public/FixedMassAircraftPerformance.h](../include/public/FixedMassAircraftPerformance.h). We recommend doing this in a fully separate library and then linking that into this code base.

### Continuous Integration & Testing

This code base is heavily tested internally. Publicly, we test using GitHub Actions. See the [ci.yml](../.github/workflows/ci.yml) for details.

### Compile

No attempt has been made to ensure that this code will compile and run properly on all operating systems. Review the [ci.yml](../.github/workflows/ci.yml) to explore what configurations we choose to test. We build like this:

```bash
cmake -G Ninja -S . -B build
cmake --build build --target FMACM
```

The resulting executable will be `/bin/FMACM`.

### Run Unit Tests

Unit tests can be run via the CMake infrastructure.
Assuming the software already compiles:

```bash
cmake --build build --target run_public_test run_fmacm_test
```

### Run

Run from the terminal. Or compile the libraries here into a larger code base for richer access.

The [main entry point](../AircraftDynamicsTestFramework/fmacm.cpp) provides:

- `--version`: report the build version;
- `--buildinfo`: report the build environment;
- a single positional argument is used to provide a configuration file.

The above command line arguments may not be combined.
Use them one at a time.

A configuration file must be provided to run a simulation.
The file must be formatted as plain text and contain paths to each scenario that is to be run.
An example is provided: [configuration example](../Run_Files/test-framework-configuration.txt).

The executable is then executed:

```bash
./bin/FMACM test-framework-configuration.txt 
```

Output is found in the run-time directory in the form of CSV files.
