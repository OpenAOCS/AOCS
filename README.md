# AOCS

This repository is intended to provide the basis of development for primitive AOCS capabilities, with an associated simulation framework.

## Git LFS

The large data binaries are stored with Git LFS, as they are relatively static and not expected to change as the repository evolves. Install and initialize Git
LFS before cloning:

```sh
sudo apt-get install git-lfs
git lfs install
git clone git@github.com:OpenAOCS/AOCS.git
```

For an existing clone, run `git lfs install` and then `git lfs pull` from the
repository root to download the large data files.

## Build

Requirements: CMake 3.20 or newer, a C++17 compiler, the JsonCpp and pybind11 development packages, and Python development headers. Eigen is included in the repository.

```sh
sudo apt-get update
sudo apt-get install cmake g++ libjsoncpp-dev pybind11-dev python3-dev ninja-build
```
For development, additionally clang tooling should be installed
```sh
sudo apt-get install clang-format clang-tidy clang-tools
```

With a new clone, configure and build with:  

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

## Before committing

The checked-in [`.clang-format`](./.clang-format) is the style policy. To perform a clang review, rebuild with the compile commands flag on:  

```sh
cmake -S . -B build -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build --parallel 4 --target gnc_lib sim sim_math_tests aocs_orbit
```

For each change, format edited files, then run the repository checks and tests:

```sh
clang-format -i path/to/changed.cpp path/to/changed.hpp
python3 scripts/check_clang_format.py
python3 scripts/check_clang_tidy.py --build-dir build --jobs 4
ctest --test-dir build --output-on-failure
```

The clang-format script skips vendored Eigen under `shared/external/`. The
clang-tidy script runs each profile across its matching translation units in
the compile database: [`gnc_lib/.clang-tidy`](./gnc_lib/.clang-tidy) for GNC and
[`sim/.clang-tidy`](./sim/.clang-tidy) for the simulation and its tools. Findings
are warnings, not build failures.  
  
GitHub Actions runs the same checks on pushes
and pull requests. Use the same clang-format version locally and in CI.

The optional CMake setting `-DAOCS_ENABLE_CLANG_TIDY=ON` also runs clang-tidy
during compilation; the script above checks all translation units without
requiring a clean rebuild.

Run the simulation from the repository root:

```sh
./build/sim/sim
```

## Analysis

Requirements: Python 3.14 or newer

Create a virtual environment
```sh
# if necessary, install venv
apt install python3.14-venv
python3 -m venv .venv
source ./venv/bin/activate
```
Install requirements
```sh
pip install -r requirements.txt
```

After running the simulation, generate the interactive report from the
repository root:

```sh
python plotting/plot_results.py
```

The report contains STATE & ORBIT, INERTIAL NAVIGATION, and COVARIANCE tabs.
The first tab contains the 3D ECI orbit, position and velocity time series in
three columns, and the six osculating-element plots. The navigation tab
compares raw GPS readings with the filtered estimate in ECEF, highlights
invalid GPS intervals in red, and plots ECI estimate-versus-truth errors. The
covariance tab plots position and velocity variances. Regenerate the simulation
TsFile before plotting so it includes the GPS and ECEF estimate telemetry.
The plotter calls the `aocs_orbit` pybind11 module, which invokes the C++
`IJKKeplerian::convert_ijk_to_keplerian` function for each state vector.
Angular elements are converted from radians to degrees.


## Roadmap
<pre>
Week 1: Inertial Nav & EKF MVP
├── High-Fidelity GPS
│   ├── <del>GPS Sensor Model</del>
│   └── <del>Sensor Invalid Feature (General)<del>
└── EKF MVP Architecture
    ├── <del>Process Noise</del>
    └── Increased STM fidelity - exponential drag, J2, Vallado Sun/Moon Ephem

Week 2: Hi-Fi Inertial Physics
├── Drag Model (MSIS) - Pull in src FORTRAN
├── Solar Pressure Model
└── DE440 Ephemeris

Week 3: Body Dynamics
├── Frame Library
│   ├── Sensor Frame
│   └── Body Frame
└── Bus Mass Simulation
    ├── CG Calculation - Wet/Dry Mass
    └── Inertia Dynamics - Tanks

Week 4: Attitude System
├── Attitude Dynamics
│   ├── Quaternion Attitude Representation
│   └── Attitude Propagation
└── Attitude Sensing
    ├── Sun Sensor
    ├── Magnetometer / IGRF or WMM
    ├── IMU
    └── Star Tracker

Week 5: Attitude Control I
├── MEKF - Markley
└── Attitude Control
    ├── Torque Rod
    └── B-Dot Control

Week 6: Hi-Fi Attitude Sim
├── Plate Model (SRP/Drag)
└── Gravity Gradient

Week 7: ADCS
├── Attitude Actuators
│   ├── Reaction Wheels
│   └── Thrusters
└── Attitude Control II
    ├── LQR Body Control
    ├── PD Momentum Control
    ├── L2 RW Allocation
    └── PID Torque Control

Week 8: Station Keeping Guidance
├── Hohmann Transfer
├── Bi-Elliptic Transfer
└── Inclination Change

Week 9: Hi-Fi Orbital Maneuvering
├── Thruster Allocation - Facet Method (?)
└── Mass Flow Dynamics      
</pre>
