# Radar System Simulator

## Overview

Radar System Simulator is a modular C++ project that simulates aircraft motion, radar tracking, RF propagation, radar performance calculations, noisy sensor measurements, and Kalman-filter-based position estimation.

The project was developed to explore the integration of C++ software engineering, radar/RF mathematics, three-dimensional kinematics, sensor simulation, and state estimation within a single system.

The simulator generates an aircraft with position, velocity, and acceleration values and tracks the aircraft as it moves through three-dimensional space. Radar and RF calculations are updated using the changing aircraft state. Simulated measurement noise is then applied to the aircraft's true position, and a Kalman filter estimates the aircraft position from the noisy measurements.

The system compares the raw radar measurement error with the Kalman-filtered position error to evaluate tracking accuracy.

---
## Development Experience

This radar tracking simulation is an independently developed project and was not created as part of a school assignment or course requirement. The project grew from my curiosity about electromagnetics, radar systems, and how electromagnetic waves can be used to detect and track moving objects. I developed the project as a way to expand my understanding of C++, radar and RF principles, mathematical modeling, linear algebra, and state estimation by translating those concepts into a working software simulation.

As the project evolved, I researched individual concepts, implemented the mathematical relationships in C++, tested calculations across repeated samples, and gradually integrated the individual components into a larger multi-module system. This process allowed me to explore not only how the individual equations work, but also how kinematics, electromagnetic wave behavior, RF calculations, radar performance, measurement uncertainty, and Kalman filtering are interconnected within a tracking system.

## From Kinematic Equations to Linear Algebra

Early in the project, I implemented aircraft motion using separate kinematic equations for position, velocity, and acceleration. This allowed me to first understand and verify each calculation independently.

As the project developed, I applied concepts from linear algebra to represent the relationships between these variables using state vectors and a state-transition matrix. Instead of treating position, velocity, and acceleration as completely independent calculations, the matrix representation showed how each variable contributes to the aircraft's changing state over time.

For example, the relationship between position, velocity, and acceleration can be represented as:
For one dimension, the aircraft state can be represented as:

            [ x ]
State =     [ v ]
            [ a ]

The state-transition relationship is:

[ x_new ]   [ 1   Δt   0.5Δt² ] [ x ]
[ v_new ] = [ 0   1    Δt      ] [ v ]
[ a_new ]   [ 0   0    1       ] [ a ]

Where:
x  = position
v  = velocity
a  = acceleration
Δt = change in time

I expanded this concept into three dimensions using position (x,y,z), velocity (vx,vy,vz), and acceleration (ax,ay,az). This resulted in a state representation that could describe the aircraft's complete simulated motion and provided an important foundation for the Kalman filter.

## Using Repeated Samples to Verify Calculations

Another important part of the development process was learning to evaluate calculations across multiple samples instead of relying on a single output. I used std::size_t as the counter in for loops to repeatedly process simulation data:

for (std::size_t i = 0; i < NumberOfSamples; ++i)
{
    // Update and evaluate simulation data
}

Running multiple samples allowed me to observe how calculated values changed from one simulation step to the next. I could examine aircraft position, velocity, range, received radar power, simulated radar measurements, and estimated positions over time.

This became an important method of verifying my work. Rather than assuming that a formula was correct because it produced one reasonable result, I could observe its behavior across repeated iterations and use the output to identify inconsistencies, debug problems, and determine whether the calculations were updating as expected.

## Understanding How the Mathematics Is Interrelated

One of the most valuable lessons from the project was understanding that the mathematical components are not isolated formulas. The output of one part of the simulation frequently becomes the input to another.

Aircraft position, velocity, and acceleration determine how the target moves through three-dimensional space. The resulting position is used to determine range from the radar, while velocity contributes to Doppler calculations. Range, radar frequency, wavelength, antenna characteristics, and other parameters are then used in RF calculations such as received power, free-space path loss, power density, and Doppler frequency shift.

These results connect with additional radar-performance calculations, including range resolution, maximum unambiguous range and velocity, signal-to-noise ratio, beamwidth, clutter calculations, pulse compression, coherent integration, probability of false alarm, and radar horizon.

Working through these relationships helped me understand the project as a system of interconnected mathematical models rather than a collection of unrelated equations.

## Developing the Kalman Filter

Building the Kalman filter was one of the most challenging and rewarding portions of the project. It required me to learn new techniques involving Eigen matrices and vectors, state representation, state prediction, measurement updates, simulated sensor noise, and estimation-error analysis.

The simulation generates noisy radar position measurements and passes those measurements into the Kalman filter. This allows the program to compare three different values:

## True aircraft position → noisy radar measurement → Kalman estimated position

I then calculate the position error of both the raw radar measurement and the Kalman estimate. Running these calculations repeatedly allowed me to evaluate whether the filter was actually improving the estimate rather than simply assuming that it was working.

For example, during one simulation run, the average raw radar position error was approximately 4.78 meters, while the average Kalman position error was approximately 2.48 meters. This provided a measurable way to evaluate the behavior of the estimator.

## Software Integration

As the project grew, another major challenge was moving from individual working components to an integrated software system. I organized the program into separate modules responsible for aircraft generation, three-dimensional tracking, RF calculations, radar resolution and quality calculations, Kalman filtering, and overall simulation control.

This required me to think carefully about which values needed to be shared between modules. Instead of repeatedly entering the same values independently, calculated results such as aircraft range, velocity, wavelength, received power, and other parameters could be passed into the next component that required them.

This part of the project helped me better understand the difference between writing an individual program that performs a calculation and designing multiple software components that must communicate and operate together.

## What I Learned

Through this project, I gained practical experience with:

Object-oriented C++ programming and classes
Header and implementation files
Multi-file project organization
Linux/WSL compilation and debugging
for loops, std::size_t, and zero-based iteration
Repeated sampling for calculation verification
Three-dimensional kinematic modeling
Linear algebra and state-transition matrices
Eigen vectors and matrices
State-space representation
Kalman filtering and state estimation
Random-number generation and simulated measurement noise
Radar and RF mathematical calculations
Tracking and estimation error analysis
Sharing calculated values between software modules
Debugging through intermediate output and repeated testing
Incremental integration of independently developed components

## Conclusion

Overall, this project taught me much more than how to translate individual mathematical formulas into C++. I progressed from writing separate kinematic equations and testing individual calculations to representing a dynamic system with linear algebra, processing repeated simulation samples, introducing measurement uncertainty, estimating aircraft state with a Kalman filter, and integrating multiple C++ modules into a working radar tracking simulation.

The development process reinforced the importance of understanding the mathematics behind the code, testing calculations across multiple samples, examining intermediate results, debugging systematically, and understanding how individual components contribute to the behavior of the complete system

---

## Current Features

- Random aircraft state generation
- Three-dimensional position, velocity, and acceleration
- Constant-acceleration aircraft motion model
- Multi-step aircraft tracking simulation
- Eigen-based state representation
- Radar range and velocity calculations
- RF wavelength and wavenumber calculations
- Radar received-power calculations
- Free-space path loss calculations
- Power density calculations
- Doppler frequency shift calculations
- Radar range resolution
- Maximum unambiguous range
- Maximum unambiguous velocity
- Signal-to-noise ratio calculations
- Signal-to-clutter ratio calculations
- Probability of false alarm calculations
- Pulse compression time-bandwidth product
- Coherent integration gain
- Radar horizon calculation
- Simulated Gaussian radar measurement noise
- Kalman filter state estimation
- Radar measurement vs. Kalman estimate error comparison
- Multi-module C++ project architecture

---

## Project Architecture

The project is divided into independent modules:

RadarSystemSimulator/
├── AircraftSimulatorRandomGenerator/
│   ├── AircraftNumberGenerator.cpp
│   └── AircraftNumberGenerator.h
│
├── EigenTracking/
│   ├── ThreeDimensionalTrackingData.cpp
│   └── ThreeDimensionalTrackingData.h
│
├── KalmanFilter/
│   ├── KalmanFilterEstimator.cpp
│   └── KalmanFilterEstimator.h
│
├── RFCalculations/
│   ├── RFCalculations.cpp
│   └── RFCalculations.h
│
├── RadarResolutionAndQuality/
│   ├── RadarResolutionAndQuality.cpp
│   └── RadarResolutionAndQuality.h
│
├── RadarSimulation/
│   ├── RadarSimulation.cpp
│   └── RadarSimulation.h
│
├── RadarTrackingSystem.cpp
├── RadarTrackingSystem.h
├── main.cpp
└── README.md

---

## Module Responsibilities

### AircraftSimulatorRandomGenerator

Generates simulated aircraft initial conditions including:

- X, Y, and Z position
- X, Y, and Z velocity
- X, Y, and Z acceleration

These values provide the initial aircraft state used by the tracking simulation.

### EigenTracking

Maintains and updates the aircraft's three-dimensional state using position, velocity, acceleration, and elapsed simulation time.

The motion model uses constant-acceleration kinematic relationships to propagate the aircraft state forward through time.

### RadarTrackingSystem

Processes aircraft motion data to calculate tracking information such as aircraft position, range, total velocity, distance traveled, and angle-related information used by other radar calculations.

### RFCalculations

Performs RF and radar propagation calculations including:

- Wavelength
- Wavenumber
- Received radar power
- Free-space path loss
- Power density
- Antenna gain
- Doppler frequency shift

### RadarResolutionAndQuality

Evaluates radar performance characteristics including:

- Maximum unambiguous range
- Maximum unambiguous velocity
- Range resolution
- Signal-to-noise ratio
- Beamwidth
- Signal-to-clutter ratio
- Probability of false alarm
- Pulse compression ratio
- Coherent integration gain
- Radar horizon distance

### KalmanFilter

Processes noisy simulated radar position measurements and produces estimated X, Y, and Z aircraft positions.

The simulator compares the resulting Kalman position estimate against both the true aircraft position and the raw radar measurement.

### RadarSimulation

Acts as the primary simulation coordinator.

It initializes the aircraft, tracking system, RF calculations, radar quality calculations, Eigen state model, and Kalman filter. It then runs the multi-step simulation and coordinates the exchange of data between the individual modules.

---

## Simulation Flow

The current simulation follows this general data flow:

Aircraft Generator
        |
        v
True Aircraft State
        |
        v
3D Motion Model
        |
        +--------------------+
        |                    |
        v                    v
Radar Tracking          RF Calculations
        |                    |
        |                    v
        |              Radar Performance
        |
        v
Simulated Radar Measurement
        |
        +---- Gaussian Measurement Noise
        |
        v
Kalman Filter
        |
        v
Estimated Aircraft Position
        |
        v
Tracking Error Analysis

---

## Example Tracking Results

A ten-step simulation produced the following average position errors:

Raw Radar Measurement Error:
4.78341 meters

Kalman Estimated Position Error:
2.48295 meters

In this simulation run, the Kalman filter reduced the average three-dimensional position error compared with the noisy radar measurements.

---

## Technologies

- C++
- Eigen
- Object-Oriented Programming
- Linux / WSL
- GNU g++
- Git / GitHub

---

## Build

Eigen must be installed before compiling the project.

Example Ubuntu/WSL installation:

sudo apt install libeigen3-dev

Compile:

g++ main.cpp \
RadarTrackingSystem.cpp \
RFCalculations/RFCalculations.cpp \
EigenTracking/ThreeDimensionalTrackingData.cpp \
KalmanFilter/KalmanFilterEstimator.cpp \
RadarResolutionAndQuality/RadarResolutionAndQuality.cpp \
AircraftSimulatorRandomGenerator/AircraftNumberGenerator.cpp \
RadarSimulation/RadarSimulation.cpp \
-I /usr/include/eigen3 \
-o radar_system

Run:

./radar_system

---

## Project Status

The current version integrates aircraft generation, three-dimensional motion, radar tracking, RF calculations, radar performance calculations, simulated sensor noise, and Kalman filtering into a single executable simulation.

Future development may include additional tracking logic, multiple-aircraft simulation, improved sensor modeling, data visualization, and further refinement of the Kalman filter and radar models.


---

## Purpose

This project is an independent software engineering and radar-systems learning project. It is intended to demonstrate the application of C++ programming, modular software design, mathematical modeling, radar/RF concepts, and state-estimation techniques.