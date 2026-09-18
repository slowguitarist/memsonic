# memsonic

An embeddable simulation engine for MEMS accelerometer, gyroscope, barometer, and magnetometer. Tailored for high-power rocketry.

## Motivation

In HPR, stateful navigation algorithms are usually designed and tested using dedicated multiphysics simulation software, while integration testing is done in wind tunnels and during test launches. Because GNC is often indivisible from the rest of software system, validating the integrity of state estimation becomes a double-edged sword:

1. Live debugging cannot accommodate in-flight conditions;
2. Physical simulation is expensive, while useful only ex post facto.

These constraints make it difficult to trace down design and logic errors that incrementally degrade the quality of state estimation under flight conditions.

Memsonic addresses the first constraint. It provides accurate deterministic approximations of calibrated sensor outputs, and supports fault injection for testing corner cases.

## Principle of operation

Soon!

## Usage

### Cross-compilation

### Example Scenarios
