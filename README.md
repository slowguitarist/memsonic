# memsonic

An embeddable simulation engine for MEMS accelerometer, gyroscope, barometer, and magnetometer. Tailored for high-power rocketry.

## Motivation

In HPR, navigation and guidance algorithms are usually developed and evaluated with multiphysics simulation software, HITL, wind tunnel experiments, and flight tests. However, tight coupling of GNC with the rest of flight software complicates failure isolation and reproduction.

In-flight debugging cannot recreate exact conditions once they have passed, and physical testing is costly and does not cover the entire fault space. These limitations make long-term regressions difficult to reproduce and localize.

Memsonic complements the existing verification techniques. It provides deterministic approximations of calibrated sensor outputs and controlled fault injection, enabling identical replay of user-defined scenarios.

## Principle of operation

Soon!

## Usage

### Cross-compilation

### Example Scenarios
