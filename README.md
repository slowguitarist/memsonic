# memsonic

An embeddable simulation engine for MEMS accelerometer, gyroscope, barometer, and magnetometer.
Tailored for high-power rocketry.

## Motivation

In HPR, navigation and guidance algorithms are often developed and evaluated with multiphysics
simulation software, HITL, wind tunnel experiments, and flight tests. However, tight coupling
of GNC with the rest of software stack complicates failure isolation and diagnosis. In-flight
debugging and physical testing are costly and hard to recreate, while unable to exhaustively
cover the fault space. These limitations make long-term regressions difficult to reproduce and
localize.

Memsonic complements existing verification techniques. It provides deterministic approximations
of calibrated sensor outputs and controlled fault injection. This enables identical replay of
user-defined scenarios and cheap comparisons across debuggable software versions.

## Principle of operation

Simulated sensor output is determined by:

**1. Kinematic targets**: ordered, timestamped pairs of *modeled* acceleration and angular
velocity vectors. This is a grand paper plan of the flight; how far will simulated output end
up from this plan depends on other parameters.

**2. ODR**: per-sensor output rate after oversampling and decimation. To avoid aliasing,
the *model* will adjust to run faster than the sensor with highest ODR.

**3. Builder template**: sets sensor bias, alignment, sensitivity, and filter properties.
A specific template will focus on one or a combination of them; use `Manual` template for hand tuning.

**4. Environment preset**: defines ground pressure, temperature, and magnetic field at the
launch site. Weather fluctuations are yet to come.

The simulation does not need an allocator or a dedicated thread, although if desired, can be
allocated on the heap and polled in its own thread. On polling, the *model* is lazily propagated
and sensor outputs evaluated, returning `Ok` if current reading has not yet been returned and
`Err` otherwise. Interrupts using this feature currently have to be configured in the user's
application.


## Usage

Soon!

### Cross-compilation

### Example Scenarios
