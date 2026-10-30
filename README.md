# Memsonic

An embeddable simulation for MEMS accelerometer, gyroscope, barometer, and magnetometer.
Tailored for high-power rocketry.

## Rationale

In HPR, navigation and guidance algorithms are often developed and evaluated with multiphysics
simulation software, HITL, wind tunnel experiments, and flight tests. However, tight coupling
of GNC with the rest of software stack complicates failure isolation and diagnosis. In-flight
debugging and physical testing are costly and hard to recreate, while unable to exhaustively
cover the fault space. These limitations make long-term regressions difficult to reproduce and
localize.

Memsonic complements existing verification techniques. It provides deterministic approximations
of calibrated sensor outputs and controlled fault injection. This enables identical replay of
user-defined scenarios and cheap comparisons across debuggable software versions.

## Operation

Simulated output is determined by:

**1. Kinematic targets**: ordered, timestamped pairs of *modeled* acceleration and angular
velocity vectors. This is a grand paper plan of the flight; how strongly will simulated output
diverge will depend on parameters below.

**2. ODR**: per-sensor output rate after oversampling and decimation. To avoid aliasing,
the *model* will adjust to run faster than the sensor with highest ODR.

**3. Builder template**: sets sensor bias, alignment, sensitivity, and filter properties.
A specific template will focus on one or a combination of them; use `Manual` template for hand tuning.

**4. Environment preset**: defines averaged ground pressure, temperature, and magnetic field at the
launch site.

Memsonic is not thread safe (worker thread recommended), allocated on stack by default
but can be allocated on heap.

## Usage

Choose [launch coditions](./src/env.rs) and [builder preset](./src/builder.rs)
(for example, `Furnas` and `SkewedIMU`), and construct an approximate *ideal* plot.
The simulation will handle noise, drift, and derived measurements.

```rust
use memsonic::{
    Simulation,
    builder::{SimBuilder, SkewedIMU},
    env::Furnas,
};

fn main() {
    // ODR for accel, gyro, baro, mag, and a "noise factor".
    let bd = SkewedIMU::new((5.0, 5.0, 20.0, 25.0), 0.5);
    // 64 is the size of stack-allocated plot. 1000 is idle time.
    let mut simu = Simulation::<64>::new::<Furnas>(bd, 1000);

    // Time elapsed, acceleration (XYZ, m/s^2), angular velocity (XYZ, deg/s).
    simu.fix(200, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
        .add(500, [0.0, 0.0, 20.0], [0.0, 0.0, 0.0]);
        // ... plot

    let ts = your_favorite_timer_function();

    if let Ok(a) = simu.accel(ts) {
        // ...
    }
}
```

### Cross-compilation

Compiles for `thumbv8m.main-none-eabihf` target by default. Hard floating-point support is strongly
recommended for performance reasons. To compile for desktop, specify the `host-tuple` target.

C API comes out of the box and is available [here](./c-api/memsonic.h).
