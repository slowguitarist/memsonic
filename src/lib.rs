//! # memsonic
//!
//! MEMS sensor simulation engine for high-power rocketry.

#![no_std]

use crate::{builder::SimBuilder, model::Model, profile::CannedProfile, sensors::Consume};

mod common;
mod filters;
mod math;
mod model;
mod profile;
mod sensors;

pub mod builder;

/////////////////////////////////////////////////////////////////////////////
// Public helper types
/////////////////////////////////////////////////////////////////////////////

/// A real vector in 3D.
pub type XYZ = [f32; 3];

/// Combined output data rates for accelerometer, gyroscope,
/// magnetometer, and barometer (in this order).
pub type ODR = (u32, u32, u32, u32);

/// Bias value for each of N measurement axes.
pub type Bias<const N: usize> = [f32; N];

/// An N by N matrix for each axis' alignment.
pub type Alignment<const N: usize> = [Bias<N>; N];

/// A theoretical kinematic target (acceleration, angular velocity).
#[derive(Clone, Copy)]
pub struct Motion(pub XYZ, pub XYZ);

/// DRY getter.
macro_rules! getter (($n:literal, $f:ident, $d:ident, $t:ty) => (
	#[doc = concat!("
	Returns a result that always contains the most recent ", $n, "
	 reading from the simulation. `Ok` indicates that the reading
	is new and `Err` -- that it did not change since last call.

	A call to this function lazily evaluates the engine's state
	since the last call to any of the [`Simulation`] methods.

	## Examples

	Assume a builder `sim` previously created and populated
	with kinematic targets, and current time `now`, in ms.

	```
	if let Ok(reading) = sim.", stringify!($f), "(now) {
		// consume reading
	}
	```
	")]
	pub fn $f(&mut self, tim: u32) -> Result<$t, $t> {
		self.step(tim).m.$d.consume()
	}
));

/////////////////////////////////////////////////////////////////////////////
// Simulation
/////////////////////////////////////////////////////////////////////////////

pub struct Simulation<const N: usize> {
    m: Model,
    p: CannedProfile<N>,
    tim: u32,
    delay: u32,
    rate: u32,
}

impl<const N: usize> Simulation<N> {
    /// Creates a new simulation, consuming the builder `b`.
    ///
    /// `new()` will return the non-copyable simulation struct itself.
    ///
    /// `delay` specifies the time during which no work should be done
    /// towards kinematic targets. This is useful to simulate inherent
    /// sensor drift while a vehicle is stationary.
    pub fn new(mut b: impl SimBuilder, delay: u32) -> Self {
        let rate = b.rate();
        Self {
            m: Model::new(rate, b.imu(), b.baro()),
            p: CannedProfile::new(delay),
            tim: 0,
            delay,
            rate,
        }
    }

    /// Adds an absolute kinematic target at relative time offset.
    ///
    /// `fix()` will return a unique reference to the simulation struct
    /// for the further data point chaining.
    ///
    /// ## Examples
    ///
    /// Assume a builder `b` previously created.
    ///
    /// ```
    /// let sim = Simulation::new(b, 1000);
    /// sim
    ///     .fix(([0.2, 0.3, 12.2], [0.1, 0.4, 2.3]), 200)
    ///     .fix(([0.3, 0.2, 26.1], [0.3, 0.3, 1.0]), 150);
    /// ```
    ///
    /// The first call to [`fix`] will create a new data point with the
    /// timestamp 1200 ms from the creation of `sim`. The first triplet
    /// will represent accelerometer values and the second -- gyroscope.
    ///
    /// The second call to [`fix`] will create a new data point with the
    /// timestamp 1350 ms, using new data directly as new target.
    pub fn fix(&mut self, f: Motion, dur: u32) -> &mut Self {
        self.p.append(f, dur, false);
        self
    }

    /// Adds a relative kinematic target at relative time offset.
    ///
    /// `add()` will return a unique reference to the simulation struct
    /// for the further data point chaining.
    ///
    /// ## Examples
    ///
    /// Assume a builder `b` previously created.
    ///
    /// ```
    /// let sim = Simulation::new(b, 1000);
    /// sim
    ///     .fix(([0.2, 0.3, 12.2], [0.1, 0.4, 2.3]), 200)
    ///     .add(([0.3, 0.2, 26.1], [0.3, 0.3, 1.0]), 150);
    /// ```
    ///
    /// The call to [`add`] will create a new data point with the timestamp
    /// 1350 ms from the creation of `sim`, and the provided kinematic data
    /// will be added to that of a previous data point. The new target, in
    /// this case, will hold `[0.5, 0.5, 38.3]` and `[0.4, 0.7, 3.3]`.
    pub fn add(&mut self, f: Motion, dur: u32) -> &mut Self {
        self.p.append(f, dur, true);
        self
    }

    /// Breaks elapsed time into equal intervals equal to engine rate
    /// and calls derivation logic multiple times. Lazy.
    fn step(&mut self, mut tim: u32) -> &mut Self {
        if tim > self.delay {
            tim -= self.delay;
        }

        if tim.wrapping_sub(self.tim) < self.rate {
            return self;
        }

        if let Some(target) = self.p.linearize(tim) {
            let secs = self.rate as f32 / 1000.0;

            while tim.wrapping_sub(self.tim) >= self.rate {
                self.tim = self.tim.wrapping_add(self.rate);
                self.m.derive(secs, target);
            }
        }

        self
    }

    getter!("accelerometer", accel, acl, XYZ);
    getter!("gyroscope", angvel, gyr, XYZ);
    getter!("magnetometer", magfield, mag, XYZ);
    getter!("barometer", pressure, bar, f32);
}
