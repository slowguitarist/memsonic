#![no_std]
#![no_main]

use crate::{builder::SimBuilder, model::Model, profile::CannedProfile, sensors::Extract};

mod filters;
mod model;
mod sensors;
mod noise;
mod config;
mod math;
mod profile;
mod builder;

pub type XYZ = [f32; 3];

#[derive(Clone, Copy)]
pub struct Motion(pub XYZ, pub XYZ);

macro_rules! getter {
	($name:ident, $field:ident, $type:ident) => {
		pub fn $name(&mut self, tim: u32) -> Result<$type, $type> {
			self.step(tim)
				.map(|_| self.m.$field.extract())
				.map_err(|_| self.m.$field.extract())
		}
	};
}

pub struct Simulation<const N: usize> {
	m: Model,
	p: CannedProfile<N>,
	tim: u32,
	delay: u32,
	rate: u32
}

impl<const N: usize> Simulation<N> {
	/// Creates a new simulation, consuming the builder `b`.
	/// 
	/// `new()` will return the non-copyable simulation struct itself.
	/// 
	/// `delay` specifies the time during which no work should be done
	/// towards kinematic targets. This is useful to simulate inherent
	/// sensor drift while a vehicle is stationary.
	pub fn new<'a>(mut b: impl SimBuilder<'a>, delay: u32) -> Self {
		let rate = b.rate();
		Self {
			m: Model::new(rate, b.imu(), b.baro()),
			p: CannedProfile::new(delay),
			tim: 0,
			delay,
			rate
		}
	}

	/// Adds an absolute kinematic target at relative time offset.
	/// 
	/// `fix()` will return a unique reference to the simulation struct
	/// for the further data point chaining.
	/// 
	/// ## Examples
	/// 
	/// Assume a builder `bob` previously created.
	/// 
	/// ```
	/// let sim = Simulation::new(bob, 1000);
	/// sim
	/// 	.fix(([0.2, 0.3, 12.2], [0.1, 0.4, 2.3]), 200)
	/// 	.fix(([0.3, 0.2, 26.1], [0.3, 0.3, 1.0]), 150);
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
	/// Assume a builder `bob` previously created.
	/// 
	/// ```
	/// let sim = Simulation::new(bob, 1000);
	/// sim
	/// 	.fix(([0.2, 0.3, 12.2], [0.1, 0.4, 2.3]), 200)
	/// 	.add(([0.3, 0.2, 26.1], [0.3, 0.3, 1.0]), 150);
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

	fn step(&mut self, mut tim: u32) -> Result<(), ()> {
		if tim > self.delay {
			tim -= self.delay;
		}
		
		if tim.wrapping_sub(self.tim) < self.rate {
			return Err(())
		}

		if let Some(target) = self.p.interpolate(tim) {
			let secs = self.rate as f32 / 1000.0;

			while tim.wrapping_sub(self.tim) >= self.rate {
				self.tim = self.tim.wrapping_add(self.rate);
				self.m.derive(secs, target);
			}

			return Ok(())
		}

		Err(())
	}

	getter!(accel, acl, XYZ);
	getter!(angvel, gyr, XYZ);
	getter!(magfield, mag, XYZ);
	getter!(pressure, bar, f32);
}