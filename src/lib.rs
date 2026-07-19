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
pub struct Motion(pub(crate) XYZ, pub(crate) XYZ);

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
	rate: u32
}

impl<const N: usize> Simulation<N> {
	pub fn new(mut b: impl SimBuilder) -> Self {
		let r: u32 = b.rate();
		Self {
			m: Model::new(r, &b.imu(), &b.baro()),
			p: CannedProfile::new(),
			tim: 0,
			rate: r
		}
	}

	pub fn point(&mut self, f: Motion, dur: u32) -> &mut Self {
		self.p.append(f, dur, false);
		self
	}

	pub fn add(&mut self, f: Motion, dur: u32) -> &mut Self {
		self.p.append(f, dur, true);
		self
	}

	fn step(&mut self, tim: u32) -> Result<(), ()> {
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