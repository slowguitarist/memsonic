//! # noise
//! 
//! Generic noise based on Irwin-Hall distribution. 'Fast' version breaks the normal
//! into four independent small (byte) distributions to reduce floating-point mul/div.
//! Normal version should always be used unless the engine falls behind desired ODR. 

trait Rand {
	fn rand(self) -> Self where Self: Copy;
}

impl Rand for u32 {
	fn rand(mut self) -> u32 { 
		self ^= self << 13; 
		self ^= self >> 17; 
		self ^= self << 5; 
		self 
	}
}

pub(crate) trait NoiseMode {}

pub(crate) struct Normal;
impl NoiseMode for Normal {}

pub(crate) struct Fast;
impl NoiseMode for Fast {}

pub(crate) trait Distort<T: Copy, M: NoiseMode> {
	fn distort(&mut self) -> T;
}

pub(crate) struct Gaussian {
	pub(crate) state: u32,
	pub(crate) sigma: f32
}

impl Gaussian {
	pub(crate) fn new(state: u32, sigma: f32) -> Self {
		Self { state, sigma }
	}
}

impl Distort<f32, Normal> for Gaussian {
	fn distort(&mut self) -> f32 {
		let mut s = 0.0;

		for _ in 0..12 {
			self.state = self.state.rand();
			s += (self.state as f32) / (u32::MAX as f32);
		}

		(s - 6.0) * self.sigma
	}
}

impl Distort<f32, Fast> for Gaussian {
	fn distort(&mut self) -> f32 {
		let mut s = 0;

		for _ in 0..3 {
			let r = self.state.rand();
			s += (r & 0xFF) + ((r >> 8) & 0xFF) + ((r >> 16) & 0xFF) + (r >> 24);
			self.state = r;
		}

		(s as f32 - 1530.0) / 256.0 * self.sigma
	}
}