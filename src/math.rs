use crate::XYZ;

/// Math on f32.

#[inline(always)]
pub(crate) const fn sq(x: f32) -> f32 {
	x * x
}

#[inline(always)]
pub(crate) const fn fabs(x: f32) -> f32 {
	(x as u32 & !(1 << 31)) as f32
}

#[inline(always)]
pub(crate) const fn basically1(x: f32) -> bool {
	fabs(x - 1.0) < crate::config::TOLER
}

// Well... only ARM MCUs have FPUs, right?
#[inline(always)]
pub(crate) fn sqrt(x: f32) -> f32 {
	#[cfg(all(target_arch = "arm", target_feature = "vfp2"))]
	{
		let sqrt: f32;
		unsafe {
			core::arch::asm!(
				"vsqrt.f32 {0}, {1}",
				out(sreg) sqrt, in(sreg) x,
				options(pure, nomem, nostack)
			);
		}
		sqrt
	}

	#[cfg(not(all(target_arch = "arm", target_feature = "vfp2")))]
	{
		libm::sqrtf(x)
	}
}

#[inline(always)]
pub(crate) fn invsqrt(x: f32) -> Option<f32> {
	let sqrt = sqrt(x);
	if sqrt == 0.0 {
		return None
	}
	Some(1.0 / sqrt)
}

/// Math on triplets of f32.

pub(crate) trait Vector where Self: Copy {
	fn add(self, v: Self) -> Self;
	fn norm(&self) -> f32;
	fn lerp(self, v: Self, k: f32) -> Self;
}

impl Vector for XYZ {
	fn add(self, v: Self) -> Self {
		[self[0] + v[0], self[1] + v[1], self[2] + v[2]]
	}

	fn norm(&self) -> f32 {
		let sq = sq(self[0]) + sq(self[1]) + sq(self[2]);
		if basically1(sq) {
			return sq
		}
		sqrt(sq)
	}

	fn lerp(self, v: Self, k: f32) -> Self {
		[
			self[0] * (1.0 - k) + v[0] * k,
			self[1] * (1.0 - k) + v[1] * k,
			self[2] * (1.0 - k) + v[2] * k,
		]
	}
}

/// Math on quaternion.

// Interpreted as WXYZ.
pub(crate) struct Quaternion(f32, f32, f32, f32);

impl Quaternion {
	pub(crate) fn new() -> Self {
		Self ( 1.0, 0.0, 0.0, 0.0 )
	}

	pub(crate) const fn rotate_w2b(&self, v: XYZ) -> XYZ {
		let tx = 2.0 * (-self.2 * v[2] + self.3 * v[1]);
		let ty = 2.0 * (-self.3 * v[0] + self.1 * v[2]);
		let tz = 2.0 * (-self.1 * v[1] + self.2 * v[0]);
		[
			v[0] + self.0 * tx + (-self.2 * tz + self.3 * ty),
			v[1] + self.0 * ty + (-self.3 * tx + self.1 * tz),
			v[2] + self.0 * tz + (-self.1 * ty + self.2 * tx)
		]
	}

	pub(crate) const fn rotate_b2w(&self, v: XYZ) -> XYZ {
		let tx = 2.0 * (self.2 * v[2] - self.3 * v[1]);
		let ty = 2.0 * (self.3 * v[0] - self.1 * v[2]);
		let tz = 2.0 * (self.1 * v[1] - self.2 * v[0]);
		[
			v[0] + self.0 * tx + (self.2 * tz - self.3 * ty),
			v[1] + self.0 * ty + (self.3 * tx - self.1 * tz),
			v[2] + self.0 * tz + (self.1 * ty - self.2 * tx)
		]
	}

	pub(crate) fn integrate(&mut self, w: XYZ, dt: f32) -> &Self {
		let dq_w = -0.5 * (self.1 * w[0] + self.2 * w[1] + self.3 * w[2]) * dt;
    	let dq_x =  0.5 * (self.0 * w[0] + self.2 * w[2] - self.3 * w[1]) * dt;
    	let dq_y =  0.5 * (self.0 * w[1] - self.1 * w[2] + self.3 * w[0]) * dt;
    	let dq_z =  0.5 * (self.0 * w[2] + self.1 * w[1] - self.2 * w[0]) * dt;

		self.0 += dq_w;
		self.1 += dq_x;
		self.2 += dq_y;
		self.3 += dq_z;

		let d_sq = sq(self.0) + sq(self.1) + sq(self.2) + sq(self.3);

		if !basically1(d_sq) && let Some(inv) = invsqrt(d_sq) {
			self.0 *= inv;
			self.1 *= inv;
			self.2 *= inv;
			self.3 *= inv;
		}
		
		self
	}
}