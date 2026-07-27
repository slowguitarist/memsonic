use libm::{cosf, powf, tanhf};

use crate::{Alignment, Bias, XYZ, common::rand};
use core::{f32::consts::PI, array::from_fn};

/////////////////////////////////////////////////////////////////////////////
// Functions on f32
/////////////////////////////////////////////////////////////////////////////

pub(crate) fn leash(pt: f32, d: f32) -> f32 {
	let r = rand();
	pt + d * f32::from_bits((r >> 9) | 0x3f000000 | (r << 31))
}

#[inline(always)]
pub(crate) const fn sq(x: f32) -> f32 {
	x * x
}

#[inline(always)]
pub(crate) const fn fabs(x: f32) -> f32 {
	(x.to_bits() & !(1 << 31)) as f32
}

#[inline(always)]
pub(crate) const fn ceil(x: f32) -> f32 {
	(x + 1.0) as i32 as f32
}

#[inline(always)]
pub(crate) const fn basically1(x: f32) -> bool {
	fabs(x - 1.0) < crate::common::TOLER
}

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
	match sqrt(x) {
		0.0 => None,
		r => Some(1.0 / r)
	}
}

pub(crate) fn interpolate<B: Blend>(
	x1: f32,
	x0: f32,
	d: f32,
	k: f32
) -> f32 {
	assert!(x0 < x1);
	x0 + (x1 - x0) * B::default().blend(d.clamp(0.0, 1.0), k)
}

/////////////////////////////////////////////////////////////////////////////
// Traits
/////////////////////////////////////////////////////////////////////////////

pub(crate) trait Vector
	where Self: Copy
{
	fn add(self, v: Self) -> Self;
	fn norm(&self) -> f32;
	fn lerp(self, v: Self, k: f32) -> Self;
}

pub(crate) trait Randomize<const N: usize>
	where Self: Copy
{
	fn randomize(self, d: f32) -> Self;
}

pub(crate) trait SquareMatrix<const N: usize>
	where Self: Copy
{
	fn identity() -> Self;
}

pub(crate) trait Blend
	where Self: Copy + Default
{
	fn blend(self, t: f32, k: f32) -> f32;
}

/////////////////////////////////////////////////////////////////////////////
// Trait implementations
/////////////////////////////////////////////////////////////////////////////

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

impl<const N: usize> Randomize<N> for Bias<N> {
	fn randomize(self, d: f32) -> Self {
		let s = d.clamp(0.0, 1.0);
		let mut k = self.iter().copied();
		from_fn(|_| leash(k.next().unwrap_or_default(), s))
	}
}

impl<const N: usize> SquareMatrix<N> for Alignment<N> {
	fn identity() -> Self {
		from_fn(|i| {
			let mut r = [0.0; N];
			r[i] = 1.0;
			r
		})
	}
}

impl<const N: usize> Randomize<N> for Alignment<N> {
	fn randomize(self, d: f32) -> Self {
		let s = d * 0.2;
		let mut k = self.iter().copied();
		from_fn(|_| k.next().unwrap_or([0.0; N]).randomize(s))
	}
}

#[allow(unused)]
#[derive(Default, Clone, Copy)]
pub(crate) struct SchilckBias;

impl Blend for SchilckBias {
	fn blend(self, t: f32, k: f32) -> f32 {
		t / ((1.0 / k.clamp(0.0, 1.0) - 2.0) * (1.0 - t) + 1.0)
	}
}

#[allow(unused)]
#[derive(Default, Clone, Copy)]
pub(crate) struct TrigIncr;

impl Blend for TrigIncr {
	fn blend(self, t: f32, k: f32) -> f32 {
		0.5 * (1.0 - cosf(PI * powf(t, fabs(k))))
	}
}

#[allow(unused)]
#[derive(Default, Clone, Copy)]
pub(crate) struct TrigDecr;

impl Blend for TrigDecr {
	fn blend(self, t: f32, k: f32) -> f32 {
		0.5 * (1.0 + cosf(PI * powf(t, fabs(k))))
	}
}

#[allow(unused)]
#[derive(Default, Clone, Copy)]
pub(crate) struct Hyperbolic;

impl Blend for Hyperbolic {
	fn blend(self, t: f32, k: f32) -> f32 {
		if let -1.0..=1.0 = k {
			return t
		}
		let kh = 0.5 * k;
		0.5 * (1.0 + tanhf(k * t - kh) / tanhf(kh))
	}
}

/////////////////////////////////////////////////////////////////////////////
// Quaternion
/////////////////////////////////////////////////////////////////////////////

/// Interpreted as WXYZ.
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