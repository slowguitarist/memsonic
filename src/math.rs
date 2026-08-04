//! # math
//!
//! A collection is scalar, vector, and matrix API
//! specific to the simulation.

use crate::{
    Alignment, Bias, XYZ,
    env::{TOLER, rand},
};
use core::{array::from_fn, f32::consts::PI};
use libm::{cosf, powf, sinf, tanhf};

/////////////////////////////////////////////////////////////////////////////
// Functions on f32
/////////////////////////////////////////////////////////////////////////////

#[inline(always)]
pub(crate) fn leash(pt: f32, d: f32) -> f32 {
    let r = rand();
    pt + d * f32::from_bits((r >> 9) | 0x3f000000 | (r << 31))
}

#[inline(always)]
pub(crate) const fn sq(x: f32) -> f32 {
    x * x
}

#[inline(always)]
pub(crate) const fn ceil(x: f32) -> f32 {
    (x + 1.0) as i32 as f32
}

#[inline(always)]
pub(crate) const fn basically1(x: f32) -> bool {
    (x - 1.0).abs() < TOLER
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
        r => Some(1.0 / r),
    }
}

#[inline(always)]
pub(crate) fn interpolate(b: impl Blend, x1: f32, x0: f32, x: f32) -> f32 {
    assert!(x0 < x1);
    x0 + (x1 - x0) * b.blend(x.clamp(0.0, 1.0))
}

/////////////////////////////////////////////////////////////////////////////
// Traits
/////////////////////////////////////////////////////////////////////////////

pub(crate) trait Vector
where
    Self: Copy,
{
    fn add(self, v: Self) -> Self;
    fn norm(&self) -> f32;
    fn lerp(self, v: Self, k: f32) -> Self;
}

pub(crate) trait Randomize<const N: usize>
where
    Self: Copy,
{
    /// The behavior of this function is entirely implementation-defined;
    /// the trait only defines the API for static dispatch.
    ///
    /// `d` represents "the degree of randomness" from 0 to 1, where 1 is
    /// most random. It is up to the implementation to trim values of `d`
    /// as well as to determine a function that calculates randomness
    /// from `d`.
    fn randomize(self, d: f32) -> Self;
}

pub(crate) trait SquareMatrix<const N: usize>
where
    Self: Copy,
{
    fn identity() -> Self;
}

pub(crate) trait Blend
where
    Self: Copy,
{
    type Params;

    fn new(p: Self::Params) -> Self;

    /// Takes a normalized input `t` and a curvature `k`, and applies
    /// a smooth function with the output range between 0 and 1.
    fn blend(&self, x: f32) -> f32;
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
            return 1.0;
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
#[derive(Clone, Copy)]
pub(crate) struct Barron {
    s: f32,
    t: f32,
}

impl Blend for Barron {
    type Params = (f32, f32);

    fn new(p: Self::Params) -> Self {
        Self {
            s: p.0.clamp(0.0, 1e5),
            t: p.1.clamp(0.0, 1.0),
        }
    }

    fn blend(&self, x: f32) -> f32 {
        let c = self.s * (self.t - x);

        if x < self.t {
            self.t * x / (x + c + 1e-9)
        } else {
            1.0 + (1.0 - self.t) * (x - 1.0) / (1.0 - x - c + 1e-9)
        }
    }
}

#[allow(unused)]
#[derive(Default, Clone, Copy)]
pub(crate) struct TrigIncr {
    k: f32,
}

impl Blend for TrigIncr {
    type Params = f32;

    fn new(p: Self::Params) -> Self {
        Self { k: p }
    }

    fn blend(&self, x: f32) -> f32 {
        0.5 * (1.0 - cosf(PI * powf(x, self.k.abs())))
    }
}

#[allow(unused)]
#[derive(Default, Clone, Copy)]
pub(crate) struct TrigDecr {
    k: f32,
}

impl Blend for TrigDecr {
    type Params = f32;

    fn new(p: Self::Params) -> Self {
        Self { k: p }
    }

    fn blend(&self, x: f32) -> f32 {
        0.5 * (1.0 + cosf(PI * powf(x, self.k.abs())))
    }
}

#[allow(unused)]
#[derive(Default, Clone, Copy)]
pub(crate) struct Hyperbolic {
    k: f32,
}

impl Blend for Hyperbolic {
    type Params = f32;

    fn new(p: Self::Params) -> Self {
        if let -1.0..=1.0 = p {
            Self { k: 0.0 }
        } else {
            Self { k: p }
        }
    }

    fn blend(&self, x: f32) -> f32 {
        if self.k == 0.0 {
            return x;
        }
        let kh = 0.5 * self.k;
        0.5 * (1.0 + tanhf(self.k * x - kh) / tanhf(kh))
    }
}

/////////////////////////////////////////////////////////////////////////////
// Quaternion
/////////////////////////////////////////////////////////////////////////////

/// Interpreted as WXYZ.
#[derive(Debug)]
pub(crate) struct Quaternion(f32, f32, f32, f32);

impl Quaternion {
    pub(crate) fn new() -> Self {
        Self(1.0, 0.0, 0.0, 0.0)
    }

    pub(crate) const fn rotate_w2b(&self, v: XYZ) -> XYZ {
        let tx = 2.0 * (-self.2 * v[2] + self.3 * v[1]);
        let ty = 2.0 * (-self.3 * v[0] + self.1 * v[2]);
        let tz = 2.0 * (-self.1 * v[1] + self.2 * v[0]);
        [
            v[0] + self.0 * tx + (-self.2 * tz + self.3 * ty),
            v[1] + self.0 * ty + (-self.3 * tx + self.1 * tz),
            v[2] + self.0 * tz + (-self.1 * ty + self.2 * tx),
        ]
    }

    pub(crate) const fn rotate_b2w(&self, v: XYZ) -> XYZ {
        let tx = 2.0 * (self.2 * v[2] - self.3 * v[1]);
        let ty = 2.0 * (self.3 * v[0] - self.1 * v[2]);
        let tz = 2.0 * (self.1 * v[1] - self.2 * v[0]);
        [
            v[0] + self.0 * tx + (self.2 * tz - self.3 * ty),
            v[1] + self.0 * ty + (self.3 * tx - self.1 * tz),
            v[2] + self.0 * tz + (self.1 * ty - self.2 * tx),
        ]
    }

    /// Only accurate for very small dt (< 0.0005s).
    fn linear_approx(&mut self, w: XYZ, dt: f32) {
        let dq_w = -0.5 * (self.1 * w[0] + self.2 * w[1] + self.3 * w[2]) * dt;
        let dq_x = 0.5 * (self.0 * w[0] + self.2 * w[2] - self.3 * w[1]) * dt;
        let dq_y = 0.5 * (self.0 * w[1] - self.1 * w[2] + self.3 * w[0]) * dt;
        let dq_z = 0.5 * (self.0 * w[2] + self.1 * w[1] - self.2 * w[0]) * dt;

        self.0 += dq_w;
        self.1 += dq_x;
        self.2 += dq_y;
        self.3 += dq_z;

        let d_sq = sq(self.0) + sq(self.1) + sq(self.2) + sq(self.3);

        if !basically1(d_sq)
            && let Some(inv) = invsqrt(d_sq)
        {
            self.0 *= inv;
            self.1 *= inv;
            self.2 *= inv;
            self.3 *= inv;
        }
    }

    /// Somewhat inaccruate for very small dt (< 0.0005s).
    fn exponent_approx(&mut self, w: XYZ, dt: f32) {
        let norm_w = sqrt(sq(w[0]) + sq(w[1]) + sq(w[2]));
        if norm_w < 1e-8 {
            return;
        }

        let theta = 0.5 * norm_w * dt;
        let cos_t = cosf(theta);
        let sin_t = sinf(theta) / norm_w;

        let dq_w = cos_t;
        let dq_x = w[0] * sin_t;
        let dq_y = w[1] * sin_t;
        let dq_z = w[2] * sin_t;

        // self = self * dq
        let w0 = self.0 * dq_w - self.1 * dq_x - self.2 * dq_y - self.3 * dq_z;
        let x0 = self.0 * dq_x + self.1 * dq_w + self.2 * dq_z - self.3 * dq_y;
        let y0 = self.0 * dq_y - self.1 * dq_z + self.2 * dq_w + self.3 * dq_x;
        let z0 = self.0 * dq_z + self.1 * dq_y - self.2 * dq_x + self.3 * dq_w;

        self.0 = w0;
        self.1 = x0;
        self.2 = y0;
        self.3 = z0;
    }

    pub(crate) fn integrate(&mut self, w: XYZ, dt: f32) -> &Self {
        if dt < 0.0005 {
            self.linear_approx(w, dt);
        } else {
            self.exponent_approx(w, dt);
        }
        self
    }
}

/////////////////////////////////////////////////////////////////////////////
// Tests
/////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    const TOL: f32 = 1e-5;

    #[test]
    fn test_quaternion_rotations() {
        // Rotate by 90 degrees around the Z axis.
        let half_angle = PI / 4.0;
        let q = Quaternion(cosf(half_angle), 0.0, 0.0, sinf(half_angle));

        let v_world = [1.0, 0.0, 0.0];

        // Should yield [0.0, 1.0, 0.0]
        let v_body = q.rotate_b2w(v_world);
        assert!(v_body[0].abs() < TOL);
        assert!((v_body[1] - 1.0).abs() < TOL);
        assert!(v_body[2].abs() < TOL);

        // Should yield [1.0, 0.0, 0.0]
        let v_reverted = q.rotate_w2b(v_body);
        assert!((v_reverted[0] - 1.0).abs() < TOL);
        assert!(v_reverted[1].abs() < TOL);
        assert!(v_reverted[2].abs() < TOL);
    }

    #[test]
    fn test_exponent_approx() {
        let mut q = Quaternion::new();
        let w = [PI, 0.0, 0.0];
        let total_dt = 0.5;
        let steps = 100;
        let dt = total_dt / steps as f32;

        for _ in 0..steps {
            q.integrate(w, dt);
        }

        let expected_w = cosf(PI / 4.0);
        let expected_x = sinf(PI / 4.0);

        assert!((q.0 - expected_w).abs() < TOL);
        assert!((q.1 - expected_x).abs() < TOL);
        assert!(q.2.abs() < TOL);
        assert!(q.3.abs() < TOL);
    }

    #[test]
    fn test_barron_blend_continuity() {
        let threshold = 0.7;
        let b = Barron::new((5.0, threshold));

        let val_below = b.blend(threshold - 1e-9);
        let val_above = b.blend(threshold + 1e-9);

        assert!(
            (val_below - val_above).abs() < TOL,
            "Barron blend discontinuity at threshold. Below: {}, Above: {}",
            val_below,
            val_above
        );
    }

    #[test]
    fn test_hyperbolic_blend_bounds() {
        let b = Hyperbolic::new(2.0);

        let start = b.blend(0.0);
        let end = b.blend(1.0);

        // Test bounds
        assert!(start.abs() < TOL);
        assert!((end - 1.0).abs() < TOL);
    }
}
