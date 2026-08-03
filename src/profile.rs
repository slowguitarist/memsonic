//! # profile
//!
//! This module is a storage unit for the user's flight points.
//! It includes a time-based "dispenser" that provides the engine's
//! model with a flight plan by interpolating between two points
//! on the user's graph.

use crate::{XYZ, math::Vector};

/// A theoretical kinematic target (acceleration, angular velocity).
pub(crate) type Motion = (XYZ, XYZ);

#[derive(Clone, Copy, Default)]
struct ModelFrame {
    ts: u32,
    acc: XYZ,
    ang: XYZ,
}

pub(crate) struct CannedProfile<const N: usize> {
    count: usize,
    f: [ModelFrame; N],
}

impl<const N: usize> CannedProfile<N> {
    // Starting point at [0] always has a 0 timestamp and no kinematics.
    pub(crate) fn new(delay: u32) -> Self {
        assert!(N > 0);
        let mut me = CannedProfile {
            count: 1,
            f: [ModelFrame::default(); N],
        };
        me.f[0].ts = delay;
        me
    }

    pub(crate) fn append(&mut self, f: Motion, dur: u32, rel: bool) {
        if self.count < N {
            let prev = self.f[self.count - 1];

            self.f[self.count] = ModelFrame {
                ts: prev.ts + dur,
                acc: if rel { f.0.add(prev.acc) } else { f.0 },
                ang: if rel { f.1.add(prev.ang) } else { f.1 },
            };

            self.count += 1;
        }
    }

    pub(crate) fn linearize(&self, tim: u32) -> Option<Motion> {
        if tim <= self.f[0].ts {
            return Some((self.f[0].acc, self.f[0].ang));
        } else if tim >= self.f[self.count - 1].ts {
            return None;
        }

        let mut i = 0;
        while i < self.count - 1 && self.f[i + 1].ts < tim {
            i += 1;
        }

        let p0 = &self.f[i];
        let p1 = &self.f[i + 1];
        let k = (tim - p0.ts) as f32 / (p1.ts - p0.ts) as f32;

        Some((p0.acc.lerp(p1.acc, k), p0.ang.lerp(p1.ang, k)))
    }
}
