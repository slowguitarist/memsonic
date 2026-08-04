//! # env
//!
//! Environment API used to represent conditions specific to a launch site.

use crate::{XYZ, math::sqrt};
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};
use libm::{atan2f, atanf, sinf, tanf};

/// ISA tropospheric lapse rate in degrees Celsius per meter.
pub(crate) const LAPSE_RT: f32 = 0.0065;

/// Specific gas "constant," changes marginally with humidity.
pub(crate) const SP_GAS: f32 = 287.05287;

/// Specific heat "constant," changes marginally with temperature.
pub(crate) const SP_HEAT: f32 = 1.4;

// Engine-specific heuristics.

pub(crate) const SHOCK_DEPTH: f32 = -2000.0;
pub(crate) const DRIFT_PER_C: f32 = 0.5;
pub(crate) const MAX_SUBNATICA: f32 = 2000.0;
pub(crate) const TOLER: f32 = 1e-3;

pub(crate) fn rand() -> u32 {
    // I, undersigned, voluntarily give up any notion of order, total or partial,
    // on this piece of memory. Here it is sufficient for a thread of execution to
    // observe at least its own writes, and acceptable if different threads end up
    // hammering the same values on their respective cache lines before syncing.
    static STATE: AtomicU32 = AtomicU32::new(0xdeadfa11);

    STATE
        .try_update(Relaxed, Relaxed, |mut v| {
            v ^= v << 13;
            v ^= v >> 17;
            v ^= v << 5;
            Some(v)
        })
        .unwrap_or(0xf70a57ed)
}

pub(crate) struct Conditions {
    pub(crate) mag: XYZ,
    pub(crate) sea_tmp: f32,
    pub(crate) sea_prs: f32,
    pub(crate) g_si_ned: f32,
    pub(crate) r_exp: f32,
}

impl Conditions {
    fn normal_gravity(mag: XYZ) -> f32 {
        const GRS80_GE: f32 = 9.780327;
        const GRS80_A: f32 = 0.0053024;
        const GRS80_B: f32 = 0.0000058;

        let horiz = sqrt(mag[0] * mag[0] + mag[1] * mag[1]);
        let inclin = atan2f(mag[2], horiz);

        let lat = atanf(0.5 * tanf(inclin));
        let sinl = sinf(lat);
        let sin2 = sinf(2.0 * lat);

        GRS80_GE * (1.0 + GRS80_A * sinl * sinl - GRS80_B * sin2 * sin2)
    }

    pub fn new(mag: XYZ, sea_prs: f32, sea_tmp: f32) -> Self {
        let g_si_ned = Self::normal_gravity(mag);

        Self {
            mag,
            sea_tmp,
            sea_prs,
            g_si_ned,
            r_exp: g_si_ned / (SP_GAS * LAPSE_RT),
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
// Launch site API
/////////////////////////////////////////////////////////////////////////////

/// Initial conditions for the simulation, from which
/// other simulation constants are heuristically derived.
pub struct Surface {
    pub mag: XYZ,
    pub tmp: f32,
    pub prs: f32,
}

impl Surface {
    pub(crate) fn into_cond(self) -> Conditions {
        Conditions::new(self.mag, self.prs, self.tmp)
    }
}

/// Structs implementing this trait must return the [`Surface`] type filled with
/// magnetic field, sea level temperature and pressure specific to their location.
pub trait Setup {
    fn setup() -> Surface;
}

/// Average mid July at Furnas Hall, University at Buffalo.
/// Lat: 43.0019167, Lon: -78.787083.
pub struct BuffaloJuly;

impl Setup for BuffaloJuly {
    fn setup() -> Surface {
        Surface {
            mag: [19_133.4, -3_452.9, 48_806.1],
            tmp: 298.0,
            prs: 101_900.0,
        }
    }
}
