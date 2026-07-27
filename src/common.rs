//! # common
//! 
//! Commonly used constants and global functions

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

/////////////////////////////////////////////////////////////////////////////
// Earth at at 43.0019167, -78.787083
/////////////////////////////////////////////////////////////////////////////

pub(crate) const EARTH_MAGFIELD: [f32; 3] = [19_133.4, -3_452.9, 48_806.1];
pub(crate) const G_SI_NED: f32 = 9.80665;
pub(crate) const SEA_PRS: f32 = 101_900.0;
pub(crate) const SEA_TMP: f32 = 298.0;
pub(crate) const LAPSE_RT: f32 = 0.0065;
pub(crate) const R_EXP: f32 = 5.25588;
pub(crate) const SP_GAS: f32 = 287.05287;
pub(crate) const SP_HEAT: f32 = 1.4;
pub(crate) const SHOCK_DEPTH: f32 = -2000.0;
pub(crate) const DRIFT_PER_C: f32 = 2.5;
pub(crate) const MAX_SUBNATICA: f32 = 2000.0;
pub(crate) const TOLER: f32 = 1e-3;

/////////////////////////////////////////////////////////////////////////////
// Pocket PRNG
/////////////////////////////////////////////////////////////////////////////

/// Returns a pseudo-random 32-bit unsigned integer.
pub(crate) fn rand() -> u32 {
	// I, undersigned, voluntarily give up any notion of order, total or partial,
	// on this piece of memory. Here it is sufficient for a thread of execution to
	// observe at least its own writes, and acceptable if different threads end up
	// hammering the same values on their respective cache lines before syncing.
	static STATE: AtomicU32 = AtomicU32::new(0xdeadfa11);

	STATE.try_update(Relaxed, Relaxed, |mut v| {
		v ^= v << 13;
		v ^= v >> 17;
		v ^= v << 5;
		Some(v)
	}).unwrap_or(0xf70a57ed)
}