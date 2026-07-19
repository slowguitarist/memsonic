//! # config
//! 
//! Global constants and sensor configuration structs.

/// Earth (at 43.0019167, -78.787083)

pub(crate) const EARTH_MAGFIELD: [f32; 3] = [19_133.4, -3_452.9, 48_806.1];
pub(crate) const G_SI_NED: f32 = 9.80665;
pub(crate) const SEA_PRS: f32 = 101_900.0;
pub(crate) const SEA_TMP: f32 = 298.0;
pub(crate) const LAPSE_RT: f32 = 0.0065;
pub(crate) const R_EXP: f32 = 5.25588;

/// Barometric

pub(crate) const SP_GAS: f32 = 287.052874;
pub(crate) const SP_HEAT: f32 = 1.4;
pub(crate) const SHOCK_DEPTH: f32 = -2000.0;
pub(crate) const DRIFT_PER_C: f32 = 2.5;
pub(crate) const MAX_SUBNATICA: f32 = 2000.0;

/// General

pub(crate) const TOLER: f32 = 1e-3;

/// Sensor config

#[derive(Clone, Copy)]
pub struct CoreConf {
	pub(crate) odr: u32,
	pub(crate) cutoff: f32,
	pub(crate) qbw: f32,
	pub(crate) sens: f32
}

#[derive(Clone, Copy)]
pub struct BiasConf<const N: usize> {
	pub(crate) state: u32,
	pub(crate) sigma: f32,
	pub(crate) bias: [f32; N],
	pub(crate) align: [[f32; N]; N],
}

pub struct SensorConf<const N: usize> {
	pub(crate) k: CoreConf,
	pub(crate) m: BiasConf<N>,
	pub(crate) s: Option<f32>
}