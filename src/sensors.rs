use libm::{expf};

use crate::XYZ;
use crate::config::*;
use crate::filters::BiquadType::LowPass;
use crate::math::*;
use crate::model::*;
use crate::filters::*;
use crate::noise::*;

pub(crate) trait Evaluate {
	fn evaluate(&mut self, s: &ModelState) -> &Self;
}

struct SensorCore<const N: usize> {
	cic: CICConf,
	biq: BiquadCoef,
	fir: [EmuCIC; N],
	iir: [Biquad; N],
	meas: [f32; N]
}

impl<const N: usize> SensorCore<N> {
	pub(crate) fn new(
		odr: u32,
		engrate: u32,
		biqtype: BiquadType,
		sens: f32
	) -> Self {
		Self {
			cic: CICConf::new(odr / engrate, sens),
			biq: BiquadCoef::derive(biqtype, odr as f32),
			fir: [EmuCIC::new(); N],
			iir: [Biquad::new(); N],
			meas: [0.0; N]
		}
	}
}

impl<const N: usize> SensorCore<N> {
	fn gather(&mut self, mut n: impl Iterator<Item = f32>) {
		for i in 0..N {
			let samp = n.next().expect("Sensor axes must be equal");

			if let Some(dec) = self.fir[i].filter(&self.cic, samp) {
				self.meas[i] = self.iir[i].filter(&self.biq, dec);
			}
		}
	}
} 

struct BiasedAxis<const N: usize> {
	align: [[f32; N]; N],
	bias: [f32; N],
	ng: Gaussian,
}

impl<const N: usize> BiasedAxis<N> {
	pub(crate) fn new(
		state: u32,
		sigma: f32,
		bias: [f32; N],
		align: [[f32; N]; N],
	) -> Self {
		Self { align, bias, ng: Gaussian::new(state, sigma) }
	}
}

impl<const N: usize> BiasedAxis<N> {
	fn scatter(&mut self, phys: &[f32; N]) -> impl Iterator<Item = f32> {
		self.align.iter()
			.zip(self.bias.iter().copied())
			.map(|(axis, bias)| {
				let mut raw = bias;

				for i in 0..N {
					raw += axis[i] * phys[i];
				}
				
				raw + <Gaussian as Distort<f32, Normal>>::distort(&mut self.ng)
			})
	}
}

pub(crate) struct Accelerometer {
	k: SensorCore<3>,
	m: BiasedAxis<3>,
	vibsens: f32
}

impl Accelerometer {
	pub(crate) fn new(
		engrate: u32,
		vibsens: f32,
		k: CoreConf,
		m: BiasConf<3>
	) -> Self {
		Self {
			k: SensorCore::new(
				k.odr,
				engrate,
				LowPass(k.cutoff, k.qbw),
				k.sens
			),
			m: BiasedAxis::new(m.state, m.sigma, m.bias, m.align),
			vibsens
		}
	}
}

impl Evaluate for Accelerometer {
	fn evaluate(&mut self, s: &ModelState) -> &Self {
		self.m.ng.sigma = s.vib * self.vibsens;
		self.k.gather(self.m.scatter(&s.acc));
		self
	}
}

pub(crate) struct Gyroscope {
	k: SensorCore<3>,
	m: BiasedAxis<3>,
	gsens: f32
}

impl Gyroscope {
	pub(crate) fn new(
		engrate: u32,
		gsens: f32,
		k: CoreConf,
		m: BiasConf<3>
	) -> Self {
		Self {
			k: SensorCore::new(
				k.odr,
				engrate,
				LowPass(k.cutoff, k.qbw),
				k.sens
			),
			m: BiasedAxis::new(m.state, m.sigma, m.bias, m.align),
			gsens
		}
	}
}

impl Evaluate for Gyroscope {
	fn evaluate(&mut self, s: &ModelState) -> &Self {
		self.k.gather(self.m.scatter(&s.ang)
			.zip(s.acc.iter().copied())
			.map(|(raw, acc)| {
				raw + acc * self.gsens
			})
		);
		self
	}
}

pub(crate) struct Magnetometer {
	k: SensorCore<3>,
	m: BiasedAxis<3>
}

impl Magnetometer {
	pub(crate) fn new(
		engrate: u32,
		k: CoreConf,
		m: BiasConf<3>
	) -> Self {
		Self {
			k: SensorCore::new(
				k.odr,
				engrate,
				LowPass(k.cutoff, k.qbw),
				k.sens
			),
			m: BiasedAxis::new(m.state, m.sigma, m.bias, m.align),
		}
	}
}

impl Evaluate for Magnetometer {
	fn evaluate(&mut self, s: &ModelState) -> &Self {
		let bodyf = s.q.rotate_w2b(EARTH_MAGFIELD);
		self.k.gather(self.m.scatter(&bodyf));
		self
	}
}

pub(crate) struct Barometer {
	k: SensorCore<1>,
	ng: Gaussian,
	bias: f32,
	lag: f32,
	tmp: f32
}

impl Barometer {
	pub(crate) fn new(
		engrate: u32,
		k: CoreConf,
		m: BiasConf<1>,
	) -> Self {
		Self {
			k: SensorCore::new(
				k.odr,
				engrate,
				LowPass(k.cutoff, k.qbw),
				k.sens
			),
			ng: Gaussian::new(m.state, m.sigma),
			bias: m.bias[0],
			lag: m.align[0][0],
			tmp: SEA_TMP
		}
	}
}

impl Evaluate for Barometer {
	fn evaluate(&mut self, s: &ModelState) -> &Self {
		let mach = s.vel.norm() / sqrt(s.tmp * SP_GAS * SP_HEAT);

		let shock = if mach > 0.8 && mach < 1.2 {
			SHOCK_DEPTH * expf(-100.0 * sq(mach - 1.0))
		} else {
			0.0
		};

		self.tmp += (s.tmp - self.tmp) * self.lag;
		let drift = (self.tmp - 20.0) * DRIFT_PER_C;

		let prs = [ s.prs + shock + drift + self.bias +
			<Gaussian as Distort<f32, Normal>>::distort(&mut self.ng) ];

		self.k.gather(prs.iter().copied());
		self
	}
}

// FIXME replace crutch with something better
pub(crate) trait Extract<T> {
	fn extract(&self) -> T;
}

macro_rules! extract3axis {
	($sensor:ident) => {
		impl Extract<XYZ> for $sensor {
			fn extract(&self) -> XYZ {
				self.k.meas
			}
		}
	};
}

extract3axis!(Accelerometer);
extract3axis!(Gyroscope);
extract3axis!(Magnetometer);

impl Extract<f32> for Barometer {
	fn extract(&self) -> f32 {
		self.k.meas[0]
	}
}