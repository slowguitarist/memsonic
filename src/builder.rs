use libm::powf;

use crate::{Alignment, Bias, ODR, config::{Collector, Disperser, SyntheticSensor}, math::{SquareMatrix, ceil, leash}};
use crate::math::RandomArray;

pub trait SimBuilder {
	type InputParams;

	fn new(r: ODR, p: Self::InputParams) -> Self;
	fn imu(&mut self) -> [SyntheticSensor<3>; 3];
	fn baro(&mut self) -> SyntheticSensor<1>;
	fn rate(&mut self) -> u32;
}

#[derive(Clone, Copy)]
pub struct SensorConf<const N: usize> {
	pub odr: u32,
	pub cutoff: f32,
	pub qbw: f32,
	pub firsens: Option<f32>,
	pub sigma: f32,
	pub bias: Bias<N>,
	pub align: Alignment<N>,
	pub sens: Option<f32>
}

impl<const N: usize> SensorConf<N> {
	fn new(odr: u32) -> Self {
		Self {
			odr,
			cutoff: 100.0,
			qbw: 0.7071,
			firsens: None,
			sigma: 0.005,
			bias: [0.0; N],
			align: Alignment::identity(),
			sens: None
		}
	}

	fn wrap(&self) -> SyntheticSensor<N> {
		SyntheticSensor {
			k: Collector {
				odr: self.odr,
				cutoff: self.cutoff,
				qbw: self.qbw,
				sens: self.firsens.unwrap_or(self.odr as f32 * 10_000.0)
			},
			m: Disperser {
				sigma: self.sigma,
				bias: self.bias,
				align: self.align
			},
			s: self.sens
		}
	}

	fn linear_tune(&mut self, deg: f32) {
		self.cutoff -= 80.0 * deg;
		self.qbw -= 0.2071 * deg;
		self.firsens = Some(100_000.0 * (1.0 - deg));
		self.sigma *= powf(10.0, 2.0 * deg);
		self.bias = self.bias.randomize(deg);
		self.align = self.align.randomize(deg);
	}
}

pub struct Manual {
	pub acc: SensorConf<3>,
	pub gyr: SensorConf<3>,
	pub mag: SensorConf<3>,
	pub bar: SensorConf<1>,
}

impl SimBuilder for Manual {
	type InputParams = ();

	fn new(r: ODR, _p: Self::InputParams) -> Self {
		Self {
			acc: SensorConf::new(r.0),
			gyr: SensorConf::new(r.1),
			mag: SensorConf::new(r.2),
			bar: SensorConf::new(r.3)
		}
	}

	fn rate(&mut self) -> u32 {
		let m = self.acc.odr
			.min(self.gyr.odr)
			.min(self.mag.odr)
			.min(self.bar.odr);

		let mut eng = m / 5;

		if eng == 0 {
			eng = 1;
			let k = 5.0 / (m as f32);

			self.acc.odr = ceil(self.acc.odr as f32 * k) as u32;
			self.gyr.odr = ceil(self.gyr.odr as f32 * k) as u32;
			self.mag.odr = ceil(self.mag.odr as f32 * k) as u32;
			self.bar.odr = ceil(self.bar.odr as f32 * k) as u32;
		}

		eng
	}

	fn imu(&mut self) -> [SyntheticSensor<3>; 3] {
		[
			self.acc.wrap(),
			self.gyr.wrap(),
			self.mag.wrap()
		]
	}

	fn baro(&mut self) -> SyntheticSensor<1> {
		self.bar.wrap()
	}
}

pub struct SkewedIMU {
	m: Manual,
	deg: f32
}

impl SimBuilder for SkewedIMU {
	type InputParams = f32;

	fn new(r: ODR, p: Self::InputParams) -> Self {
		Self {
			m: Manual::new(r, ()),
			deg: p.clamp(0.0, 1.0)
		}
	}

	fn rate(&mut self) -> u32 {
		self.m.rate()
	}

	fn imu(&mut self) -> [SyntheticSensor<3>; 3] {
		self.m.acc.linear_tune(leash(self.deg, 0.05));
		self.m.gyr.linear_tune(leash(self.deg, 0.05));
		self.m.mag.linear_tune(leash(self.deg, 0.05));

		self.m.acc.sens = Some(self.deg / 5.0 + 0.0005);
		self.m.gyr.sens = Some(self.deg / 200.0 + 0.00001);

		self.m.imu()
	}

	fn baro(&mut self) -> SyntheticSensor<1> {
		self.m.bar.linear_tune(self.deg / 10.0);
		self.m.baro()
	}
}