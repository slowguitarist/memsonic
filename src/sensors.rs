//! # sensors
//!
//! A flow of data through a sensor is generalized by two operations:
//! "disperese" and "collect". The former adds noise and applies bias,
//! and the latter provides oversampling by spinning the FIR-IIR state
//! machine.

use crate::{
    Alignment, Bias, XYZ,
    env::{DRIFT_PER_C, SHOCK_DEPTH, SP_GAS, SP_HEAT, rand},
    filters::{
        Biquad, BiquadCoef,
        BiquadType::{self, LowPass},
        CICConf, EmuCIC, Filter,
    },
    math::{Vector, sq, sqrt},
    model::ModelState,
};
use libm::expf;

#[derive(Clone, Copy)]
pub(crate) struct Collector {
    pub(crate) odr: u32,
    pub(crate) cutoff: f32,
    pub(crate) qbw: f32,
    pub(crate) sens: f32,
}

#[derive(Clone, Copy)]
pub(crate) struct Disperser<const N: usize> {
    pub(crate) sigma: f32,
    pub(crate) bias: Bias<N>,
    pub(crate) align: Alignment<N>,
}

/////////////////////////////////////////////////////////////////////////////
// Gaussian-like noise generator
/////////////////////////////////////////////////////////////////////////////

pub(crate) trait NoiseMode {}

pub(crate) struct Normal;
impl NoiseMode for Normal {}

pub(crate) struct Fast;
impl NoiseMode for Fast {}

pub(crate) trait Distort<T: Copy, M: NoiseMode> {
    fn distort(&self) -> T;
}

pub(crate) struct Gaussian(f32);

impl Distort<f32, Normal> for Gaussian {
    fn distort(&self) -> f32 {
        let mut s = 0.0;

        for _ in 0..12 {
            s += (rand() as f32) / (u32::MAX as f32);
        }

        (s - 6.0) * self.0
    }
}

impl Distort<f32, Fast> for Gaussian {
    fn distort(&self) -> f32 {
        let mut s = 0;

        for _ in 0..3 {
            let r = rand();
            s += (r & 0xFF) + ((r >> 8) & 0xFF) + ((r >> 16) & 0xFF) + (r >> 24);
        }

        (s as f32 - 1530.0) / 256.0 * self.0
    }
}

/////////////////////////////////////////////////////////////////////////////
// Sensor implementations
/////////////////////////////////////////////////////////////////////////////

pub(crate) trait Evaluate {
    fn evaluate(&mut self, s: &ModelState) -> &Self;
}

struct SensorCore<const N: usize> {
    cic: CICConf,
    biq: BiquadCoef,
    fir: [EmuCIC; N],
    iir: [Biquad; N],
    meas: [f32; N],
    rel: bool,
}

impl<const N: usize> SensorCore<N> {
    pub(crate) fn new(odr: u32, engrate: u32, biqtype: BiquadType, sens: f32) -> Self {
        Self {
            cic: CICConf::new(odr / engrate, sens),
            biq: BiquadCoef::derive(biqtype, odr as f32),
            fir: [EmuCIC::new(); N],
            iir: [Biquad::new(); N],
            meas: [0.0; N],
            rel: false,
        }
    }
}

impl<const N: usize> SensorCore<N> {
    fn collect(&mut self, mut n: impl Iterator<Item = f32>) {
        for i in 0..N {
            let samp = n.next().expect("Sensor axes must be equal");

            if let Some(dec) = self.fir[i].filter(&self.cic, samp) {
                self.rel = true;
                self.meas[i] = self.iir[i].filter(&self.biq, dec);
            }
        }
    }
}

struct BiasedAxis<const N: usize> {
    align: Alignment<N>,
    bias: [f32; N],
    ng: Gaussian,
}

impl<const N: usize> BiasedAxis<N> {
    pub(crate) fn new(sigma: f32, bias: [f32; N], align: Alignment<N>) -> Self {
        Self {
            align,
            bias,
            ng: Gaussian(sigma),
        }
    }
}

impl<const N: usize> BiasedAxis<N> {
    fn disperse(&mut self, phys: &[f32; N]) -> impl Iterator<Item = f32> {
        self.align
            .iter()
            .zip(self.bias.iter().copied())
            .map(|(axis, bias)| {
                let mut raw = bias;

                for i in 0..N {
                    raw += axis[i] * phys[i];
                }

                raw + <Gaussian as Distort<f32, Normal>>::distort(&self.ng)
            })
    }
}

pub(crate) struct Accelerometer {
    k: SensorCore<3>,
    m: BiasedAxis<3>,
    vibsens: f32,
}

impl Accelerometer {
    pub(crate) fn new(engrate: u32, vibsens: f32, k: Collector, m: Disperser<3>) -> Self {
        Self {
            k: SensorCore::new(k.odr, engrate, LowPass(k.cutoff, k.qbw), k.sens),
            m: BiasedAxis::new(m.sigma, m.bias, m.align),
            vibsens,
        }
    }
}

impl Evaluate for Accelerometer {
    fn evaluate(&mut self, s: &ModelState) -> &Self {
        self.m.ng.0 = s.vib * self.vibsens;
        self.k.collect(self.m.disperse(&s.acc));
        self
    }
}

pub(crate) struct Gyroscope {
    k: SensorCore<3>,
    m: BiasedAxis<3>,
    gsens: f32,
}

impl Gyroscope {
    pub(crate) fn new(engrate: u32, gsens: f32, k: Collector, m: Disperser<3>) -> Self {
        Self {
            k: SensorCore::new(k.odr, engrate, LowPass(k.cutoff, k.qbw), k.sens),
            m: BiasedAxis::new(m.sigma, m.bias, m.align),
            gsens,
        }
    }
}

impl Evaluate for Gyroscope {
    fn evaluate(&mut self, s: &ModelState) -> &Self {
        self.k.collect(
            self.m
                .disperse(&s.ang)
                .zip(s.acc.iter().copied())
                .map(|(raw, acc)| raw + acc * self.gsens),
        );
        self
    }
}

pub(crate) struct Magnetometer {
    k: SensorCore<3>,
    m: BiasedAxis<3>,
}

impl Magnetometer {
    pub(crate) fn new(engrate: u32, k: Collector, m: Disperser<3>) -> Self {
        Self {
            k: SensorCore::new(k.odr, engrate, LowPass(k.cutoff, k.qbw), k.sens),
            m: BiasedAxis::new(m.sigma, m.bias, m.align),
        }
    }
}

impl Evaluate for Magnetometer {
    fn evaluate(&mut self, s: &ModelState) -> &Self {
        let bodyf = s.q.rotate_w2b(s.env.mag);
        self.k.collect(self.m.disperse(&bodyf));
        self
    }
}

pub(crate) struct Barometer {
    k: SensorCore<1>,
    ng: Gaussian,
    bias: f32,
    lag: f32,
    tmp: f32,
}

impl Barometer {
    pub(crate) fn new(
        sim_rate: u32,
        filters: Collector,
        noise: Disperser<1>,
        sea_tmp: f32,
    ) -> Self {
        Self {
            k: SensorCore::new(
                filters.odr,
                sim_rate,
                LowPass(filters.cutoff, filters.qbw),
                filters.sens,
            ),
            ng: Gaussian(noise.sigma),
            bias: noise.bias[0],
            lag: noise.align[0][0],
            tmp: sea_tmp,
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

        let prs = [s.prs
            + shock
            + drift
            + self.bias
            + <Gaussian as Distort<f32, Normal>>::distort(&self.ng)];

        self.k.collect(prs.iter().copied());
        self
    }
}

/////////////////////////////////////////////////////////////////////////////
// API extension
/////////////////////////////////////////////////////////////////////////////

pub(crate) trait Consume<T> {
    fn consume(&mut self) -> Result<T, T>;
}

macro_rules! extract3axis {
    ($sensor:ident) => {
        impl Consume<XYZ> for $sensor {
            fn consume(&mut self) -> Result<XYZ, XYZ> {
                if self.k.rel {
                    self.k.rel = false;
                    Ok(self.k.meas)
                } else {
                    Err(self.k.meas)
                }
            }
        }
    };
}

extract3axis!(Accelerometer);
extract3axis!(Gyroscope);
extract3axis!(Magnetometer);

impl Consume<f32> for Barometer {
    fn consume(&mut self) -> Result<f32, f32> {
        if self.k.rel {
            self.k.rel = false;
            Ok(self.k.meas[0])
        } else {
            Err(self.k.meas[0])
        }
    }
}
