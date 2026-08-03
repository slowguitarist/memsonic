//! # model
//!
//! A small self-propagating physics model whose only purpose
//! is to serve as a reference point for sensor evaluations.

use crate::{
    XYZ,
    builder::SyntheticSensor,
    env::{Conditions, LAPSE_RT, MAX_SUBNATICA, Surface},
    math::{Quaternion, Vector, sq},
    profile::Motion,
    sensors::{Accelerometer, Barometer, Evaluate, Gyroscope, Magnetometer},
};
use libm::powf;

pub(crate) struct ModelState {
    pub(crate) acc: XYZ,
    pub(crate) ang: XYZ,
    pub(crate) vel: XYZ,
    pub(crate) pos: XYZ,
    pub(crate) prs: f32,
    pub(crate) tmp: f32,
    pub(crate) vib: f32,
    pub(crate) q: Quaternion,
    pub(crate) env: Conditions,
}

impl ModelState {
    fn new(mag: XYZ, sea_tmp: f32, sea_prs: f32) -> Self {
        Self {
            acc: XYZ::default(),
            ang: XYZ::default(),
            vel: XYZ::default(),
            pos: XYZ::default(),
            prs: sea_prs,
            tmp: sea_tmp,
            vib: 0.0,
            q: Quaternion::new(),
            env: Conditions::new(mag, sea_prs, sea_tmp),
        }
    }
}

pub(crate) struct Model {
    pub(crate) s: ModelState,
    pub(crate) acl: Accelerometer,
    pub(crate) gyr: Gyroscope,
    pub(crate) mag: Magnetometer,
    pub(crate) bar: Barometer,
}

impl Model {
    pub(crate) fn new(
        rate: u32,
        imu: [SyntheticSensor<3>; 3],
        bar: SyntheticSensor<1>,
        site: Surface,
    ) -> Self {
        Self {
            s: ModelState::new(site.mag, site.tmp, site.prs),
            acl: Accelerometer::new(rate, imu[0].s.unwrap_or(0.0005), imu[0].k, imu[0].m),
            gyr: Gyroscope::new(rate, imu[1].s.unwrap_or(0.00001), imu[1].k, imu[1].m),
            mag: Magnetometer::new(rate, imu[2].k, imu[2].m),
            bar: Barometer::new(rate, bar.k, bar.m, site.tmp),
        }
    }

    pub(crate) fn derive(&mut self, dt: f32, f: Motion) -> &Self {
        self.s.acc = f.0;
        self.s.ang = f.1;

        let mut kin = self.s.q.integrate(self.s.ang, dt).rotate_b2w(self.s.acc);

        kin[2] += self.s.env.g_si_ned;

        for (i, val) in kin.iter().enumerate() {
            self.s.vel[i] += val * dt;
            self.s.pos[i] += self.s.vel[i] * dt;
        }

        let alt = if self.s.pos[2] > MAX_SUBNATICA {
            -MAX_SUBNATICA
        } else {
            -self.s.pos[2]
        };

        self.s.tmp = self.s.env.sea_tmp - LAPSE_RT * alt;
        self.s.prs = self.s.env.sea_prs * powf(self.s.tmp / self.s.env.sea_tmp, self.s.env.r_exp);

        let v_sq = sq(self.s.vel[0]) + sq(self.s.vel[1]) + sq(self.s.vel[2]);
        self.s.vib = 1.0 + 0.005 * self.s.acc.norm() + 0.0001 * v_sq;

        self.acl.evaluate(&self.s);
        self.gyr.evaluate(&self.s);
        self.mag.evaluate(&self.s);
        self.bar.evaluate(&self.s);

        self
    }
}
