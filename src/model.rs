use libm::powf;

use crate::{
    Motion, XYZ,
    builder::SyntheticSensor,
    common::{G_SI_NED, LAPSE_RT, MAX_SUBNATICA, R_EXP, SEA_PRS, SEA_TMP},
    math::{Quaternion, Vector, sq},
    sensors::{Accelerometer, Barometer, Evaluate, Gyroscope, Magnetometer},
};

pub(crate) struct ModelState {
    pub(crate) acc: XYZ,
    pub(crate) ang: XYZ,
    pub(crate) vel: XYZ,
    pub(crate) pos: XYZ,
    pub(crate) prs: f32,
    pub(crate) tmp: f32,
    pub(crate) vib: f32,
    pub(crate) q: Quaternion,
}

impl ModelState {
    fn new() -> Self {
        Self {
            acc: XYZ::default(),
            ang: XYZ::default(),
            vel: XYZ::default(),
            pos: XYZ::default(),
            prs: SEA_PRS,
            tmp: SEA_TMP,
            vib: 0.0,
            q: Quaternion::new(),
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
    pub(crate) fn new(rate: u32, imu: [SyntheticSensor<3>; 3], bar: SyntheticSensor<1>) -> Self {
        Self {
            s: ModelState::new(),
            acl: Accelerometer::new(rate, imu[0].s.unwrap_or(0.0005), imu[0].k, imu[0].m),
            gyr: Gyroscope::new(rate, imu[1].s.unwrap_or(0.00001), imu[1].k, imu[1].m),
            mag: Magnetometer::new(rate, imu[2].k, imu[2].m),
            bar: Barometer::new(rate, bar.k, bar.m),
        }
    }

    pub(crate) fn derive(&mut self, dt: f32, f: Motion) -> &Self {
        self.s.acc = f.0;
        self.s.ang = f.1;

        let mut kin = self.s.q.integrate(self.s.ang, dt).rotate_b2w(self.s.acc);

        kin[2] += G_SI_NED;

        for (i, val) in kin.iter().enumerate() {
            self.s.vel[i] += val * dt;
            self.s.pos[i] += self.s.vel[i] * dt;
        }

        let alt = if self.s.pos[2] > MAX_SUBNATICA {
            -MAX_SUBNATICA
        } else {
            -self.s.pos[2]
        };

        self.s.tmp = SEA_TMP - LAPSE_RT * alt;
        self.s.prs = SEA_PRS * powf(self.s.tmp / SEA_TMP, R_EXP);

        let v_sq = sq(self.s.vel[0]) + sq(self.s.vel[1]) + sq(self.s.vel[2]);
        self.s.vib = 1.0 + 0.005 * self.s.acc.norm() + 0.0001 * v_sq;

        self.acl.evaluate(&self.s);
        self.gyr.evaluate(&self.s);
        self.mag.evaluate(&self.s);
        self.bar.evaluate(&self.s);

        self
    }
}
