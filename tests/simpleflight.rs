use memsonic::{
    Simulation,
    builder::{SimBuilder, SkewedIMU},
    env::BuffaloJuly,
};

#[test]
fn simple_flight() {
    let b = SkewedIMU::new((10, 10, 30, 40), 0.5);
    let mut s = Simulation::<8>::new::<BuffaloJuly>(b, 1000);

    // 1. Boost
    s.fix(1800, [0.8, -0.4, 78.5], [1.5, -2.0, 210.0])
        // 2. Coast
        .fix(6200, [0.2, -0.1, -14.2], [0.4, 0.6, 95.0])
        // 3. Apogee
        .fix(16_000, [3.5, -2.8, -1.2], [32.0, -22.0, 15.0])
        // 4. Descent
        .fix(31_000, [1.5, -1.2, 10.8], [14.0, -11.0, 6.0])
        // 5. Main event
        .fix(30_000, [-18.5, 24.0, 175.0], [-145.0, 92.0, 260.0])
        // 6. Chute
        .fix(7000, [0.3, -0.2, 9.81], [2.5, -1.8, 4.0])
        // Landing
        .fix(42_000, [9.81, 0.0, 0.0], [0.0, 0.0, 0.0]);

    let step = 10u32;
    let mut ts = 0u32;

    while ts < 144_000 {
        if let Ok(a) = s.accel(ts) {
            println!("ACCL {} | {:?}", ts, a);
        }

        if let Ok(a) = s.angvel(ts) {
            println!("GYRO {} | {:?}", ts, a);
        }

        if let Ok(a) = s.magfield(ts) {
            println!("MAGA {} | {:?}", ts, a);
        }

        if let Ok(a) = s.pressure(ts) {
            println!("BARO {} | {:?}", ts, a);
        }

        ts += step;
    }
}
