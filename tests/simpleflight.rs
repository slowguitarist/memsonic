use memsonic::{
    Simulation,
    builder::{SimBuilder, SkewedIMU},
    env::BuffaloJuly,
};

#[test]
fn simple_flight() {
    let b = SkewedIMU::new((10, 10, 30, 40), 0.5);
    let mut simu = Simulation::<17>::new::<BuffaloJuly>(b, 1000);

    // 1. Ignition & Launch Rail Exit (Ramp-up)
    simu.fix(200, [0.3, -0.1, 28.5], [0.5, -0.8, 45.0])
        // 2. Peak Motor Thrust (Sustained ~8g Burn)
        .fix(1600, [0.8, -0.4, 78.5], [1.5, -2.0, 210.0])
        // 3. Motor Burnout & Thrust Tail-off
        .fix(2200, [0.4, -0.2, 12.0], [0.8, -1.0, 160.0])
        // 4. Max Dynamic Pressure (Peak Aero Drag Post-Burnout)
        .fix(800, [0.2, -0.1, -25.4], [0.4, 0.6, 120.0])
        // 5. Unpowered Ascent (Decelerating Coast)
        .fix(7200, [0.1, -0.1, -12.1], [0.3, 0.4, 65.0])
        // 6. Cresting Apogee (Velocity Approaches Zero)
        .fix(12000, [0.0, 0.0, -0.8], [2.1, -1.5, 20.0])
        // 7. Drogue Separation Charge (Transient Pressure Impulse)
        .fix(200, [12.5, -8.2, 42.0], [75.0, -52.0, 35.0])
        // 8. Drogue Line Stretch & Inflation Shock
        .fix(500, [-4.2, 5.1, 28.0], [-38.0, 26.0, 18.0])
        // 9. Steady Drogue Descent (~20 m/s Terminal Velocity)
        .fix(30_500, [1.5, -1.2, 10.8], [14.0, -11.0, 6.0])
        // 10. Main Canopy Separation Charge (1,000 ft AGL)
        .fix(200, [-8.5, 12.0, 35.0], [-60.0, 40.0, 110.0])
        // 11. Main Snatch Shock Peak (18g Mechanical Line Stretch)
        .fix(400, [-18.5, 24.0, 175.0], [-145.0, 92.0, 260.0])
        // 12. Canopy Inflation Transient (Rapid Deceleration to 6 m/s)
        .fix(1100, [-5.2, 6.8, 32.0], [-45.0, 28.0, 85.0])
        // 13. Inflation Complete & Oscillation Damping
        .fix(3500, [0.3, -0.2, 11.2], [5.0, -3.2, 12.0])
        // 14. Steady Main Canopy Descent (~6 m/s Terminal Velocity)
        .fix(49_800, [0.1, -0.1, 9.81], [1.5, -1.0, 2.0])
        // 15. Ground Touchdown (Impact Shock Spike)
        .fix(200, [48.2, -22.0, -18.5], [180.0, -95.0, 45.0])
        // 16. Rest (Airframe Stationary Flat on Ground)
        .fix(2000, [9.81, 0.0, 0.0], [0.0, 0.0, 0.0]);

    let step = 10u32;
    let mut ts = 0u32;

    while ts < 113_500 {
        if let Ok(a) = simu.accel(ts) {
            println!("ACC {} {:?}", ts, a);
        }

        if let Ok(a) = simu.angvel(ts) {
            println!("GYR {} {:?}", ts, a);
        }

        if let Ok(a) = simu.magfield(ts) {
            println!("MAG {} {:?}", ts, a);
        }

        if let Ok(a) = simu.pressure(ts) {
            println!("BAR {} {:?}", ts, a);
        }

        ts += step;
    }
}
