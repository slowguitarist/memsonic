//! # ffi
//!
//! C-ABI adaptor. Exposes a flat, `no_std`-compatible interface for
//! current public API.
//!
//! ## Memory model
//!
//! All state lives inside a caller-allocated blob whose required size
//! and alignment are exported as compile-time constants (`MS_SIM_SIZE`,
//! `MS_SIM_ALIGN`). Ensuring them is the responsibility of the caller.
//! The simulation is not thread-safe by itself.
//!
//! ```c
//! static MS_SIM_ALIGN_ATTR uint8_t sim_buf[MS_SIM_SIZE];
//! MsODR odr = {10.0f, 10.0f, 30.0f, 40.0f};
//! MsSimulation *sim = ms_sim_new_skewed(
//!     sim_buf, &odr, 0.5f, MS_ENV_FURNAS, 1000);
//! ```

use core::{mem, ptr};

use crate::{
    Simulation,
    builder::{Manual, SimBuilder, SkewedIMU},
    env::{Furnas, IREC, Setup, Surface},
};

// Only compiled when memsonic is compiled as standalone.
#[cfg(all(not(feature = "desktop"), not(test)))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// A safe heuristic for the amount of kimenatic targets.
pub const FFI_N: usize = 256;

type Sim = Simulation<FFI_N>;

/////////////////////////////////////////////////////////////////////////////
// C API -- Types
/////////////////////////////////////////////////////////////////////////////

#[unsafe(no_mangle)]
pub static MS_SIM_SIZE: usize = mem::size_of::<Sim>();

#[unsafe(no_mangle)]
pub static MS_SIM_ALIGN: usize = mem::align_of::<Sim>();

/// Output data rates for accelerometer, gyroscope, magnetometer, and
/// barometer (Hz), in that order. Maps to [`crate::ODR`].
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MsODR {
    pub acc: f32,
    pub gyr: f32,
    pub mag: f32,
    pub bar: f32,
}

impl From<MsODR> for crate::ODR {
    fn from(o: MsODR) -> Self {
        (o.acc, o.gyr, o.mag, o.bar)
    }
}

/// A 3-element "3-axis" vector. Maps to [`crate::XYZ`].
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct MsXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<MsXYZ> for crate::XYZ {
    fn from(v: MsXYZ) -> Self {
        [v.x, v.y, v.z]
    }
}

impl From<crate::XYZ> for MsXYZ {
    fn from([x, y, z]: crate::XYZ) -> Self {
        Self { x, y, z }
    }
}

/// Sensor reading result tag. Maps to normally returned Result<T>.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct MsReady(pub i32);

pub const MS_READY_NEW: MsReady = MsReady(1);
pub const MS_READY_STALE: MsReady = MsReady(0);

/// Launch-site environment selector passed to `ms_sim_new_*`.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct MsEnv(pub i32);

pub const MS_ENV_FURNAS: MsEnv = MsEnv(0);
pub const MS_ENV_IREC: MsEnv = MsEnv(1);

/////////////////////////////////////////////////////////////////////////////
// FFI helpers
/////////////////////////////////////////////////////////////////////////////

fn env_surface(e: MsEnv) -> Surface {
    match e {
        MS_ENV_IREC => IREC::setup(),
        _ => Furnas::setup(),
    }
}

/// Caller guarantees alignment and size.
unsafe fn place(buf: *mut u8, sim: Sim) -> *mut Sim {
    let p = buf as *mut Sim;
    unsafe { ptr::write(p, sim) };
    p
}

/////////////////////////////////////////////////////////////////////////////
// C API -- Functions
/////////////////////////////////////////////////////////////////////////////

/// Creates a new simulation using the [`Manual`] builder.
///
/// Returns a valid pointer into `buf` iff both buf and odr are valid.
///
/// TODO: Allow manipulating [`Manual`] from C.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_new_manual(
    buf: *mut u8,
    odr: *const MsODR,
    env: MsEnv,
    delay: u32,
) -> *mut Sim {
    if buf.is_null() || odr.is_null() {
        return ptr::null_mut();
    }

    let odr_val: crate::ODR = unsafe { (*odr).into() };
    let b = Manual::new(odr_val, ());
    let sim = Sim::new_with_surface(b, env_surface(env), delay);

    unsafe { place(buf, sim) }
}

/// Creates a new simulation using the [`SkewedIMU`] builder.
///
/// Returns a valid pointer into `buf` iff both buf and odr are valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_new_skewed(
    buf: *mut u8,
    odr: *const MsODR,
    d: f32,
    env: MsEnv,
    delay: u32,
) -> *mut Sim {
    if buf.is_null() || odr.is_null() {
        return ptr::null_mut();
    }

    let odr_val: crate::ODR = unsafe { (*odr).into() };
    let b = SkewedIMU::new(odr_val, d);
    let sim = Sim::new_with_surface(b, env_surface(env), delay);

    unsafe { place(buf, sim) }
}

/// Appends an absolute kinematic waypoint at a relative time offset.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_fix(sim: *mut Sim, dur: u32, a: *const MsXYZ, g: *const MsXYZ) {
    if sim.is_null() || a.is_null() || g.is_null() {
        return;
    }

    let (a_val, g_val) = unsafe { ((*a).into(), (*g).into()) };
    unsafe { (*sim).fix(dur, a_val, g_val) };
}

/// Appends a relative kinematic waypoint at a relative time offset.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_add(sim: *mut Sim, dur: u32, a: *const MsXYZ, g: *const MsXYZ) {
    if sim.is_null() || a.is_null() || g.is_null() {
        return;
    }

    let (a_val, g_val) = unsafe { ((*a).into(), (*g).into()) };
    unsafe { (*sim).add(dur, a_val, g_val) };
}

/// Reads the accelerometer at simulation time `tim` (ms).
///
/// Writes the `[x, y, z]` result into `*out`.
/// Returns `MS_READY_NEW` (1) if the reading is fresh, `MS_READY_STALE` (0)
/// if it has not changed since the last call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_accel(sim: *mut Sim, tim: u32, out: *mut MsXYZ) -> MsReady {
    if sim.is_null() || out.is_null() {
        return MS_READY_STALE;
    }

    let result = unsafe { (*sim).accel(tim) };
    let (tag, val) = match result {
        Ok(v) => (MS_READY_NEW, v),
        Err(v) => (MS_READY_STALE, v),
    };

    unsafe { ptr::write(out, val.into()) };
    tag
}

/// Reads the gyroscope at simulation time `tim` (ms).
///
/// Writes the `[x, y, z]` result into `*out`.
/// Returns `MS_READY_NEW` (1) if the reading is fresh, `MS_READY_STALE` (0)
/// if it has not changed since the last call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_angvel(sim: *mut Sim, tim: u32, out: *mut MsXYZ) -> MsReady {
    if sim.is_null() || out.is_null() {
        return MS_READY_STALE;
    }

    let result = unsafe { (*sim).angvel(tim) };
    let (tag, val) = match result {
        Ok(v) => (MS_READY_NEW, v),
        Err(v) => (MS_READY_STALE, v),
    };

    unsafe { ptr::write(out, val.into()) };
    tag
}

/// Reads the magnetometer at simulation time `tim` (ms).
///
/// Writes the `[x, y, z]` result into `*out`.
/// Returns `MS_READY_NEW` (1) if the reading is fresh, `MS_READY_STALE` (0)
/// if it has not changed since the last call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_magfield(sim: *mut Sim, tim: u32, out: *mut MsXYZ) -> MsReady {
    if sim.is_null() || out.is_null() {
        return MS_READY_STALE;
    }

    let result = unsafe { (*sim).magfield(tim) };
    let (tag, val) = match result {
        Ok(v) => (MS_READY_NEW, v),
        Err(v) => (MS_READY_STALE, v),
    };

    unsafe { ptr::write(out, val.into()) };
    tag
}

/// Reads the barometer at simulation time `tim` (ms).
///
/// Writes the pressure result (Pa) into `*out`.
/// Returns `MS_READY_NEW` (1) if the reading is fresh, `MS_READY_STALE` (0)
/// if it has not changed since the last call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_pressure(sim: *mut Sim, tim: u32, out: *mut f32) -> MsReady {
    if sim.is_null() || out.is_null() {
        return MS_READY_STALE;
    }

    let result = unsafe { (*sim).pressure(tim) };
    let (tag, val) = match result {
        Ok(v) => (MS_READY_NEW, v),
        Err(v) => (MS_READY_STALE, v),
    };

    unsafe { ptr::write(out, val) };
    tag
}

/// Drops the simulation. Currently a no-op, may be useful in future.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ms_sim_drop(sim: *mut Sim) {
    if !sim.is_null() {
        unsafe { ptr::drop_in_place(sim) };
    }
}
