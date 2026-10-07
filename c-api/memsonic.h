/**
 * C interface to the memsonic simulation library.
 *
 * All state lives inside a caller-allocated blob whose required size
 * and alignment are exported as compile-time constants (`MS_SIM_SIZE`,
 * `MS_SIM_ALIGN`). Ensuring them is the responsibility of the caller.
 * The simulation is not thread-safe by itself.
 */

#ifndef MEMSONIC_H
#define MEMSONIC_H

#include <stddef.h>
#include <stdint.h>

/** A safe heuristic for the amount of kimenatic targets. */
#define MS_SIM_N 256U
#define MS_ALIGN __attribute__((aligned(8)))

extern const size_t MS_SIM_SIZE;
extern const size_t MS_SIM_ALIGN;

typedef struct {
  float acc, gyr, mag, bar;
} MsODR;

typedef struct {
  float x, y, z;
} MsXYZ;

/** Helper type not defined in library source code. */
typedef struct {
  uint32_t dur;
  MsXYZ a;
  MsXYZ g;
} MsTarget;

typedef struct {
  int32_t val;
} MsReady;

static const MsReady MS_READY_NEW = {1};
static const MsReady MS_READY_STALE = {0};

typedef struct {
  int32_t val;
} MsEnv;

/** Average mid-July at Furnas Hall, University at Buffalo (43°N, 79°W). */
static const MsEnv MS_ENV_FURNAS = {0};

/** Average mid-June at IREC launch facility near Saragosa, TX (31°N, 104°W). */
static const MsEnv MS_ENV_IREC = {1};

typedef struct MsSimulation MsSim;

/**
 * @brief Creates a new simulation using the [`Manual`] builder.
 *
 * Currently, manipulating "Manual" fields through the FFI is unsupported.
 *
 * @param buf   Buffer of size >= `MS_SIM_SIZE`, aligned to `MS_SIM_ALIGN`.
 * @param odr   Pointer to ODR configuration (Hz per sensor).
 * @param env   Launch-site environment.
 * @param delay Pre-simulation idle period.
 *
 * @return Valid pointer into buf iff both buf and odr are valid.
 */
MsSim *ms_sim_new_manual(uint8_t *buf, const MsODR *odr, MsEnv env,
                         uint32_t delay);

/**
 * @brief Creates a new simulation using the [`SkewedIMU`] builder.
 *
 * @param buf   Buffer of size >= `MS_SIM_SIZE`, aligned to `MS_SIM_ALIGN`.
 * @param odr   Pointer to ODR configuration (Hz per sensor).
 * @param d	Degradation factor in [0.0; 1.0].
 * @param env   Launch-site environment.
 * @param delay Pre-simulation idle period.
 *
 * @return Valid pointer into buf iff both buf and odr are valid.
 */
MsSim *ms_sim_new_skewed(uint8_t *buf, const MsODR *odr, float d, MsEnv env,
                         uint32_t delay);

/**
 * @brief Append an absolute kinematic target to the flight profile.
 *
 * Up to `MS_SIM_N` targets may be added; additional calls are no-ops.
 *
 * @param sim	Simulation pointer..
 * @param dur	Time offset from the previous target, in ms.
 * @param a	Accel [x, y, z] (m/s^2).
 * @param g	Gyro  [x, y, z] (deg/s).
 */
void ms_sim_fix(MsSim *sim, uint32_t dur, const MsXYZ *a, const MsXYZ *g);

/**
 * @brief Append a relative kinematic target to the flight profile.
 *
 * Up to `MS_SIM_N` waypoints may be added; additional calls are no-ops.
 *
 * @param sim	Simulation pointer.
 * @param dur	Time offset from the previous target, in ms.
 * @param a	Accel [x, y, z] (m/s^2).
 * @param g	Gyro  [x, y, z] (deg/s).
 */
void ms_sim_add(MsSim *sim, uint32_t dur, const MsXYZ *a, const MsXYZ *g);

/**
 * @brief Read the accelerometer output at time `tim`.
 *
 * Lazily advances the simulation to `tim` and returns the most recent
 * sample. Always writes this sample to out.
 *
 * @param sim	Simulation pointer.
 * @param tim	Current simulation time in milliseconds.
 * @param out	Destination for the [x, y, z] reading.
 *
 * @return	`MS_READY_NEW` if fresh, `MS_READY_STALE` otherwise.
 */
MsReady ms_sim_accel(MsSim *sim, uint32_t tim, MsXYZ *out);

/**
 * @brief Read the gyroscope output at time `tim`.
 *
 * Lazily advances the simulation to `tim` and returns the most recent
 * sample. Always writes this sample to out.
 *
 * @param sim	Simulation pointer.
 * @param tim	Current simulation time in milliseconds.
 * @param out	Destination for the [x, y, z] reading.
 *
 * @return	`MS_READY_NEW` if fresh, `MS_READY_STALE` otherwise.
 */
MsReady ms_sim_angvel(MsSim *sim, uint32_t tim, MsXYZ *out);

/**
 * @brief Read the magnetometer output at time `tim`.
 *
 * Lazily advances the simulation to `tim` and returns the most recent
 * sample. Always writes this sample to out.
 *
 * @param sim	Simulation pointer.
 * @param tim	Current simulation time in milliseconds.
 * @param out	Destination for the [x, y, z] reading.
 *
 * @return	`MS_READY_NEW` if fresh, `MS_READY_STALE` otherwise.
 */
MsReady ms_sim_magfield(MsSim *sim, uint32_t tim, MsXYZ *out);

/**
 * @brief Read the barometer output at time `tim`.
 *
 * Lazily advances the simulation to `tim` and returns the most recent
 * sample. Always writes this sample to out.
 *
 * @param sim	Simulation pointer.
 * @param tim	Current simulation time in milliseconds.
 * @param out	Destination for the [pressure] reading.
 *
 * @return	`MS_READY_NEW` if fresh, `MS_READY_STALE` otherwise.
 */
MsReady ms_sim_pressure(MsSim *sim, uint32_t tim, float *out);

/**
 * @brief Drop the simulation.
 * Currently a no-op, though may be useful in future.
 */
void ms_sim_drop(MsSim *sim);

#endif /* MEMSONIC_H */
