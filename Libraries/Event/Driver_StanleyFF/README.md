# Stanley FF Lateral Driver Variant

Adds a `Stanley_FF` choice to the closed-loop Lateral Driver variant
subsystem in `sm_car.slx`: Stanley feedback plus explicit curvature
feedforward. Built for the FSAE Autocross event, where the raceline +
g-g trajectory is faster than the reactive Stanley controller can track
cleanly, but usable by any vehicle/maneuver.

## Why

The stock Stanley controller is purely reactive — it steers only in
response to heading and cross-track *error*, so it turns in late at
speed. The template compensates by handing Stanley a reference pose
interpolated a preview distance (up to ~20 m) ahead of the car
(`getPoseCurRef`), which restores stability but introduces a systematic
offset toward the inside of corners: the heading error toward the far
preview point is roughly `curvature x preview distance`, far more steer
than the corner needs, and equilibrium is only reached once the
cross-track error grows enough to cancel the excess.

This variant splits the steering job into four terms, each with one job
and one physically meaningful parameter:

| Term | Formula | Role | Parameter |
|------|---------|------|-----------|
| Heading feedback | `psi_e` at `v*T_REF` ahead | align with path; short speed-scaled preview supplies loop damping | `StanleyFF.TRef` |
| Cross-track feedback | `atan(K*e/v)` | pull back to the line | `Stanley.NForward` |
| Kinematic feedforward | `atan(L*kappa)` | steer for the corner geometry before an error develops | `StanleyFF.KFF` |
| Dynamic feedforward | `K_US*v^2*kappa` | extra steer for tire slip at lateral g (understeer gradient) | `StanleyFF.KUnder` |

Curvature `kappa` is estimated online from the trajectory preview
points (yaw vs. distance) that the Maneuver block already publishes,
evaluated `v*T_FF` ahead (`StanleyFF.TFF`). No interface changes, no
vehicle-model coupling: a different vehicle needs only its wheelbase
(already in the driver database) and these scalars.

## Files

| File | Purpose |
|------|---------|
| `lateralDrivingControllerFF.m` | Controller source (copied into the MATLAB Function block by the build script) |
| `sm_car_build_driver_stanley_ff.m` | One-time build/install script (idempotent) |

## Installing

With the project open:

```matlab
sm_car_build_driver_stanley_ff
```

The script copies the existing `Stanley` variant subsystem to
`Stanley FF`, replaces its controller code, wires in the preview bus and
the `K_FF` / `T_FF` / `K_US` / `T_REF` parameters, and saves
`sm_car.slx`. Re-run it after editing `lateralDrivingControllerFF.m`.

## Activating

Set the lateral driver class for a vehicle or maneuver in
`Libraries/Event/sm_car_gen_driver_database.m`:

```matlab
Driver.Lateral.class.Value = 'Stanley_FF'
```

The FSAE Autocross event activates it for the FSAE Achilles via
`DDatabase.FSAE_AutoX.FSAE_Achilles.Lateral.class.Value`; all other
events keep the standard Stanley driver.

Parameters live in the same file
(`Driver.<vehicle>.Lateral.StanleyFF.*`): `KFF` (feedforward gain,
1 = geometric), `TFF` (feedforward preview time, s), `KUnder`
(understeer gradient, rad per m/s^2 — measure from a skidpad test as
the slope of steering angle vs. lateral acceleration), `TRef`
(reference preview time, s — loop damping).

## Validation (FSAE Autocross, FSAE Achilles)

| Configuration | Course time | Max deviation | Cone contacts |
|---------------|-------------|---------------|---------------|
| Stanley, centerline trajectory (baseline) | 76.6 s | 0.77 m | 0 |
| Stanley, raceline + g-g (mu=0.85) | 50.9 s | 2.06 m | 9 |
| **Stanley FF, raceline + g-g (mu=0.80)** | **55.6 s** | **1.43 m** | **1 (marginal, 0.44 m CG clearance)** |

The remaining tracking error is dominated by transient overspeed at
corner entry (the longitudinal driver's braking response), not by the
lateral controller — see `tauBrake` in
`sm_car_fsae_autox_vehicle_limits.m`, which pre-compensates part of it
in the target speed profile.
