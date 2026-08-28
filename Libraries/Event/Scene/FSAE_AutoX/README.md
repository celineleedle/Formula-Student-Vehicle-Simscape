# FSAE Autocross Event

This folder adds an FSAE Autocross event to the Formula Student Simscape
Vehicle Templates, following the same pattern as the Skidpad event.

Two course layouts are available in `sm_car_fsae_autox_define_course.m`:

* **`FSAE_2024`** (default) — the official Formula SAE Michigan 2024
  Skidpad/Autocross course, digitized from the competition course map on
  fsaeonline.com. The driving line was traced from the map and scaled
  using its 25-ft grid (~835 m). The digitized line is the driving line,
  so in slalom sections it already weaves; cones mark both track edges.
  Note the digitization is faithful to the map's resolution (~0.5 m per
  pixel) — the overall geometry is accurate, but exact cone counts and
  placements within slaloms are approximations.
* **`Custom`** — a parameterized segment list (straights, arcs, slaloms)
  that follows the FSAE rules for autocross courses (Rules D8.6.1):

| Feature        | Rule                          | Custom layout                   |
|----------------|-------------------------------|---------------------------------|
| Straights      | ≤ 60 m                        | 40 m start, 30/35 m others      |
| Constant turns | 23–45 m diameter              | 30, 40, 24, 32 m diameter turns |
| Hairpins       | ≥ 9 m outside diameter        | 10 m centerline diameter        |
| Slaloms        | cones 7.62–12.19 m apart      | 5 cones @ 10 m, 4 cones @ 9 m   |
| Track width    | ≥ 3.5 m                       | 4.5 m                           |
| Length         | < 0.805 km                    | ~0.42 km                        |

To switch layouts, change the default in
`sm_car_fsae_autox_define_course.m` (or edit the calls in the scene data
and trajectory scripts), then re-run `sm_car_build_scene_fsae_autox`.

## Running the event

With the project open:

```matlab
sm_car_config_maneuver('sm_car','FSAE AutoX')
```

then simulate `sm_car`. The event stops automatically when the vehicle
crosses the finish line (`Maneuver.xMax`).

## Files

| File | Purpose |
|------|---------|
| `sm_car_fsae_autox_define_course.m` | **Course layout** — layout selector and Custom segment list |
| `FSAE_AutoX_2024_ctrline.mat` | Digitized 2024 Michigan course centerline |
| `sm_car_scenedata_fsae_autox.m` | Scene parameters (track, cones, ground plane) |
| `sm_car_trajectory_fsae_autox.m` | Builds the driver trajectory (curvature-based speed profile) |
| `sm_car_maneuverdata_fsae_autox.m` | Maneuver parameters per vehicle instance |
| `sm_car_fsae_autox_cones_stl.m` | Writes `FSAE_AutoX_cones.stl` for the scene |
| `sm_car_build_scene_fsae_autox.m` | One-time build/install script (see below) |
| `FSAE_AutoX_trajectory_default.mat` | Generated trajectory data |
| `FSAE_AutoX_cones.stl` | Generated cone geometry |
| `sm_car_scene_fsae_autox.slx` | Generated scene library |

Related files elsewhere in the project:

* `Libraries/Event/Init_data_fsae_autox.m` — initial vehicle state
* `Libraries/Event/Maneuver_data_fsae_autox.m` — loads Maneuver into workspace
* `Scripts_Data/Configure_Event/sm_car_config_maneuver.m` — `'fsae autox'` case
* `Libraries/Event/sm_car_gen_driver_database.m` — `FSAE_AutoX` driver entry

## Modifying the course

1. Edit the segment list in `sm_car_fsae_autox_define_course.m`
   (straights, arcs, slaloms — see the header comments).
2. Re-run the build script to regenerate the cones, trajectory, and scene:

   ```matlab
   sm_car_build_scene_fsae_autox
   ```

The build script is idempotent — it replaces the existing scene in
`sm_car.slx` and regenerates all derived files.

## Speed profile

The target speed along the driving line is computed in
`sm_car_trajectory_fsae_autox.m` from path curvature with a lateral
acceleration limit plus forward/backward passes with longitudinal
acceleration limits. Tune `v_max`, `gy_max`, `gx_accel`, and `gx_decel`
there to match your vehicle's capability.

Note: in the slalom sections the visual track centerline runs straight
through the cones while the driver trajectory weaves around them.
