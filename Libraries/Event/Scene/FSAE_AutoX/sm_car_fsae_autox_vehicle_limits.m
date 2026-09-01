function limits = sm_car_fsae_autox_vehicle_limits
%sm_car_fsae_autox_vehicle_limits  Vehicle performance envelope for trajectory
%   limits = sm_car_fsae_autox_vehicle_limits
%
%   Defines the performance envelope used by sm_car_trajectory_fsae_autox
%   to compute the target speed profile (g-g diagram).  These parameters
%   characterize the VEHICLE'S CAPABILITY only -- the driver model and
%   vehicle model are not touched.  To use a different vehicle with this
%   event, measure or estimate these values for that vehicle and re-run
%   sm_car_trajectory_fsae_autox (or sm_car_build_scene_fsae_autox).
%
%   How to characterize your vehicle:
%     mVehicle  Total mass including driver (kg)
%     muRoad    Peak tire-road friction coefficient.  Estimate from a
%               skidpad test: mu = v^2/(r*g) at the limit.
%     CLA       Downforce coefficient x frontal area (m^2), such that
%               downforce = 1/2*rho*CLA*v^2.  Set 0 if no aero package.
%     CDA       Drag coefficient x frontal area (m^2).  Set 0 to ignore.
%     PMax      Peak drivetrain power at the wheels (W).  Caps the
%               acceleration available at speed.  Estimate from an
%               acceleration run.  Set Inf to ignore.
%     vMax      Speed cap on straights (m/s)
%
% Copyright 2026 The MathWorks, Inc.

% --- Performance envelope (defaults sized for the FSAE Achilles) ---
limits.mVehicle = 200;    % kg    Total vehicle + driver mass
limits.muRoad   = 0.85;   % (1)   Peak tire-road friction coefficient
limits.CLA      = 0;      % m^2   Downforce coeff * area (0 = no aero)
limits.CDA      = 0;      % m^2   Drag coeff * area (0 = ignore drag)
limits.PMax     = 60e3;   % W     Peak power at the wheels
limits.vMax     = 22;     % m/s   Speed cap on straights
limits.rhoAir   = 1.206;  % kg/m^3 Air density

% --- Trajectory generation settings ---
limits.vStart      = 1;    % m/s  Initial speed, matches Init vehicle speed
limits.vEnd        = 2;    % m/s  Target speed at end of course
limits.useRaceline = true; % Optimize driving line within track width
limits.xMargin     = 1.2;  % m    Clearance kept to cones/track edge:
                           %      half vehicle track width plus expected
                           %      path-tracking error of the driver
end
