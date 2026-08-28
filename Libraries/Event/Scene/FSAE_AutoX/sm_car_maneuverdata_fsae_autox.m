function maneuver_data = sm_car_maneuverdata_fsae_autox
%sm_car_maneuverdata_fsae_autox  Maneuver data for FSAE Autocross event
% Copyright 2026 The MathWorks, Inc.

maneuver_type = 'FSAE_AutoX';

Instance_List = {...
    'Sedan_Hamba','Sedan_HambaLG','SUV_Landy','Bus_Makhulu','Truck_Amandla','Truck_Rhuqa','FSAE_Achilles'};

% Assign same values as defaults for all instances
for i=1:length(Instance_List)
    Instance = Instance_List{i};
    mdata.(Instance).Type                  = maneuver_type;
    mdata.(Instance).Instance              = Instance;

    mdata.(Instance).Trajectory_LoadFile.Value    = 'FSAE_AutoX_trajectory_default.mat';
    mdata.(Instance).Trajectory_LoadFile.Units    = '';
    mdata.(Instance).Trajectory_LoadFile.Comments = '';

    mdata.(Instance).xMaxLat.Value         = 5;
    mdata.(Instance).xMaxLat.Units         = 'm';
    mdata.(Instance).xMaxLat.Comments      = '';

    mdata.(Instance).vMinTarget.Value      = 5;
    mdata.(Instance).vMinTarget.Units      = 'm/s';
    mdata.(Instance).vMinTarget.Comments   = '';

    mdata.(Instance).vGain.Value           = 1;
    mdata.(Instance).vGain.Units           = '';
    mdata.(Instance).vGain.Comments        = 'Scales target speed Trajectory vx';

    mdata.(Instance).xPreview.x.Value      = [2.5 3 21];
    mdata.(Instance).xPreview.x.Units      = 'm';
    mdata.(Instance).xPreview.x.Comments   = '';

    mdata.(Instance).xPreview.v.Value      = [0 5 20];
    mdata.(Instance).xPreview.v.Units      = 'm/s';
    mdata.(Instance).xPreview.v.Comments   = '';

    mdata.(Instance).nPreviewPoints.Value      = 5;
    mdata.(Instance).nPreviewPoints.Units      = '';
    mdata.(Instance).nPreviewPoints.Comments   = 'For Pure Pursuit Driver';
end

% Unique trajectory settings (smaller vehicles)
mdata.Sedan_Hamba.xPreview.x.Value      = [2.5 3 10];
mdata.FSAE_Achilles.xPreview.x.Value    = [2.5 3 10];

% Fill in trajectory data and stop distance
for i = 1:length(fieldnames(mdata))
    Instance = Instance_List{i};
    mdata.(Instance).Trajectory = load(mdata.(Instance).Trajectory_LoadFile.Value);

    % Stop event when vehicle crosses finish line
    mdata.(Instance).xMax.Value    = ...
        floor(mdata.(Instance).Trajectory.xTrajectory.Value(end)-2);
    mdata.(Instance).xMax.Units    = 'm';
    mdata.(Instance).xMax.Comments = 'Stop test when vehicle has reached this distance';
end

maneuver_data.(maneuver_type) = mdata;
