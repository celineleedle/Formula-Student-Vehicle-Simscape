function scene_data = sm_car_scenedata_fsae_autox
%% FSAE Autocross scene parameters
%  Course geometry is defined in sm_car_fsae_autox_define_course.m
% Copyright 2026 The MathWorks, Inc.

scene_data.Name = 'FSAE_AutoX';

% Course geometry (centerline, driving line, cones, width)
course = sm_car_fsae_autox_define_course;

scene_data.Track.ctrline = course.ctrline;  % [x y z] (m)
scene_data.Track.refpath = course.refpath;  % [x y z] (m)
scene_data.Track.w       = course.w;        % m
scene_data.Track.h       = 0.01;            % m
scene_data.Track.clr     = [1 1 1]*0.5;     % [R G B]
scene_data.Track.opc     = 1;               % (0-1)

% Slalom cones (visual only)
scene_data.Cones.pos      = course.cones;   % [x y z] (m)
scene_data.Cones.r_base   = 0.14;           % m
scene_data.Cones.r_top    = 0.02;           % m
scene_data.Cones.height   = 0.30;           % m
scene_data.Cones.clr      = [0.90 0.38 0.0];% [R G B]
scene_data.Cones.opc      = 1;              % (0-1)
scene_data.Cones.stl_file = 'FSAE_AutoX_cones.stl';

% Ground plane sized to course extents plus margin
margin = 20; % m
xmin = min(course.ctrline(:,1)); xmax = max(course.ctrline(:,1));
ymin = min(course.ctrline(:,2)); ymax = max(course.ctrline(:,2));

scene_data.Plane.l   = (xmax-xmin)+2*margin;    % m
scene_data.Plane.w   = (ymax-ymin)+2*margin;    % m
scene_data.Plane.h   = 0.01;                    % m
scene_data.Plane.x   = (xmax+xmin)/2;           % m
scene_data.Plane.y   = (ymax+ymin)/2;           % m
scene_data.Plane.z   = 0;                       % m
scene_data.Plane.clr = [0.4660 0.6740 0.1880]*0.7; % [R G B]
scene_data.Plane.opc = 1;                       % (0-1)
