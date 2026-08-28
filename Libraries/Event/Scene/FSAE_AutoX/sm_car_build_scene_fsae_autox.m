function sm_car_build_scene_fsae_autox
%sm_car_build_scene_fsae_autox  Build and install FSAE Autocross scene
%   sm_car_build_scene_fsae_autox
%   Run this function ONCE (with the project open) to add the FSAE
%   Autocross event to the Simscape Vehicle Templates model.  It is safe
%   to run again (for example after editing the course layout).
%
%   Steps performed:
%   1. Add this folder to the project path
%   2. Generate the cone STL file and default trajectory .mat file
%   3. Build the scene library sm_car_scene_fsae_autox.slx
%   4. Add the scene as a variant choice in sm_car/World/Scene and add
%      "FSAE AutoX" to the Scene dropdown of the World block, then save
%      sm_car.slx
%
%   After running, configure the event with:
%      sm_car_config_maneuver('sm_car','FSAE AutoX')
%
% Copyright 2026 The MathWorks, Inc.

scene_dir = fileparts(which(mfilename));
cd(scene_dir)

%% 1. Ensure folder is on the project path
try
    proj = currentProject;
    pathList = {proj.ProjectPath.File};
    if(~any(strcmp(pathList,scene_dir)))
        addPath(proj,scene_dir);
        disp('Added FSAE_AutoX scene folder to project path.');
    end
catch
    warning('No project open. Adding folder to MATLAB path for this session only.');
    addpath(scene_dir);
end

%% 2. Generate supporting files
sm_car_fsae_autox_cones_stl;
sm_car_trajectory_fsae_autox;

%% 3. Build scene library
libname = 'sm_car_scene_fsae_autox';
if(bdIsLoaded(libname)), close_system(libname,0); end
if(exist(fullfile(scene_dir,[libname '.slx']),'file'))
    delete(fullfile(scene_dir,[libname '.slx']));
end
new_system(libname,'Library');

blk = [libname '/FSAE AutoX'];
add_block('built-in/Subsystem',blk,'Position',[100 100 200 142]);

% Connection port to World frame
add_block('built-in/PMIOPort',[blk '/W'],...
    'Position',[500 93 530 107],'Side','Left');

% Ground plane
add_block('sm_lib/Body Elements/Brick Solid',[blk '/Ground Plane'],...
    'Position',[100 70 140 110],...
    'BrickDimensions','[scene_data.Plane.l scene_data.Plane.w scene_data.Plane.h]',...
    'GraphicDiffuseColor','scene_data.Plane.clr',...
    'GraphicOpacity','scene_data.Plane.opc');
add_block('sm_lib/Frames and Transforms/Rigid Transform',[blk '/Transform Plane'],...
    'Position',[200 70 240 110],...
    'TranslationMethod','Cartesian',...
    'TranslationCartesianOffset',...
    '[scene_data.Plane.x scene_data.Plane.y scene_data.Plane.z-scene_data.Plane.h/2]');

% Track surface (extrusion along centerline, cross-section from mask init)
add_block('sm_lib/Body Elements/Extruded Solid',[blk '/Track'],...
    'Position',[100 170 140 210],...
    'ExtrusionCrossSection','xy_data_track',...
    'ExtrusionLength','scene_data.Track.h',...
    'GraphicDiffuseColor','scene_data.Track.clr',...
    'GraphicOpacity','scene_data.Track.opc');
add_block('sm_lib/Frames and Transforms/Rigid Transform',[blk '/Transform Track'],...
    'Position',[200 170 240 210],...
    'TranslationMethod','Cartesian',...
    'TranslationCartesianOffset','[0 0 scene_data.Track.h/2+0.001]');

% Slalom cones from STL file
add_block('sm_lib/Body Elements/File Solid',[blk '/Cones'],...
    'Position',[100 270 140 310],...
    'ExtGeomFileName','FSAE_AutoX_cones.stl',...
    'UnitType','Custom',...
    'ExtGeomFileUnits','m',...
    'BasedOnType','Density',...
    'Density','1000',...
    'GraphicDiffuseColor','scene_data.Cones.clr',...
    'GraphicOpacity','scene_data.Cones.opc');

% Observation output (unused, matches other scene variants)
add_block('built-in/Ground',[blk '/Ground'],'Position',[400 340 420 360]);
add_block('built-in/Outport',[blk '/Obs'],'Position',[480 343 510 357]);

% Wire physical connections
add_line(blk,'Transform Plane/RConn1','Ground Plane/RConn1');
add_line(blk,'W/RConn1','Transform Plane/LConn1');
add_line(blk,'Transform Track/RConn1','Track/RConn1');
add_line(blk,'W/RConn1','Transform Track/LConn1');
add_line(blk,'W/RConn1','Cones/RConn1');
add_line(blk,'Ground/1','Obs/1');

% Mask with scene data parameter
maskObj = Simulink.Mask.create(blk);
maskObj.Type = 'FSAE Autocross Scene';
maskObj.Description = 'FSAE Autocross course: track surface, slalom cones, and ground plane.';
maskObj.addParameter('Name','scene_data',...
    'Prompt','Scene data (structure)','Type','edit','Value','Scene.FSAE_AutoX');
maskObj.Initialization = ...
    'xy_data_track = sm_car_road_track_extrusion(scene_data.Track.ctrline, scene_data.Track.w);';

save_system(libname,fullfile(scene_dir,[libname '.slx']));
disp(['Created library ' libname '.slx']);

%% 4. Install scene in sm_car.slx
load_system('sm_car');
scene_sys = 'sm_car/World/Scene';
newBlk    = [scene_sys '/FSAE AutoX'];

% Remove existing copy if re-running
if(getSimulinkBlockHandle(newBlk) > 0)
    delete_block(newBlk);
end
add_block(blk,newBlk,'Position',[60 260 160 302]);
set_param(newBlk,'LinkStatus','none');            % Standalone copy
set_param(newBlk,'VariantControl','FSAE_AutoX');  % Variant label

% Add option to Scene dropdown on World block
maskW = Simulink.Mask.get('sm_car/World');
pScene = maskW.getParameter('popup_scene');
opts = pScene.TypeOptions;
if(~any(strcmp(opts,'FSAE AutoX')))
    pScene.TypeOptions = [opts; {'FSAE AutoX'}];
end

save_system('sm_car');
close_system(libname,0);
disp('FSAE AutoX scene installed in sm_car.slx.');
disp('Configure the event with:  sm_car_config_maneuver(''sm_car'',''FSAE AutoX'')');
