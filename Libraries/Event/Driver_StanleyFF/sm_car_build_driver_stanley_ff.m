function sm_car_build_driver_stanley_ff
%sm_car_build_driver_stanley_ff  Install Stanley FF lateral driver variant
%   sm_car_build_driver_stanley_ff
%   Run this function ONCE (with the project open) to add the
%   'Stanley_FF' lateral driver variant (Stanley + curvature feedforward)
%   to the closed-loop driver in sm_car.slx.  Safe to run again, for
%   example after editing lateralDrivingControllerFF.m.
%
%   Steps performed:
%   1. Copy the existing 'Stanley' variant subsystem to 'Stanley FF'
%      inside the Lateral Driver variant subsystem
%   2. Replace its controller code with lateralDrivingControllerFF.m
%      (Stanley feedback + curvature feedforward from the preview points)
%   3. Wire the trajectory preview (distance/yaw points) and the new
%      K_FF / T_FF parameters into the controller
%   4. Add K_FF / T_FF mask parameters bound to
%      Driver.Lateral.StanleyFF.KFF/TFF and save sm_car.slx
%
%   Select the variant for a vehicle/maneuver by setting
%      Driver.Lateral.class.Value = 'Stanley_FF'
%   (see sm_car_gen_driver_database.m).
%
% Copyright 2026 The MathWorks, Inc.

mdl = 'sm_car';
load_system(mdl);

%% Locate the Lateral Driver variant subsystem (closed-loop driver)
latList = find_system(mdl,'LookUnderMasks','all','FollowLinks','on',...
    'BlockType','SubSystem','Name','Lateral Driver');
latList = latList(contains(latList,'Closed Loop'));
assert(~isempty(latList),'Lateral Driver subsystem not found in %s',mdl);
lat = latList{1};

srcBlk = [lat '/Stanley'];
dstBlk = [lat '/Stanley FF'];

%% 1. Copy the Stanley variant (remove existing copy if re-running)
if(getSimulinkBlockHandle(dstBlk) > 0)
    delete_block(dstBlk);
end
add_block(srcBlk,dstBlk);
set_param(dstBlk,'VariantControl','Stanley_FF');
pos = get_param(srcBlk,'Position');
set_param(dstBlk,'Position',pos+[0 250 0 250]);

%% 2. Disconnect Distance and Preview from their Terminators
for inp = {'Distance','Preview'}
    ph = get_param([dstBlk '/' inp{1}],'PortHandles');
    lh = get_param(ph.Outport(1),'Line');
    if(lh > 0), delete_line(lh); end
end
% Remove now-orphaned Terminators
terms = find_system(dstBlk,'SearchDepth',1,'BlockType','Terminator');
for i = 1:length(terms)
    ph = get_param(terms{i},'PortHandles');
    lh = get_param(ph.Inport(1),'Line');
    if(lh > 0), delete_line(lh); end
    delete_block(terms{i});
end

%% 3. Replace the controller code (adds input ports 9-13)
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','Path',[dstBlk '/Stanley Driver']);
assert(~isempty(ch),'MATLAB Function block not found in %s',dstBlk);
ch.Script = fileread(fullfile(fileparts(which(mfilename)),...
    'lateralDrivingControllerFF.m'));

%% 4. Add preview bus selector and feedforward parameter constants
fcnBlk = [dstBlk '/Stanley Driver'];
fPos   = get_param(fcnBlk,'Position');

bsBlk = [dstBlk '/Preview Select'];
add_block('built-in/BusSelector',bsBlk,...
    'Position',[fPos(1)-150 fPos(4)+40 fPos(1)-140 fPos(4)+90]);
set_param(bsBlk,'OutputSignals',...
    'pp_preview_dist_pts,pp_preview_yaw_pts,pp_preview_x_pts,pp_preview_y_pts');

add_block('built-in/Constant',[dstBlk '/Constant_KFF'],'Value','K_FF',...
    'Position',[fPos(1)-150 fPos(4)+110 fPos(1)-100 fPos(4)+130]);
add_block('built-in/Constant',[dstBlk '/Constant_TFF'],'Value','T_FF',...
    'Position',[fPos(1)-150 fPos(4)+150 fPos(1)-100 fPos(4)+170]);
add_block('built-in/Constant',[dstBlk '/Constant_KUS'],'Value','K_US',...
    'Position',[fPos(1)-150 fPos(4)+190 fPos(1)-100 fPos(4)+210]);
add_block('built-in/Constant',[dstBlk '/Constant_TREF'],'Value','T_REF',...
    'Position',[fPos(1)-150 fPos(4)+230 fPos(1)-100 fPos(4)+250]);

%% 5. Wire the new controller inputs
% Port order matches the argument list of lateralDrivingControllerFF:
% 9 = ad_vehicle (Distance), 10 = pp_preview_dist_pts,
% 11 = pp_preview_yaw_pts, 12 = K_FF, 13 = T_FF,
% 14 = pp_preview_x_pts, 15 = pp_preview_y_pts, 16 = K_US, 17 = T_REF
fph  = get_param(fcnBlk,'PortHandles');
dph  = get_param([dstBlk '/Distance'],'PortHandles');
pph  = get_param([dstBlk '/Preview'],'PortHandles');
bph  = get_param(bsBlk,'PortHandles');
kph  = get_param([dstBlk '/Constant_KFF'],'PortHandles');
tph  = get_param([dstBlk '/Constant_TFF'],'PortHandles');

add_line(dstBlk,dph.Outport(1),fph.Inport(9), 'autorouting','on');
add_line(dstBlk,pph.Outport(1),bph.Inport(1), 'autorouting','on');
add_line(dstBlk,bph.Outport(1),fph.Inport(10),'autorouting','on');
add_line(dstBlk,bph.Outport(2),fph.Inport(11),'autorouting','on');
add_line(dstBlk,kph.Outport(1),fph.Inport(12),'autorouting','on');
add_line(dstBlk,tph.Outport(1),fph.Inport(13),'autorouting','on');
add_line(dstBlk,bph.Outport(3),fph.Inport(14),'autorouting','on');
add_line(dstBlk,bph.Outport(4),fph.Inport(15),'autorouting','on');
uph = get_param([dstBlk '/Constant_KUS'],'PortHandles');
add_line(dstBlk,uph.Outport(1),fph.Inport(16),'autorouting','on');
rph = get_param([dstBlk '/Constant_TREF'],'PortHandles');
add_line(dstBlk,rph.Outport(1),fph.Inport(17),'autorouting','on');

%% 6. Extend the mask with the feedforward parameters
mk = Simulink.Mask.get(dstBlk);
mk.Type = 'Lateral Driver (Stanley + Curvature Feedforward)';
mk.Description = sprintf(['Stanley lateral control plus curvature ' ...
    'feedforward.\nThe feedforward term steers for the path curvature ' ...
    'T_FF seconds ahead\n(from the trajectory preview points) before ' ...
    'an error develops;\nStanley feedback corrects the residual.']);
if(isempty(mk.getParameter('K_FF')))
    mk.addParameter('Name','K_FF','Type','edit',...
        'Prompt','Curvature feedforward gain (1 = geometric)',...
        'Value','Lateral.StanleyFF.KFF.Value');
end
if(isempty(mk.getParameter('T_FF')))
    mk.addParameter('Name','T_FF','Type','edit',...
        'Prompt','Feedforward preview time (s)',...
        'Value','Lateral.StanleyFF.TFF.Value');
end
if(isempty(mk.getParameter('K_US')))
    mk.addParameter('Name','K_US','Type','edit',...
        'Prompt','Understeer gradient (rad per m/s^2 lateral accel)',...
        'Value','Lateral.StanleyFF.KUnder.Value');
end
if(isempty(mk.getParameter('T_REF')))
    mk.addParameter('Name','T_REF','Type','edit',...
        'Prompt','Reference preview time (s)',...
        'Value','Lateral.StanleyFF.TRef.Value');
end

%% 7. Save
save_system(mdl);
disp(['Installed Stanley FF lateral driver variant in ' mdl '.slx.']);
disp('Activate with Driver.Lateral.class.Value = ''Stanley_FF''');
end
