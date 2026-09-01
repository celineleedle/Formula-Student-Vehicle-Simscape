function sm_car_trajectory_fsae_autox
% Function to construct FSAE Autocross trajectory
%   Computes the driving line and target speed profile for the autocross
%   course:
%     1. Driving line: minimum-curvature raceline within the track width
%        (sm_car_fsae_autox_raceline), or the course reference path
%     2. Speed profile: g-g diagram limits from the vehicle performance
%        envelope (sm_car_fsae_autox_vehicle_limits) -- speed-dependent
%        lateral grip with downforce, friction-ellipse coupling between
%        lateral and longitudinal acceleration, and a power cap -- applied
%        with forward/backward passes along the path.
%
%   All vehicle capability parameters live in
%   sm_car_fsae_autox_vehicle_limits.m; edit that file to retune the
%   trajectory for a different vehicle.
%
% Copyright 2026 The MathWorks, Inc.

cd(fileparts(which(mfilename)))

% Vehicle performance envelope and trajectory settings
limits = sm_car_fsae_autox_vehicle_limits;

grav   = 9.80665;                                  % m/s^2
kAero  = 0.5*limits.rhoAir*limits.CLA/limits.mVehicle; % Downforce accel/(m/s)^2
kDrag  = 0.5*limits.rhoAir*limits.CDA/limits.mVehicle; % Drag decel/(m/s)^2
mu     = limits.muRoad;

% Get driving line
course = sm_car_fsae_autox_define_course;
if(limits.useRaceline)
    path = sm_car_fsae_autox_raceline(course, limits.xMargin);
else
    path = course.refpath;
end
x_new = path(:,1)';
y_new = path(:,2)';
z_new = path(:,3)';

% Distance traveled along path
ds_seg          = sqrt(diff(x_new).^2+diff(y_new).^2);
xTrajectory_new = [0 cumsum(ds_seg)];

% Curvature from three-point finite differences
npts = length(x_new);
curv = zeros(1,npts);
for i = 2:npts-1
    p1 = [x_new(i-1) y_new(i-1)];
    p2 = [x_new(i)   y_new(i)];
    p3 = [x_new(i+1) y_new(i+1)];
    a = norm(p2-p1); b = norm(p3-p2); c = norm(p3-p1);
    area2 = (p2(1)-p1(1))*(p3(2)-p1(2))-(p2(2)-p1(2))*(p3(1)-p1(1));
    if(a*b*c > 0)
        curv(i) = 2*abs(area2)/(a*b*c);
    end
end
curv(1) = curv(2); curv(end) = curv(end-1);

% Smooth curvature to avoid speed target chatter.  Use max-preserving
% smoothing so tight corners (hairpin) keep their full curvature value.
curv = max(movmean(curv,5), movmax(curv,5));

% Cornering speed limit with speed-dependent grip:
%   v^2*curv <= mu*(g + kAero*v^2)   =>   v = sqrt(mu*g/(curv - mu*kAero))
% Downforce raises the limit; if curv <= mu*kAero grip grows faster than
% the demand and the corner is flat-out (vMax applies).
denom  = curv - mu*kAero;
vx_new = limits.vMax*ones(1,npts);
lim_i  = denom > mu*grav/limits.vMax^2;   % Corners below vMax
vx_new(lim_i) = sqrt(mu*grav./denom(lim_i));

% End of course: finish at low speed
vx_new(end) = limits.vEnd;
vx_new(1)   = limits.vStart;

% Forward (acceleration) and backward (braking) passes with the friction
% ellipse: longitudinal grip shrinks as cornering uses up the tires,
%   ax_avail = ax_max*sqrt(1 - (alat/alat_max)^2)
% Acceleration is also capped by drivetrain power.  Repeat the passes so
% the speed-dependent coupling converges.
for pass = 1:3
    % Forward pass: acceleration limit
    for i = 2:npts
        v     = vx_new(i-1);
        aGrip = mu*(grav + kAero*v^2);              % Total grip (m/s^2)
        eFac  = sqrt(max(0, 1 - (v^2*curv(i-1)/aGrip)^2));
        aPow  = limits.PMax/(limits.mVehicle*max(v,0.5)) - kDrag*v^2;
        aAcc  = max(0, min(aGrip*eFac, aPow));
        vx_new(i) = min(vx_new(i), sqrt(v^2 + 2*aAcc*ds_seg(i-1)));
    end
    % Backward pass: braking limit (drag assists)
    for i = npts-1:-1:1
        v     = vx_new(i+1);
        aGrip = mu*(grav + kAero*v^2);
        eFac  = sqrt(max(0, 1 - (v^2*curv(i+1)/aGrip)^2));
        aBrk  = aGrip*eFac + kDrag*v^2;
        vx_new(i) = min(vx_new(i), sqrt(v^2 + 2*aBrk*ds_seg(i)));
    end
end

% Brake-anticipation margin: pull braking onset earlier by the driver
% reaction time tauBrake.  Each target is the minimum of the profile over
% the window the vehicle covers in tauBrake seconds ahead.  In
% acceleration zones speeds ahead are higher, so this is a no-op there;
% in braking zones it starts the speed reduction earlier so the
% closed-loop driver reaches corner-entry speed in time.
if(limits.tauBrake > 0)
    vx_margin = vx_new;
    for i = 1:npts-1
        xWin = vx_new(i)*limits.tauBrake;
        j = i;
        while(j < npts && (xTrajectory_new(j+1)-xTrajectory_new(i)) <= xWin)
            j = j+1;
            vx_margin(i) = min(vx_margin(i), vx_new(j));
        end
    end
    vx_new = vx_margin;
end

% Predicted time along trajectory (point-mass estimate)
v_mid = 0.5*(vx_new(1:end-1)+vx_new(2:end));
tPred = sum(ds_seg./v_mid);
disp(['Trajectory: ' num2str(xTrajectory_new(end),'%3.1f') ' m, predicted time ' ...
    num2str(tPred,'%3.1f') ' s (point-mass estimate)']);

% Calculate target yaw angle (rad)
yaw_interval = 2;
aYaw_new = atan2(...
    y_new(yaw_interval+1:end)-y_new(1:end-yaw_interval),...
    x_new(yaw_interval+1:end)-x_new(1:end-yaw_interval));

aYaw_wrap = [repmat(aYaw_new(1),1,yaw_interval) aYaw_new];

% Unwrap aYaw so it is continuous
sigwrap = 0;
aYaw_new = aYaw_wrap;
for i = 2:length(aYaw_wrap)
    diff_aY = aYaw_wrap(i)-aYaw_wrap(i-1);
    if(diff_aY>pi)
        sigwrap = sigwrap-1;
    elseif(diff_aY<-pi)
        sigwrap = sigwrap+1;
    end
    aYaw_new(i) = aYaw_wrap(i)+2*pi*sigwrap;
end

% Plot trajectory and speed profile
fig_handle_name = 'h1_sm_car_fsae_autox';
handle_var = evalin('base',['who(''' fig_handle_name ''')']);
if(isempty(handle_var))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
elseif ~isgraphics(evalin('base',handle_var{:}))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
end
figure(evalin('base',fig_handle_name))
clf(evalin('base',fig_handle_name))

subplot(211)
plot(course.refpath(:,1),course.refpath(:,2),'--','Color',[0.6 0.6 0.6]); hold on
scatter(x_new,y_new,4,vx_new,'filled');
hold off
colorbar
axis equal
xlabel('X (m)'); ylabel('Y (m)');
title('FSAE Autocross Trajectory (color = target speed m/s, dashed = reference path)');

subplot(212)
plot(xTrajectory_new,vx_new)
xlabel('Distance Traveled (m)');
ylabel('Target Speed (m/s)');
title('Target Speed Along Trajectory');

% Assign parameters for trajectory definition
x.Value = x_new;
x.Units = 'm';
x.Comments = '';

y.Value = y_new;
y.Units = 'm';
y.Comments = '';

z.Value = z_new;
z.Units = 'm';
z.Comments = '';

xTrajectory.Value = xTrajectory_new;
xTrajectory.Units = 'm';
xTrajectory.Comments = 'Distance traveled';

vx.Value = vx_new;
vx.Units = 'm/s';
vx.Comments = 'Vehicle speed along direction of travel';

aYaw.Value = aYaw_new;
aYaw.Units = 'rad';
aYaw.Comments = 'Yaw Angle, non-wrapping';

save FSAE_AutoX_trajectory_default x y z vx aYaw xTrajectory
