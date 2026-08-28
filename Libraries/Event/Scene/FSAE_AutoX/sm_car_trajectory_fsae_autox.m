function sm_car_trajectory_fsae_autox
% Function to construct FSAE Autocross trajectory
%   Computes a target speed profile along the autocross driving line
%   using a curvature-based lateral acceleration limit and
%   forward/backward passes with longitudinal acceleration limits.
%
% Copyright 2026 The MathWorks, Inc.

cd(fileparts(which(mfilename)))

% Parameters for speed profile
v_start  = 1;     % Initial speed (m/s), matches Init vehicle speed
v_max    = 20;    % Maximum speed on straights (m/s)
v_end    = 2;     % Target speed at end of course (m/s)
gy_max   = 7;     % Maximum lateral acceleration (m/s^2)
gx_accel = 5;     % Maximum longitudinal acceleration (m/s^2)
gx_decel = 5;     % Maximum longitudinal deceleration (m/s^2)

% Get driving line
course = sm_car_fsae_autox_define_course;
x_new  = course.refpath(:,1)';
y_new  = course.refpath(:,2)';
z_new  = course.refpath(:,3)';

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

% Speed limit from lateral acceleration
vx_new = min(v_max, sqrt(gy_max./max(curv,1e-6)));

% End of course: finish at low speed
vx_new(end) = v_end;

% Forward pass: acceleration limit
vx_new(1) = v_start;
for i = 2:npts
    vx_new(i) = min(vx_new(i), sqrt(vx_new(i-1)^2 + 2*gx_accel*ds_seg(i-1)));
end

% Backward pass: braking limit
for i = npts-1:-1:1
    vx_new(i) = min(vx_new(i), sqrt(vx_new(i+1)^2 + 2*gx_decel*ds_seg(i)));
end

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
scatter(x_new,y_new,4,vx_new,'filled');
colorbar
axis equal
xlabel('X (m)'); ylabel('Y (m)');
title('FSAE Autocross Trajectory (color = target speed m/s)');

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
