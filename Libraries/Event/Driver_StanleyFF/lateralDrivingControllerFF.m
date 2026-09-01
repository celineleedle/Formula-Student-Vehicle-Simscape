function steeringAngle = lateralDrivingControllerFF(referencePose, currentPose, v_x, direction, L_AXLE_DIST, MAX_STEERING_ANGLE_DEG, K_cte_forward, K_cte_reverse, ad_vehicle, pp_preview_dist_pts, pp_preview_yaw_pts, K_FF, T_FF, pp_preview_x_pts, pp_preview_y_pts, K_US, T_REF)
%#codegen % Directive for MATLAB Coder

% lateralDrivingControllerFF  Stanley lateral control + curvature feedforward
%
% Extends the Stanley method (Hoffmann et al. 2007) with a curvature
% feedforward term.  Stanley alone is purely reactive: it steers only in
% response to heading and cross-track error, so it turns in late at speed
% and the error grows with the speed of the trajectory.  The feedforward
% term steers for the path curvature ahead of the vehicle BEFORE an error
% develops, leaving the Stanley feedback to correct only the residual.
%
% Note: the template's referencePose input is interpolated a preview
% distance AHEAD of the vehicle (see getPoseCurRef).  That compensates
% reactive Stanley's lag, but produces a systematic offset to the inside
% of corners: the heading error toward the preview point is about
% (curvature * preview distance), far more than the Ackermann steer the
% corner needs, and equilibrium is reached only once the cross-track
% error has grown enough to cancel the excess.  This controller
% therefore rebuilds the TRUE closest-point reference from the first
% preview point (which lies at the vehicle's own position along the
% path) and provides anticipation through the explicit curvature
% feedforward instead.
%
% Inputs (first eight identical to lateralDrivingController):
%   referencePose: preview-point pose (UNUSED here, kept for port
%                  compatibility -- see note above)
%   currentPose:   [x_curr, y_curr, yaw_curr] vehicle reference point pose
%   v_x:           Current longitudinal velocity (m/s)
%   direction:     Driving direction (+1 forward, -1 reverse)
%   L_AXLE_DIST:   Wheelbase (m)
%   MAX_STEERING_ANGLE_DEG: Maximum steering angle (deg)
%   K_cte_forward, K_cte_reverse: Cross-track error gains
%   ad_vehicle:            Distance of vehicle along trajectory (m)
%   pp_preview_dist_pts:   Cumulative path distance of preview points (m)
%   pp_preview_yaw_pts:    Path yaw at preview points (rad, unwrapped)
%   K_FF:  Feedforward gain (1 = geometric Ackermann steer from curvature)
%   T_FF:  Feedforward preview time (s); curvature is evaluated
%          abs(v_x)*T_FF ahead of the vehicle along the path
%   pp_preview_x_pts, pp_preview_y_pts: Path position of preview points;
%          the first point is the closest-point reference
%   K_US:  Understeer gradient (rad per m/s^2 of lateral acceleration).
%          Near the grip limit the front tires need slip-angle steer well
%          beyond the kinematic Ackermann angle; this term feeds it
%          forward as K_US * v^2 * curvature.  Measure from a skidpad
%          test: slope of steering angle vs. lateral acceleration.
%   T_REF: Reference preview time (s).  The Stanley errors are evaluated
%          against the path point abs(v_x)*T_REF ahead rather than the
%          closest point: the short speed-scaled preview provides the
%          phase lead (damping) the feedback loop needs at speed, while
%          keeping the corner bias of a long fixed preview small.
%
% Output:
%   steeringAngle: Steering angle of the front wheels (rad, +left)

SOFTENING_CONSTANT_V = 0.1; % Prevents division by zero at very low speed

MAX_STEERING_ANGLE = MAX_STEERING_ANGLE_DEG * (pi/180);

% --- Input Validation / Pre-processing (Basic) ---
referencePose = referencePose(:)'; %#ok<NASGU> % Unused, see header note
currentPose = currentPose(:)';

% --- Extract Pose Components ---
x_vehicle_ref_pt   = currentPose(1);
y_vehicle_ref_pt   = currentPose(2);
yaw_vehicle        = currentPose(3);

% Reference point: short speed-scaled preview along the path.  The
% preview segment spans ad .. ad+pd, with the first point at the
% vehicle's own distance along the path.
n_pp     = length(pp_preview_dist_pts);
rel_dist = pp_preview_dist_pts - ad_vehicle;
d_ref    = abs(v_x) * T_REF;
d_ref    = max(rel_dist(1), min(d_ref, rel_dist(n_pp)));
idx_ref  = 1;
min_dref = inf;
for i = 1:n_pp
    dd = abs(rel_dist(i) - d_ref);
    if dd < min_dref
        min_dref = dd;
        idx_ref  = i;
    end
end
x_path   = pp_preview_x_pts(idx_ref);
y_path   = pp_preview_y_pts(idx_ref);
yaw_path = pp_preview_yaw_pts(idx_ref);

% --- 1. Calculate Front Axle Position ---
% Stanley controller operates on the error at the front axle.
x_front_axle = x_vehicle_ref_pt + L_AXLE_DIST * cos(yaw_vehicle);
y_front_axle = y_vehicle_ref_pt + L_AXLE_DIST * sin(yaw_vehicle);

% --- 2. Calculate Heading Error ---
heading_error_raw = yaw_path - yaw_vehicle;
psi_e = atan2(sin(heading_error_raw), cos(heading_error_raw));

% --- 3. Calculate Cross-Track Error at the Front Axle ---
dx_fa = x_front_axle - x_path;
dy_fa = y_front_axle - y_path;
e_fa = dx_fa * sin(yaw_path) - dy_fa * cos(yaw_path);

% --- 4. Cross-Track Steering Component (Stanley feedback) ---
effective_velocity_for_atan = abs(v_x) + SOFTENING_CONSTANT_V;

if direction == 1
    delta_cte = atan2(K_cte_forward *  e_fa, effective_velocity_for_atan);
else % if direction == -1
    delta_cte = atan2(K_cte_reverse * -e_fa, effective_velocity_for_atan);
end

% --- 5. Curvature Feedforward Component ---
% Path curvature = d(yaw)/d(distance).  Preview yaw is unwrapped and the
% preview distances are monotonically increasing, so a central finite
% difference around the point closest to the feedforward lookahead
% distance gives the curvature directly.
kappa_ff = 0.0;
if n_pp >= 2
    % Lookahead distance along the path, clamped to the preview segment
    d_ff = abs(v_x) * T_FF;
    d_ff = max(rel_dist(1), min(d_ff, rel_dist(n_pp)));

    % Index of preview point nearest the lookahead distance
    idx_ff = 1;
    min_diff = inf;
    for i = 1:n_pp
        dd = abs(rel_dist(i) - d_ff);
        if dd < min_diff
            min_diff = dd;
            idx_ff = i;
        end
    end

    % Central difference (one-sided at the segment ends)
    i1 = max(idx_ff-1, 1);
    i2 = min(idx_ff+1, n_pp);
    ds_seg = pp_preview_dist_pts(i2) - pp_preview_dist_pts(i1);
    if ds_seg > 1e-3
        kappa_ff = (pp_preview_yaw_pts(i2) - pp_preview_yaw_pts(i1)) / ds_seg;
    end
end

% Feedforward steer: geometric (Ackermann) angle for the curvature plus
% the dynamic (understeer) contribution for the lateral acceleration the
% curvature demands at the current speed
a_y_ff   = v_x^2 * kappa_ff;
delta_ff = K_FF * atan(L_AXLE_DIST * kappa_ff) + K_US * a_y_ff;

% --- 6. Combine Steering Components with Directionality ---
steeringAngleUnsaturated = direction * (psi_e + delta_cte + delta_ff);

% --- 7. Saturate Steering Angle ---
if steeringAngleUnsaturated > MAX_STEERING_ANGLE
    steeringAngle = MAX_STEERING_ANGLE;
elseif steeringAngleUnsaturated < -MAX_STEERING_ANGLE
    steeringAngle = -MAX_STEERING_ANGLE;
else
    steeringAngle = steeringAngleUnsaturated;
end

end
