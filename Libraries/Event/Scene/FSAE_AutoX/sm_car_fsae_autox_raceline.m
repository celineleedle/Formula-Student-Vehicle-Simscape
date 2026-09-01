function raceline = sm_car_fsae_autox_raceline(course, xMargin)
%sm_car_fsae_autox_raceline  Minimum-curvature raceline within track width
%   raceline = sm_car_fsae_autox_raceline(course, xMargin)
%
%   Computes an optimized driving line for the autocross course by
%   minimizing path curvature within the cone-to-cone corridor.  Larger
%   corner radii permit higher cornering speeds for the same lateral
%   acceleration limit, so the same closed-loop driver completes the
%   course faster with no controller changes.
%
%   The optimization is purely geometric -- it uses only the course
%   geometry (reference path, cones, track width) and a clearance margin.
%   No vehicle model parameters are involved, so the result is valid for
%   any vehicle whose half-width plus tracking error is below xMargin.
%
%   Method: each point of the reference path may move laterally along its
%   normal by an offset alpha, bounded by the corridor.  The sum of
%   squared second differences of the offset path (a discrete curvature
%   measure) is minimized by projected gradient descent.  The corridor
%   bound at each point is the smaller of (track halfwidth - xMargin) and
%   (distance to the nearest cone - xMargin), so the line cannot approach
%   any cone closer than xMargin, including slalom cones placed on the
%   track centerline in the 'Custom' layout.
%
%   Inputs
%     course   Structure from sm_car_fsae_autox_define_course
%              (fields refpath, cones, w)
%     xMargin  Clearance to cones/track edge (m)
%
%   Output
%     raceline [x y z] optimized driving line, sampled at ~1 m spacing
%
% Copyright 2026 The MathWorks, Inc.

% Resample reference path at uniform spacing (uniform ds makes the
% second-difference curvature measure consistent along the path)
ds   = 1.0; % m
ref  = course.refpath(:,1:2);
dseg = [0; cumsum(sqrt(sum(diff(ref).^2,2)))];
[dseg, iu] = unique(dseg);
ref  = ref(iu,:);
dq   = (0:ds:dseg(end))';
px   = interp1(dseg,ref(:,1),dq);
py   = interp1(dseg,ref(:,2),dq);
n    = length(dq);

% Unit normals from smoothed tangents
tx = gradient(px); ty = gradient(py);
tx = movmean(tx,5); ty = movmean(ty,5);
tn = sqrt(tx.^2+ty.^2);
nx = -ty./tn; ny = tx./tn;

% Corridor halfwidth: track edge and nearest-cone clearance
hw = (course.w/2 - xMargin)*ones(n,1);
if(~isempty(course.cones))
    cones = course.cones(:,1:2);
    dCone = zeros(n,1);
    for i = 1:n
        dCone(i) = sqrt(min((cones(:,1)-px(i)).^2 + (cones(:,2)-py(i)).^2));
    end
    hw = min(hw, dCone - xMargin);
end
hw = max(hw, 0);          % Never allow negative corridor
hw([1:3 end-2:end]) = 0;  % Pin start and finish to the reference path

% Minimize J(alpha) = |D*(px+nx.*alpha)|^2 + |D*(py+ny.*alpha)|^2
% where D is the second-difference operator (discrete curvature since
% spacing is uniform).  J is quadratic in alpha; solve by projected
% gradient descent with the box constraint |alpha| <= hw.
e  = ones(n,1);
D  = spdiags([e -2*e e],0:2,n-2,n);
Ax = D*spdiags(nx,0,n,n);
Ay = D*spdiags(ny,0,n,n);
bx = -D*px;
by = -D*py;

% Step size from largest eigenvalue of the Hessian (power iteration)
H = @(v) Ax'*(Ax*v) + Ay'*(Ay*v);
v = randn(n,1); v = v/norm(v);
for k = 1:30
    v = H(v); v = v/norm(v);
end
L    = 2*(v'*H(v));
step = 1/L;

alpha = zeros(n,1);
for k = 1:5000
    grad  = 2*(Ax'*(Ax*alpha - bx) + Ay'*(Ay*alpha - by));
    anew  = min(max(alpha - step*grad, -hw), hw);
    if(max(abs(anew-alpha)) < 1e-6), alpha = anew; break; end
    alpha = anew;
end

raceline = [px+nx.*alpha, py+ny.*alpha, zeros(n,1)];
end
