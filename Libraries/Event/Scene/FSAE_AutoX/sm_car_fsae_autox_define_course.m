function course = sm_car_fsae_autox_define_course(varargin)
%sm_car_fsae_autox_define_course  Define FSAE Autocross course geometry
%   course = sm_car_fsae_autox_define_course
%   course = sm_car_fsae_autox_define_course(layout)
%
%   layout 'FSAE_2024' (default)
%       Course digitized from the official Formula SAE Michigan 2024
%       Skidpad/Autocross course map (fsaeonline.com).  The centerline
%       was traced from the map's driving line and scaled using the
%       25-ft grid.  Cones are placed along both track edges.
%
%   layout 'Custom'
%       Course assembled from the segment list below, following the FSAE
%       rules for autocross courses (Rules D8.6.1):
%         * Straights:      no longer than 60 m
%         * Constant turns: 23 m to 45 m diameter
%         * Hairpin turns:  minimum 9 m outside diameter
%         * Slaloms:        cones in a line, spaced 7.62 m to 12.19 m apart
%         * Track width:    minimum 3.5 m
%         * Course length:  less than 0.805 km
%       Edit the segment list ("seglist" below) to modify the course.
%       Segment types:
%         {'straight', length}                     length (m)
%         {'arc', radius, angle, dir}              radius (m) of centerline,
%                                                  angle (deg), dir 'L' or 'R'
%         {'slalom', nCones, spacing, amplitude}   nCones cones on the track
%                                                  centerline; the reference
%                                                  path weaves around them
%                                                  with the given amplitude (m)
%
%   Output structure fields:
%     course.ctrline   [x y z] track centerline for visuals  (m)
%     course.refpath   [x y z] driving line for the driver
%     course.cones     [x y z] cone positions                (m)
%     course.w         track width (m)
%
% Copyright 2026 The MathWorks, Inc.

if(nargin>0), layout = varargin{1}; else, layout = 'FSAE_2024'; end

% Track width (m), FSAE minimum is 3.5 m
track_w = 4.5;

switch layout
    case 'FSAE_2024'
        % Digitized centerline (starts at [-25 0], initial heading +x)
        data = load('FSAE_AutoX_2024_ctrline.mat');
        ctr  = data.ctrline(:,1:2);

        % The digitized line is the driving line: in slalom sections it
        % already weaves, so the reference path equals the centerline
        ref  = ctr;

        % Place cones along both track edges to mark the course
        coneSpacing = 6; % m
        d  = [0; cumsum(sqrt(sum(diff(ctr).^2,2)))];
        dc = (coneSpacing:coneSpacing:d(end)-coneSpacing)';
        cx = interp1(d,ctr(:,1),dc);
        cy = interp1(d,ctr(:,2),dc);
        % Unit normals from tangent direction
        tx = gradient(cx); ty = gradient(cy);
        tn = sqrt(tx.^2+ty.^2); tx = tx./tn; ty = ty./tn;
        nx = -ty; ny = tx;
        cone = [cx+nx*track_w/2 cy+ny*track_w/2;
                cx-nx*track_w/2 cy-ny*track_w/2];

    case 'Custom'
        % Distance between sample points along segments (m)
        ds = 0.5;

        % Course segment list
        seglist = {...
            {'straight', 40};                % Start straight
            {'arc',      15,  100, 'R'};     % Constant turn (30 m diameter)
            {'straight', 10};
            {'arc',      20,  150, 'L'};     % Sweeper (40 m diameter)
            {'straight',  8};
            {'slalom',    5, 10, 1.25};      % Slalom, 5 cones at 10 m
            {'straight',  6};
            {'arc',       5,  180, 'R'};     % Hairpin (10 m centerline diameter)
            {'straight', 30};
            {'arc',       8,   60, 'L'};     % Chicane
            {'arc',       8,   60, 'R'};
            {'arc',      12,  120, 'L'};     % Constant turn (24 m diameter)
            {'straight', 15};
            {'arc',      16,  110, 'R'};     % Constant turn (32 m diameter)
            {'slalom',    4,  9, 1.25};      % Slalom, 4 cones at 9 m
            {'straight', 35};                % Finish straight
            };

        % Walk along segments, keeping track of position and heading.
        x0 = -25; y0 = 0; aHead0 = 0;

        ctr  = [x0 y0];  % Track centerline points
        ref  = [x0 y0];  % Reference (driving line) points
        cone = zeros(0,2);  % Slalom cone positions

        pos   = [x0 y0];
        aHead = aHead0;

        for s_i = 1:length(seglist)
            seg = seglist{s_i};
            switch lower(seg{1})
                case 'straight'
                    len  = seg{2};
                    nPts = max(ceil(len/ds),2);
                    xl   = linspace(0,len,nPts+1)';
                    ptsL = [xl(2:end) zeros(nPts,1)];  % Local frame [x y]
                    ptsG = local2global(ptsL,pos,aHead);
                    ctr  = [ctr; ptsG]; %#ok<*AGROW>
                    ref  = [ref; ptsG];
                    pos  = ptsG(end,:);

                case 'arc'
                    r     = seg{2};
                    angSw = seg{3}*pi/180;
                    if(upper(seg{4})=='L'), sgn = 1; else, sgn = -1; end
                    nPts  = max(ceil(r*angSw/ds),4);
                    th    = linspace(0,angSw,nPts+1)';
                    ptsL  = [r*sin(th(2:end)) sgn*r*(1-cos(th(2:end)))];
                    ptsG  = local2global(ptsL,pos,aHead);
                    ctr   = [ctr; ptsG];
                    ref   = [ref; ptsG];
                    pos   = ptsG(end,:);
                    aHead = aHead + sgn*angSw;

                case 'slalom'
                    nCones = seg{2};
                    sp     = seg{3};
                    amp    = seg{4};
                    len    = (nCones+1)*sp;
                    nPts   = max(ceil(len/ds),2);
                    xl     = linspace(0,len,nPts+1)';
                    % Track centerline goes straight through the slalom
                    ptsL   = [xl(2:end) zeros(nPts,1)];
                    ptsG   = local2global(ptsL,pos,aHead);
                    ctr    = [ctr; ptsG];
                    % Cones on the centerline at spacing sp
                    coneL  = [(sp:sp:nCones*sp)' zeros(nCones,1)];
                    cone   = [cone; local2global(coneL,pos,aHead)];
                    % Reference path weaves around the cones
                    xw     = [0 (sp:sp:nCones*sp) len]';
                    yw     = [0 amp*(-1).^(0:nCones-1) 0]';
                    yl     = interp1(xw,yw,xl,'pchip');
                    refL   = [xl(2:end) yl(2:end)];
                    ref    = [ref; local2global(refL,pos,aHead)];
                    pos    = ptsG(end,:);

                otherwise
                    error(['Unknown segment type ' seg{1}]);
            end
        end

    otherwise
        error(['Unknown layout ' layout '. Use ''FSAE_2024'' or ''Custom''.']);
end

% Assemble output, z = 0 (flat course)
course.ctrline = [ctr zeros(size(ctr,1),1)];
course.refpath = [ref zeros(size(ref,1),1)];
course.cones   = [cone zeros(size(cone,1),1)];
course.w       = track_w;

end

function ptsG = local2global(ptsL,pos,aHead)
% Rotate points from segment-local frame to global frame and translate
R    = [cos(aHead) -sin(aHead); sin(aHead) cos(aHead)];
ptsG = (R*ptsL')' + pos;
end
