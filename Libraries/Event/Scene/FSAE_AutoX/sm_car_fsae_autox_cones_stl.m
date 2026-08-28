function sm_car_fsae_autox_cones_stl
%sm_car_fsae_autox_cones_stl  Create STL file with FSAE Autocross cones
%   sm_car_fsae_autox_cones_stl
%   This function writes a single STL file containing one truncated cone
%   at each slalom cone position defined in the scene data.  The file is
%   referenced by a File Solid block in the FSAE AutoX scene.
%
% Copyright 2026 The MathWorks, Inc.

% Write STL next to this file so it is found on the project path
cd(fileparts(which(mfilename)))

scene_data = sm_car_scenedata_fsae_autox;

pos = scene_data.Cones.pos;
r0  = scene_data.Cones.r_base;
r1  = scene_data.Cones.r_top;
h   = scene_data.Cones.height;

nSeg = 20;  % Facets around each cone
th   = linspace(0,2*pi,nSeg+1); th(end) = [];

% Build triangle list for one cone at origin (base at z=0)
% Vertices: base ring, top ring, base center, top center
F = zeros(0,9); % Each row: [x1 y1 z1 x2 y2 z2 x3 y3 z3]
for k = 1:nSeg
    k2 = mod(k,nSeg)+1;
    b1 = [r0*cos(th(k))  r0*sin(th(k))  0];
    b2 = [r0*cos(th(k2)) r0*sin(th(k2)) 0];
    t1 = [r1*cos(th(k))  r1*sin(th(k))  h];
    t2 = [r1*cos(th(k2)) r1*sin(th(k2)) h];
    % Side (two triangles), outward winding
    F(end+1,:) = [b1 b2 t2]; %#ok<*AGROW>
    F(end+1,:) = [b1 t2 t1];
    % Base (facing down)
    F(end+1,:) = [b1 [0 0 0] b2];
    % Top cap (facing up)
    F(end+1,:) = [t1 t2 [0 0 h]];
end

% Write ASCII STL with a copy of the cone at each position
fid = fopen(scene_data.Cones.stl_file,'w');
fprintf(fid,'solid fsae_autox_cones\n');
for c_i = 1:size(pos,1)
    off = repmat(pos(c_i,:),1,3);
    for f_i = 1:size(F,1)
        v = F(f_i,:) + off;
        p1 = v(1:3); p2 = v(4:6); p3 = v(7:9);
        n  = cross(p2-p1,p3-p1);
        if(norm(n)>0), n = n/norm(n); end
        fprintf(fid,'facet normal %e %e %e\n',n);
        fprintf(fid,'outer loop\n');
        fprintf(fid,'vertex %e %e %e\n',p1);
        fprintf(fid,'vertex %e %e %e\n',p2);
        fprintf(fid,'vertex %e %e %e\n',p3);
        fprintf(fid,'endloop\n');
        fprintf(fid,'endfacet\n');
    end
end
fprintf(fid,'endsolid fsae_autox_cones\n');
fclose(fid);

disp(['Wrote ' scene_data.Cones.stl_file ' with ' ...
    num2str(size(pos,1)) ' cones.']);
