clear all; close all;
% From cad:
Lsv = 8.5;
del = 0.4;
hy = -1.5;

% From dynamic sim: 
theta = -deg2rad(30);
Fx = 1;
R = 1.5;
% Fp = cose; 
% Bisognerebbe fare la somma vettoriale di tutte le forze 
hz = -(Lsv-del)*sin(theta);
dr = (Lsv-del)*cos(theta);

% ABOVE
dela = del*cos(theta);
la = sqrt(dr^2+hy^2); % l above (seen from above)
gamma = -asin(hy/la); % >0!!!
xa = sqrt(dela^2+la^2+2*dela*la*cos(gamma));
alpha=atan(hy/(dela+dr));

Fva = Fx*tan(gamma-alpha);
Ftota=sqrt(Fva^2+Fx^2);
Ma = Fva*dela;


% FRONT
% lf = 8;
% ASSERT drf = sqrt(lf^2-hf^2) == dra;
% gammaf = asin(hz/lf);
% xf = sqrt(del^2+lf^2+2*del*lf*cos(gammaf));
% alphaf = atan(hz/(del+dr));
% ASSERT alphaf == asin(hz/xf);
% Mf = R*del;
% Fx = Fx+Fpx; R = R+Fpz;
F_tot_glob = [-Fx;Fva;-R]; %x,y,z system of reference
M_tot_glob = [0;0;-Ma];

%% ROTATION MATRICES
beam_ax = [dr;-hy;-hz];
beamL = norm(beam_ax);
horiz_proj = la/beamL;
beam_v = beam_ax/beamL;
% horiz_proj = sqrt(1-beam_ax(3)^2);
a1 = asin(beam_v(2)/horiz_proj);
a2 = asin(-beam_v(3)/1);

% Matrix such that vec(princ_sys) = MAT*vec(x,y,z_sys);
% Rotation of a1(<0) around z
R1 = rotz(a1);
R2 = roty(a2);
R3 = rotx(0); % brings system into principal axsis
Rotmat = R3*R2*R1;


%% 
F_loc=Rotmat*F_tot_glob; % x<0 is traction, yloc,zloc are shear on the section
M_loc=Rotmat*M_tot_glob; % x is torsion, Myloc, Mzloc are bending