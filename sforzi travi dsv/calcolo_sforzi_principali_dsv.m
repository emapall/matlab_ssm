clear all; close all;
% Fsx =  5.2734e+05; 4.0070e+05  
% Fsy = -2.4978e+05; -5.5703e+04
% From cad:
Lsv = 7;
del = 0.3;
rRocket = 3.7/2;
hy = rRocket/sqrt(2);

% From dynamic sim: 
theta = -deg2rad(25.3452); % from sim
Fx =  4.0070e+05/2; % 2.0144e+05;%
R = -5.5703e+04/2; %-1.2297e+05;%
% Fp = cose; 
% Bisognerebbe fare la somma vettoriale di tutte le forze 
hz = -(Lsv-del)*sin(theta);
dr = (Lsv-del)*cos(theta);

% ABOVE
dela = del*cos(theta);
la = sqrt(dr^2+hy^2); % l above (seen from above)
gamma = asin(hy/la); % >0!!!
xa = sqrt(dela^2+la^2+2*dela*la*cos(gamma));
alpha=gamma-atan(hy/(dela+dr));

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
a1 = asin(beam_v(2)/horiz_proj)-pi/2;
a2 = asin(beam_v(3)/1)-pi/2;

% Matrix such that vec(princ_sys) = MAT*vec(x,y,z_sys);
% Rotation of a1(<0) around z
R1 = rotz(a1);
R2 = rotx(a2);
R3 = rotz(0); % brings system into principal axsis
Rotmat = R3*R2*R1;



%%
% close all;
% org =[0;0;0];
% hold on;
% plot3([0 ,dr/beamL],[0 -hy/beamL],[0 -hz/beamL],"r*-");
% 
% plot3([0 1],[0 0],[0 0],"k:");
% plot3([0 0],[0 0],[0 1],"k:");
% plot3([0 0],[0 1],[0 0],"k:");
% plot3([0 cos(a1)],[0 sin(a1)],[0 0],"k:");

%% 
F_loc=Rotmat*F_tot_glob; % x<0 is traction, yloc,zloc are shear on the section
M_loc=Rotmat*M_tot_glob; % x is torsion, Myloc, Mzloc are bending

%% ASSUME CIRCULAR SECTION --> CALCULATE STRESSES, THICKNESS AND WHAT NOT

% rho = 0.15; % section radius
N = -F_loc(3); % traction force
T = max(abs([F_loc(1),F_loc(2)]));
Mt = abs(M_loc(3));
Mb = max(abs([M_loc(1),M_loc(2)]));

rho = 0.05:0.001:0.25;

vm_coeff = (1+Mb./rho/N).^2+3*((Mt./rho+2*T)/N).^2;
% sigma_y = 600E6;
k = 1./sqrt(vm_coeff);

%%
sigma_yeld = 600E6;
mat_dens = 8000;
weight = mat_dens * N/sigma_yeld./k*beamL;

figure;
subplot(2,1,1);
plot(rho*100,weight);
title("Weight (kg) vs radius of section (cm)");

subplot(2,1,2);
t = N/sigma_yeld./k/2/pi./rho;
plot(rho*100,t*1000);
title("Tickness(mm) vs radius of section(cm)");

% sgtitle("Steel");
%% 
sigma_yeld = 1500E6;
mat_dens = 3000; % 3 ton/m3 ??? fiber density
weight = mat_dens * N/sigma_yeld./k*beamL;

% figure;
subplot(2,1,1); hold on;
plot(rho*100,weight);
% legend("Steel - 600 Mpa","Carbon Fiber - 1500 Mpa");
% title("Weight (kg) vs radius of section (cm)");

subplot(2,1,2); hold on;
t = N/sigma_yeld./k/2/pi./rho;
plot(rho*100,t*1000);
legend("Steel - 600 Mpa","Carbon Fiber - 1500 Mpa");

% title("Tickness(mm) vs radius of section(cm)");

% sgtitle("Carbon fiber");

