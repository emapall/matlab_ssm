clear all; close all;
%% From this script the output is
% The closure angle phi_closed (phi_sc: secondary closed) AND
% LHSR the distance from which the secondary joint comes out of the rocket

RR = 1.5;
LHS = 0.50;
% LHSR = 1; IS DETERMINED
% phi_closed = deg2rad(87); IS DETERMINED
Ls = 7;
SSR = 0.15;
CCL = 0.4;
CT = 0.3;
Lsp = Ls - CT;

RRp = 1.05*SSR+RR; %Radius Rocket prime (')
DO = sqrt(RRp.^2+CCL.^2);
alpha_DT = acos(RRp./DO);
alpha_CL = atan(CCL./RRp);
alpha_p = alpha_CL+alpha_DT;

BP = RR/2^0.5 - CCL;
xP = RRp-BP.*tan(alpha_p);
phi_s_closed = acos((RRp-xP)/(Lsp-LHS));
LHSR = xP - RR/2^.5 - LHS*cos(phi_s_closed);

disp("Closure angle");
disp(rad2deg(phi_s_closed));

disp("LHSR,cm");
disp(LHSR*100);

%  theta % rastremation angle of secondary leg, seen in "its" plane.


%% 