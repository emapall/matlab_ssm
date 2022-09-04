function [F, pa,pb,x] = damperForce(Lp,Lpdot)
% Returns the (axial module of the) force of the damper, given its current
% length and its derivative
global pIn xIn Aa Ab Kf Kp Lp_extended; 
% initial pressure and length of gas chamber, Areas of chambers,
% and K= force/Speed Coefficent
% K = Kq (force/volumetric flow rate coeff) *(1/) (Ab-Aa) or similar
x = xIn - (Lp_extended-Lp).*(Ab/Aa); % todo fix
% disp(x);
% disp(Lpdot);
pa = pIn*(xIn./x).^1.4; % pressure in gas chamber (and consequently in internal oil chamber) [Pa]
pb = pa-(Kp+Kf).*Lpdot; %>0 pressure in external oil chamber
F =  -(pa-(Kp+Kf).*Lpdot).*Ab; % force positive if traction, piston compresses if Lpdot<0 
% [N] = ([Pa] - [Pa/(m/s)] * [m/s])*[m^2]


% TODO: FIX, INTRODUCE THE NON LINEARITIES
%   1) end of extension for piston
%   2) no/reduced force in case of positive Lpdot, because different flow
%   valves? 
end

