function [Lp, Lpdot,phi,Fp,pa,pb,Fsy,Fsx,R,Rx,x,dxFootdot] = determineStateEvolution(t,y)
% given the state in the differential equation for the rocket, returns the
% relevant variables to determine the state of the rocket and its evolution

global d Lsp Ls mu_din dxMaxFoot thetaPS;


alpha = asin(y(4)/Ls);
alphadot = y(2)/Ls/cos(alpha);
% FIRST: CALCULATE L_PRIMARY, L_PRIMARY DOT, AND PRIMARY FORCE
Lp =sqrt(d^2+Lsp^2-2*d*Lsp*cos(alpha + thetaPS));
% alpha >0 for leg under the horizon
phi = acos((+d^2+Lp^2-Lsp^2)/Lp/d/2) + thetaPS - pi/2; % cosine theorem: Lss2 = d2+Lp2-2 d Lp (PHIPv+(90 -thetaPS)) 
Lpdot = d*Lsp*sin(alpha + thetaPS)/Lp*alphadot;

vxFoot = -Ls.*sin(alpha).*alphadot;
dxFoot = y(5);
KFootReturn = 0.1;
if(vxFoot>=0)
    KSPeedFoot = min(1,max(-(1+KFootReturn)/0.1*(dxFoot-dxMaxFoot),-KFootReturn));
else
    KSPeedFoot = min(1,max((1+KFootReturn)/0.1*(dxFoot+dxMaxFoot),-KFootReturn));
end
% disp(KSPeedFoot);
dxFootdot = KSPeedFoot*vxFoot;
mu_att = -dxFoot./dxMaxFoot*mu_din;
% following block gets overwritten

[Fp, pa, pb,x] = damperForce(Lp,Lpdot);
% disp([x, pa/1e5]);

R = (-Fp).*(cos(phi+alpha))./...  % Ry
                    (cos(alpha)+mu_att*sin(alpha))*Lsp/Ls;
if R<0
    R=0;
end
% TODO: R CANNOT BE LOWER THAN 0 (no contact)

if(x<=0)
    disp("The piston broke through the chamber wall");
end

% THIRD: Calculate F_s x and y
Fsy = R+Fp*cos(phi); % TODO: +- ON this formula for the sign of Fpy
% Fs vettore = -Fp vettore + R vettore;
Rx = mu_att*R;
% Fsx = -sin*Fp, FP CON SEGNO
Fsx = mu_att*R -Fp*sin(phi); % TODO: +- ON this formula for the sign of Fpx

end

