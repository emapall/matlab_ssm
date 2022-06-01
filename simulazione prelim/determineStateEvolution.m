function [Lp, Lpdot,phi,Fp,pa,pb,Fsy,Fsx,R,Rx,x,dxFootdot] = determineStateEvolution(t,y)
% given the state in the differential equation for the rocket, returns the
% relevant variables to determine the state of the rocket and its evolution

global d Lss Ls mu_din dxMaxFoot;


alpha = asin(y(4)/Ls);
alphadot = y(2)/Ls/cos(alpha);
% FIRST: CALCULATE L_PRIMARY, L_PRIMARY DOT, AND PRIMARY FORCE
Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
phi = acos((Lss^2-d^2-Lp^2)/Lp/d/2); % cosine theorem: Lss^2 = d^2+Lp^2-2Lp*d*cos(phi)
Lpdot = d*Lss*cos(alpha)/Lp*alphadot;

vxFoot = -Ls.*sin(alpha).*alphadot;
dxFoot = y(5);
KFootReturn = 0.1;
if(vxFoot>=0)
    KSPeedFoot = min(1,max(-(1+KFootReturn)/0.1*(dxFoot-dxMaxFoot),-KFootReturn));
else
    KSPeedFoot = min(1,max((1+KFootReturn)/0.1*(dxFoot+dxMaxFoot),-KFootReturn));
end
disp(KSPeedFoot);
dxFootdot = KSPeedFoot*vxFoot;
mu_att = -dxFoot./dxMaxFoot*mu_din;
% following block gets overwritten

[Fp, pa, pb,x] = damperForce(Lp,Lpdot);
% disp([x, pa/1e5]);

R = (-Fp).*(sin(phi-alpha))./(...  % Ry
                    (cos(alpha)+mu_att)*(1-(Ls-Lss)/Lss));
if R<0
    R=0;
end
% TODO: R CANNOT BE LOWER THAN 0 (no contact)

if(x<=0)
    disp("Dippork!");
end

% THIRD: Calculate F_s x and y
Fsy = R+Fp*cos(phi); % TODO: +- ON this formula for the sign of Fpy
% Fs vettore = -Fp vettore + R vettore;
Rx = mu_att*R;
% Fsx = -sin*Fp, FP CON SEGNO
Fsx = mu_att*R -Fp*sin(phi); % TODO: +- ON this formula for the sign of Fpx

end

