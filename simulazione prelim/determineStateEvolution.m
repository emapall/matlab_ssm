function [Lp, Lpdot,phi,Fp,pa,pb,Fsy,Fsx,R,Rx,x] = determineStateEvolution(t,y)
% given the state in the differential equation for the rocket, returns the
% relevant variables to determine the state of the rocket and its evolution

global d Lss Ls mu_din;


alpha = asin(y(4)/Ls);
alphadot = y(2)/Ls/cos(alpha);
% FIRST: CALCULATE L_PRIMARY, L_PRIMARY DOT, AND PRIMARY FORCE
Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
phi = acos((Lss^2-d^2-Lp^2)/Lp/d/2); % cosine theorem: Lss^2 = d^2+Lp^2-2Lp*d*cos(phi)
Lpdot = d*Lss*cos(alpha)/Lp*alphadot;
[Fp, pa, pb,x] = damperForce(Lp,Lpdot);

% SECOND: CALCULATE R (system already solved on paper in closed form)
R = (-Fp).*(sin(phi-alpha))./(...
                    (cos(alpha)+sign(alphadot)*mu_din)*(1-(Ls-Lss)/Lss)...
            );
% TODO: R CANNOT BE LOWER THAN 0 (no contact)

% THIRD: Calculate F_s x and y
Fsy = R+Fp*cos(phi); % TODO: +- ON this formula for the sign of Fpy
% Fs vettore = -Fp vettore + R vettore;
Rx = sign(alphadot)*mu_din*R;
% Fsx = -sin*Fp, FP CON SEGNO
Fsx = sign(alphadot)*mu_din*R -Fp*sin(phi); % TODO: +- ON this formula for the sign of Fpx

end

