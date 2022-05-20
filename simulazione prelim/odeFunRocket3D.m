function [ydot] = odeFunRocket3D(t,y)
%y is a coloumn vector holding the state of the system, so composed:
% 1 2 ) v_x/y rocket 3 4) x/y rocket
% 5) alpha_dot 6) alpha 
% 7) gamma_dot 8) gamma only if non-vertical
global rocketMass g d Lss Ls mu_din;
ydot = zeros(size(y));

alpha = y(6);
alphadot =y(5);
%% FIRST: CALCULATE L_PRIMARY, L_PRIMARY DOT, AND PRIMARY FORCE
Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
phi = acos((Lss^2-d^2-Lp^2)/Lp/d/2); % cosine theorem: Lss^2 = d^2+Lp^2-2Lp*d*cos(phi)
Lpdot = d*Lss*cos(alpha)/Lp*alphadot;
[Fp, pa, pb] = damperForce(Lp,Lpdot);
%% SECOND: CALCULATE R (system already solved on paper in closed form)
R = (-Fp)*(sin(phi-alpha))/(...
                    (cos(alpha)+sign(alphadot)*mu_din)*(1-(Ls-Lss)/Lss)...
            );
% TODO: R CANNOT BE LOWER THAN 0 (no contact)
%% THIRD: Calculate F_s x and y
Fsy = R+Fp*cos(phi); % TODO: +- ON this formula for the sign of Fpy
% Fs vettore = -Fp vettore + R vettore;
% Rx = sing(alphadot)*mu*R
% Fsx = -sin*Fp, FP CON SEGNO
Fsx = sign(alphadot)*mu_din*R -Fp*sin(phi); % TODO: +- ON this formula for the sign of Fpx
%% 4TH: CALCULATE THE VARIATION OF THE VARIOUS STATE VARIABLES
ydot(1) = 0; % we don't care about x in this case
ydot(3) = 0; % =y(1);
ydot(2) = (Fsy-Fp*cos(phi))/rocketMass-g;
ydot(4) = y(2);
ydot(5) = y(2)/Ls/cos(alpha);
ydot(6) = y(5); %alpha - dot = alphadot;
% ciao

hold on;
plot(t,pa,"ob");
plot(t,pb,"*k");
% Graphics(y(6),Fp,Fsx,Fsy);
% TODO: INSERT CHECK ON leg effectively touching the ground 
end