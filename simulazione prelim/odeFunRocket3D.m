function [ydot] = odeFunRocket3D(t,y)
%y is a coloumn vector holding the state of the system, so composed:
% 1 2 ) v_x/y rocket 3 4) x/y rocket
% 5) alpha_dot 6) alpha 
% 7) gamma_dot 8) gamma only if non-vertical
global rocketMass g Ls Lp_extended;
ydot = zeros(size(y));
alpha = y(6);


[Lp, ~,phi,Fp,~,~,Fsy,~,~,~] = determineStateEvolution(t,y);
if Lp>=Lp_extended && y(2)>0 % NOTE: AN ASSERT SHOULD BE MADE AS TO WETHER LP>=LP_ext IF AND ONLY IF the rocket is not touching
    ydot(2) = -g;
    ydot(5) = 0;
    ydot(6) = 0; %alpha - dot = alphadot;
else
    ydot(2) = (Fsy-Fp*cos(phi))/rocketMass-g;
    ydot(5) = y(2)/Ls/cos(alpha);
    ydot(6) = y(5); %alpha - dot = alphadot;
end
ydot(1) = 0; % we don't care about x in this case
ydot(3) = 0; % =y(1);
ydot(4) = y(2);

ydot(2) = (Fsy-Fp*cos(phi))/rocketMass-g;
ydot(5) = y(2)/Ls/cos(alpha);
ydot(6) = y(5); %alpha - dot = alphadot;

% TODO: INSERT CHECK ON leg effectively touching the ground 
end