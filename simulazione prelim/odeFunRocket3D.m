function [ydot] = odeFunRocket3D(t,y)
%y is a coloumn vector holding the state of the system, so composed:
% 1 2 ) v_x/y rocket 3 4) x/y rocket, 5) dx footpad wrt leg
% 
% 7) gamma_dot 8) gamma only if non-vertical
global rocketMass g h0;
ydot = zeros(size(y));


if  y(4)>h0 % NOTE: AN ASSERT SHOULD BE MADE AS TO WETHER LP>=LP_ext IF AND ONLY IF the rocket is not touching
    ydot(2) = -g;
    dxFootdot = -y(5);
    disp("Volo libero!!!");
else
%   [Lp,Lpdot,phi,Fp,pa,pb,Fsy,Fsx,R,Rx,x,dxFootdot]  
    [~ ,~    ,phi,Fp,~ ,~ ,Fsy,~  ,~,~ ,~,dxFootdot] = determineStateEvolution(t,y);
    ydot(2) = (Fsy-Fp*cos(phi))/rocketMass-g;
%     disp(y(4));
%     disp(Fp);
%     disp(y(2));
%     disp(t);
end
ydot(1) = 0; % we don't care about x in this case
ydot(3) = 0; % =y(1);
ydot(4) = y(2);
ydot(5) = dxFootdot;

% disp("----------------------");
% TODO: INSERT CHECK ON leg effectively touching the ground 
end