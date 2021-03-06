clear all; close all;
%% Variables initialization
global rocketMass g d Lss Ls mu_din;
rocketMass = 10000; % 10 tons?
g = 10; %m/s^2
d = 4; %m
Ls = 8; % m, estimated from picture
alpha0 = deg2rad(30); %estimated from picture
a = 2; % m
Lss = Ls-a;
mu_din = 0.7;
v_vert0 = -3; %m/s
alphadot0 = v_vert0/Ls/cos(alpha0);
%% damper
global pIn xIn Aa Ab Kf Kp Lp_extended; 
pIn = 1e5; % 2 atm
xIn = 1; % m
Aa = pi*.3^2;
Ab = pi*.35^2;
Kf = 0;
Kp = 1e5; % 0.5 atm per 1 m/s of stroke compression speed;
Lp_extended =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha0));

%% differential equation
time_extremes = [0 5];
y0 = zeros(6,1);
%y is a coloumn vector holding the state of the system, so composed:
% 1 2 ) v_x/y rocket 3 4) x/y rocket
% 5) alpha_dot 6) alpha 
% 7) gamma_dot 8) gamma only if non-vertical
y0 = [0; v_vert0; 0; 5; alphadot0; alpha0;];
[t,y] = ode45(@odeFunRocket3D,time_extremes,y0);

