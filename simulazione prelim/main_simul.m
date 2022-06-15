clear all; close all;
%% Variables initialization
global rocketMass g d Lss Ls mu_din  h0 dxMaxFoot; %alpha0
rocketMass = 50000; % 10 tons?
g = 10; %m/s^2
% geometric parameters
d = 4; %m
Ls = 7; % m, estimated from picture
h0 = 4; % m, distance of secondary strut joint from footpads plane
a = 0.0; % m
Lss = Ls-a;
mu_din = 0.7;
dxMaxFoot = 0.001;
% initial conditions
alpha0 = asin(h0/Ls); %estimated from picture
v_vert0 = -3; %m/s
% alphadot0 = v_vert0/Ls/cos(alpha0);
%% damper
% AUMENTARE LA CORSA E' BENEFICO --> POSSO DIMINUIRE KP E DIMINUISCO LA
% SOVRAPPRESS MAX INIZIALE!
global pIn xIn Aa Ab Kf Kp Lp_extended; 
pIn = 50e5; % 2 atm
xIn = 1; % m air chamber lenght initial
Aa = pi*.15^2;
Ab = pi*.2^2;
Kf = 0;
Kp = 40e5; % 0.5 atm per 1 m/s of stroke compression speed;
Lp_extended =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha0));

%% differential equation
time_extremes = [0 10];
y0 = zeros(6,1);
%y is a coloumn vector holding the state of the system, so composed:
% 1 2 ) v_x/y rocket 3 4) x/y rocket
% 5) alpha_dot 6) alpha 
% 7) gamma_dot 8) gamma only if non-vertical
y0 = [0; v_vert0; 0; 4; 0];
opt = odeset('AbsTol',1e-6,'RelTol',1e-6);
[t,y] = ode45(@odeFunRocket3D,time_extremes,y0,opt);

%% debugging and visual - re-calculating everything 
N = length(t);
Lp=zeros(N,1);
alpha=Lp; alphadot=Lp; Fsx=Lp; Fsy=Lp; phi=Lp;
Lpdot=Lp; Fp=Lp; pa=Lp; pb=pa; x=Lp; R=Lp;

for i=1:N
    alpha(i) = asin(y(i,4)/Ls);
    alphadot(i) = y(i,2)/Ls/cos(alpha(i));

    [cLp, cLpdot,cphi,cFp,cpa,cpb,cFsy,cFsx,cR,cRx,cx] = ...
                determineStateEvolution(t(i),y(i,:));
    Lp(i)=cLp; Lpdot(i)=cLpdot; phi(i)=cphi; Fp(i)=cFp; pa(i)=cpa; pb(i)=cpb; Fsy(i)=cFsy; Fsx(i)=cFsx; R(i)=cR; Rx(i)=cRx; x(i)=cx;
    if i>1
    if abs(Fsx(i)-Fsx(i-1))>abs(Fsx(i))
        k=1;
    end
    end
end

%% graphics: 
f1 = figure;
sgtitle("Quantities vs time(s)");
subplot(2,2,1);
plot(t,y(:,4));
title("Vertical position(m)");

subplot(2,2,2);
plot(t,R/rocketMass/g);
title("Reaction acceleration(g's)");

subplot(2,2,3);
plot(t,pa/1e5);hold on;plot(t,pb/1e5);
ylabel("press, atm");
yyaxis right;
ylabel("Piston stroke, m");
title("<-Pressure and Piston stroke->");
plot(t,x,"k");

subplot(2,2,4);
plot(t,Fp);
title("Piston Force");

%% call to animation function:

%% graphics:
% dt = diff(t);
% for i=1:length(t)
%     plot(nan,nan); hold on;
%     xLeg = Ls*cos(alpha(i));
%     dxFoot = y(i,5);
% %     xLeg = 0;
%     vxLeg = -Ls.*sin(alpha(i)).*alphadot(i)
%     plot([xLeg xLeg-dxFoot],[0 -1],"*-");
%     plot([xLeg+0 xLeg+vxLeg],[0 0],"g-o");
%     xaxis([0 9]);
%     hold off;pause(dt(i));
% end