clear all; close all;
%% Variables initialization
global rocketMass g d Lsp Ls mu_din h0 dxMaxFoot thetaPS; %alpha0
rocketMass = 25000; % 10 tons?
g = 9.81; %m/s^2

% geometric parameters
% phiSC=88.07; % [deg] (from "Angolo_chiusura_distanze_giunti")%

phiS0=30; % [deg] instant before touchdown 

Ls = 9; % m, estimated from picture, design choice
alpha0 = deg2rad(phiS0); %[deg] estimated from picture,design choice
h0 = Ls*sin(alpha0); % m, distance of secondary strut joint from footpads plane
CT=0.5; % design choice
AT=0.5; % design choice
% a = CT+AT; % m

Lsp = Ls-AT;
Lspp = Ls-CT;

dv = 4.25; %m % design choice
dh = .49630; % (from "Angolo_chiusura_distanze_giunti")
d=sqrt(dv.^2+dh.^2);
thetaPS = atan(dv./dh);

mu_din = 0.7; % rubber footpad parameters
dxMaxFoot = 0.001;
% initial conditions

v_vert0 = -5; %m/s
% alphadot0 = v_vert0/Ls/cos(alpha0);
%% damper
% AUMENTARE LA CORSA E' BENEFICO --> POSSO DIMINUIRE KP E DIMINUISCO LA
% SOVRAPPRESS MAX INIZIALE!
global pIn xIn Aa Ab Kf Kp Lp_extended; 
pIn = 25e5; % 2 atm %initial pressure 
xIn = 1.5; % m air chamber initial length % < lb_min-1.1*LHPS
Aa = pi*.125^2; % outer to inner chamber valve area
Ab = pi*.15^2; % inner to outer chamber valve area
Kf = 0;
Kp = 65e5; % 0.5 atm per 1 m/s of stroke compression speed;
Lp_extended =sqrt(d^2+Lsp^2-2*d*Lsp*cos(alpha0 + thetaPS));


%% differential equation
time_extremes = [0 10];
y0 = zeros(6,1);
%y is a coloumn vector holding the state of the system, so composed:
% 1 2 ) v_x/y rocket 3 4) x/y rocket
% 5) alpha_dot 6) alpha 
% 7) gamma_dot 8) gamma only if non-vertical
y0 = [0; v_vert0; 0; Ls*sin(alpha0); 0];
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
sgtitle("Quantities vs time(s)","FontSize",20);

%
subplot(2,2,1);
plot(t,y(:,4));
title("Vertical position(m)");

%
subplot(2,2,2);
plot(t,R/rocketMass/g);
title("Reaction acceleration(g's)");

%
subplot(2,2,3);
yyaxis left;
plot(t,pa/1e5,'-b')
hold on
plot(t,pb/1e5,'-','COlor',[0 .6 0]);
ylabel("pressure [atm]");

yyaxis right;
plot(t,x,'Color',[0.9 .5 .1]);
ylabel("Piston stroke [m]","Color",'k');

title("Damper internal pressures and Piston stroke");
legend("Air pressure","Oil pressure","Piston stroke");

%
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


%%
% figure;
% plot(t,90-rad2deg(phi+alpha));
% title("Beta - primary to secondary leg angle ");
% xlabel("t-sec");
% ylabel("beta - deg");
% 
% figure;hold on;
% title("Phi and alpha vs time ");
% xlabel("t-sec");
% ylabel("deg ");
% plot(t,rad2deg(phi));
% plot(t,rad2deg(alpha));
% legend("Phi","alpha");

% external oil chamber run to constrain IBEs and IBEc

dLOE0S=(x(1)-x(end))/Ab*Aa
dLOEmax=(x(1)-min(x))/Ab*Aa
phiPS=90-phi(end)*180/pi
phiSS=alpha(end)*180/pi
betaS=90-rad2deg(phi(end)+alpha(end))
Lps=Lp(end)