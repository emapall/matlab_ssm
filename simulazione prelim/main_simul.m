clear all; close all;
%% Variables initialization
global rocketMass g d Lsp Ls mu_din h0 dxMaxFoot thetaPS; %alpha0

rocketMass = 7000; %[kg] ~ 25 tons / 4 legs
g = 9.81; %m/s^2

% geometric parameters

phiS0=30; % [deg] instant before touchdown 

Ls = 9; % m, estimated from picture, design choice
alpha0 = deg2rad(phiS0); %[deg] estimated from Falcon9 picture,design choice
h0 = Ls*sin(alpha0); % m, distance of secondary strut joint from footpads plane
CT=0.5; % design choice
AT=0.5; % design choice
% a = CT+AT; % m

Lsp = Ls-AT;
Lspp = Ls-CT;

dv = 4.25; %m % design choice
dh = .5063; % (from "Angolo_chiusura_distanze_giunti")
d=sqrt(dv.^2+dh.^2);
thetaPS = atan(dv./dh);

mu_din = 0.7; % rubber footpad parameters
dxMaxFoot = 0.001;
% initial conditions

v_vert0 = -5; %m/s
% alphadot0 = v_vert0/Ls/cos(alpha0);
%% damper parameters
% AUMENTARE LA CORSA E' BENEFICO --> POSSO DIMINUIRE KP E DIMINUISCO LA
% SOVRAPPRESS MAX INIZIALE!
global pIn xIn Aa Ab Kf Kp Lp_extended; 
pIn = 1.25e6; %  initial pressure [Pa]
xIn = 1.5; % air chamber initial length [m]  % < lb_min-1.1*LHPS
Aa = pi*.125^2; % outer to inner chamber valve area [m^2]
Ab = pi*.15^2; % inner to outer chamber valve area [m^2]
Kf = 0;
Kp = 1.25e6; % pressure loss coefficient [Pa/(m/s)];
Lp_extended =sqrt(d^2+Lsp^2-2*d*Lsp*cos(alpha0 + thetaPS));
plots=1;

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
[~,i4]=max(abs(R));
reaction_g_max=R(i4)/rocketMass/g

%% graphics: 
if plots==1
f1 = figure;
sgtitle("Quantities vs time(s)","FontSize",20);

%
subplot(2,2,1);
plot(t,y(:,4));
title("Vertical position(m)");

%
subplot(2,2,2);
plot(t,R/rocketMass/g);

title("Reaction acceleration [g]");
txt=(['Maximum value: ',num2str(round(R(i4)/rocketMass/g*1e2)/1e2),' g']);
text(5,0.5*R(i4)/rocketMass/g,txt)

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
title("Damper Force");


%%
[~,i1]=max(abs(Fsx));
[~,i2]=max(abs(Fsy));
[~,i3]=max(abs(Fp));

figure(2)
format short
subplot(2,2,1)
plot(t,Fsx)
ylabel('Fsx [N]')
txt=({['Maximum value: ',num2str(round(Fsx(i1)/1e3)/1e3),' MN'];['Static value: ',...
    num2str(round(Fsx(end)/1e3)/1e3),' MN'];['Impact Ratio= ',num2str(Fsx(i1)/Fsx(end))]});
text(7,0.7*Fsx(i1),txt)

subplot(2,2,2)
plot(t,Fsy)
ylabel('Fsy [N]')
txt=({['Maximum value: ',num2str(round(Fsy(i2)/1e3)/1e3),' MN'];['Static value: ',...
    num2str(round(Fsy(end)/1e3)/1e3),' MN'];['Impact Ratio= ',num2str(Fsy(i2)/Fsy(end))]});
text(7,0.7*Fsy(i2),txt)

subplot(2,2,3)
plot(t,Fp)
ylabel('Fp [N]')
txt=({['Maximum value: ',num2str(round(Fp(i3)/1e3)/1e3),' MN'];['Static value: ',...
    num2str(round(Fp(end)/1e3)/1e3),' MN'];['Impact Ratio= ',num2str(Fp(i3)/Fp(end))]});
text(7,0.7*Fp(i3),txt)

subplot(2,2,4)
plot(t,R)
ylabel('R [N]')
txt=({['Maximum value: ',num2str(round(R(i4)/1e3)/1e3),' MN'];['Static value: ',...
    num2str(round(R(end)/1e3)/1e3),' MN'];['Impact Ratio = ',num2str(R(i4)/R(end))]});
text(7,0.7*R(i4),txt)

%% secondary joint reaction characteristics, to size secondary joint
figure(3)
plot(atand(Fsy./Fsx)+180*(Fsx<0),sqrt(Fsx.^2+Fsy.^2))
xlabel('reaction direction angle with horizontal [deg]')
ylabel('reaction force modulus')
title('Secondary Joint reaction analysis')
% xlim([0 max([Fsx(i1),Fsy(i2)])])
% ylim([0 max([Fsx(i1),Fsy(i2)])])



end
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
%%
% external oil chamber run to constrain IBEs and IBEc

dLOE0S=(x(1)-x(end))/Ab*Aa
dLOEmax=(x(1)-min(x))/Ab*Aa
phiPS=90-phi(end)*180/pi
phiSS=alpha(end)*180/pi
betaS=90-rad2deg(phi(end)+alpha(end))
Lps=Lp(end)