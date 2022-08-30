clear
close

%nuovi input

 Lpc=4;
 alphaPR=4.95;
 
%  IBEs=0.9;

% Lpc=4;

% IBEs=1.1;

CA=.1;
% dvobj=
% dhobj=
% LHP=.5;
% LHPS=.2;

%da simulazioni precedenti
% dLOE=0.4; 
Lss=9;
CT=.2;
SSR=.15;
phiSC=87.97;
phiSS=20.78;
IBSR=.14; %Rext simulazione main
tp=.015;
sw=.01;


Ls=Lss+CT+CA;
IBE0=IBEs+dLOE;
PSR=IBSR+4*tp+3*sw;

% alphaPR=atand((PSR-IBSR)./IBE0);


% lunghezze data dalla geometria

betaC=90-phiSC+alphaPR;
htA=SSR-CA*tand(90-phiSC);
hGps= htA-IBSR*cosd(betaC); %hA


% conti vettoriali, con origine in Gs (giunto secondario), x radiale, y
% verticale e z, di gonseguenza, uscente dal foglio

Gps_vect=[Lss;hGps;0]; % when the secondary leg is aligned with the horizontal plane
GpsC_vect=rotz(deg2rad(phiSC))'*Gps_vect;
LssC_vect=rotz(deg2rad(phiSC))'*[Lss;0;0];
GsGps=norm(Gps_vect);
% xGps=Lss*cosd(phiSC)-hGps*cosd(alphaPR) sbagliato lol
% yGps=Lss*sind(phiSC)-hGps*sind(alphaPR)
Gp_vect=GpsC_vect+-rotz(deg2rad(alphaPR))'*[0;Lpc;0];


% le coordinate di Gp sono rispettivamente dh e dv

dh=Gp_vect(1);
dv=Gp_vect(2);
d=norm(Gp_vect);

thetaPS=atand(dv/dh);

GpsS_vect=rotz(deg2rad(-phiSS))'*Gps_vect;
Lps_vect=GpsS_vect-Gp_vect;
Lps=norm(Lps_vect);

phiPS=acosd(Lps_vect'*[1;0;0]/Lps);

LssS_vect=rotz(deg2rad(-phiSS))'*[Lss;0;0];
betaS=acosd((LssS_vect'*Lps_vect)/(Lss*Lps));
% phiSS=acosd((OGps^2+d^2-Lps^2)/(OGps*d))+atand(hGps/Lss)-phiSC




% gammaLE+betaC<90-betaS
% gammaLE=atand(IBSR/LHPS)
gammaLEmax=90-betaS-betaC;
LHPSmin=1.1*IBSR/tand(gammaLEmax);
% gammaTE+alphaPR<90-phiPS
% gammaTE=atand(PSR/LHP)
gammaTEmax=90-phiPS-alphaPR;
LHPmin=1.1*PSR/tand(gammaTEmax);

lpm_max=Lpc-IBE0-LHPmin;

% ltm=.3; %ingombro meccanismo bloccaggio tlescopico
% lpm=((Lps-Lpc)+IBEc-IBEs)/3+(tp-ltm);
% LHP=Lpc-IBEc-lpm
% if gammaLE<gammaLEmax && gammaTE<gammaTEmax
%     disp('design funzionante')
% else
%     disp('problema alle bielle')
% end

plot([0,LssS_vect(1)],[0,LssS_vect(2)],'--r')
hold on
xline(0);
yline(0);
plot([0,LssC_vect(1)],[0,LssC_vect(2)],'--r')
plot([Gp_vect(1),GpsS_vect(1)],[Gp_vect(2),GpsS_vect(2)],'--b')
plot([Gp_vect(1),GpsC_vect(1)],[Gp_vect(2),GpsC_vect(2)],'--b')
axis equal