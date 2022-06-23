%dimensionamento bielle
clear
materialdata;

%inputs
%geometria bielle
betaC=4.3;
betaS=23.7;
phiPS=44.5;
alphaPR=2.4;
SSR=0.15;
LHPR=0.36;
hR=LHPR;
hS=SSR;% more or less

% buffer details
rin=.1;
rext=.14;
Aa=rin^2*pi; %[m^2]
Ab=rext^2*pi;
Pa0=3; %[MPa]
Pbsmax=15; %[10^6 N]

% buffer config
lambda=0.5; % rapporto tra dLEO0S, dLEO0c, possibly <0
dLOE0S=0.4;
xin=1.2;

IBEs=0.9;
IBE0=IBEs+dLOE0S;
IBEc=IBEs+lambda*dLOE0S;

%materials of bushing , biella and pin, 
materialindexRod=1;
materialindexPin=1;
D=.03; %hole diameter []
delta=0.7; %e/D
tau=1.2; %D/t
psi=0.9; %Dp/D

FcyRod=materials(materialindexRod).properties(5)*6.89476;
FtyRod=materials(materialindexRod).properties(3)*6.89476;
rhoRod=materials(materialindexRod).properties(1)*27679.9047;
Erod=materials(materialindexRod).properties(6)*6894.76; %[MPa]
FSn=1.75;
% materialindexBush=materialindexRod; hp: bushing and rod same material



xc=xin-Ab/Aa*dLOE0S*lambda;
Pac=Pa0*(xin/xc)^1.4;

Nc=Pac*Ab; %[MN]
Ns=Pbsmax*Ab;

%% primario-razzo
[Prbmin,Prbmax,PrBmin,PrBmax]= rodForces(5,180-(90-phiPS)+alphaPR,-Ns/2,-Nc/2);

Dr=D*1.2;

% pin-bushing bearing force (bushing same material as rod)
Pbr_Bush=FcyRod*Dr^2*psi/tau; %[MPa]*[m^2]=[MN]

% Pbr_Rod=FcyRod*D^2/tau; surely higher than previous
FSbrR=Pbr_Bush/max(abs([Prbmin,Prbmax,PrBmin,PrBmax]));

% max compression,tension net section strengh of B
A_BcrR=max([FSn*abs(PrBmin)/FcyRod],[FSn*PrBmax/FtyRod]);
% max compression,tension net section strengh of B 
A_bcrR=max([FSn*abs(Prbmin)/FcyRod],[FSn*Prbmax/FtyRod]);


% epsilonR=sqrt(1-A_BcrR/(2*Dr^2*delta/tau)); 
epsilonR=0.5;
sigmaR=0.35;
% sigmaR=A_bcrR/A_BcrR;
%  lug strenght
% worst case tension load=
PlugRmax=sqrt(Prbmax^2+PrBmax^2);
[FS_totR,~,~,massIndexR] = LugStrength(PlugRmax/2,0,Dr*100,delta,tau,psi,materialindexRod);
% nessun problema

% buckling BR
tBR=Dr/tau;
wBR=Dr*2*delta;
dBR=epsilonR*tBR;
sBR=epsilonR*wBR;
[GyRadBR,ABR] = gyrationRadiusIsection(tBR,wBR,dBR,sBR);
LBR=hR/tand(90-phiPS);
PbuckBR=pi^2*Erod*GyRadBR^2/(LBR*0.65)*ABR;
FSbuckBR=PbuckBR/-PrBmin;

% % buckling bR
tbR=Dr/tau*(sigmaR)^.5;
wbR=Dr*2*delta*(sigmaR)^.5;
dbR=epsilonR*tbR;
sbR=epsilonR*wbR;
[GyRadbR,AbR] = gyrationRadiusIsection(tbR,wbR,dbR,sbR);
LbR=hR*tand(90-phiPS);
PbuckbR=pi^2*Erod*GyRadbR^2/(LbR*0.65)*AbR;
FSbuckbR=PbuckbR/-Prbmin;


%% primario-secondario
[Psbmin,Psbmax,PsBmin,PsBmax]= rodForces(5,-betaS-betaC,-Ns/2,-Nc/2);

% pin-bushing bearing force (bushing same material as rod)
Pbr_Bush=FcyRod*D^2*psi/tau; %[MPa]*[m^2]=[MN]

% Pbr_Rod=FcyRod*D^2/tau; surely higher than previous
FSbrS=Pbr_Bush/max(abs([Psbmin,Psbmax,PsBmin,PsBmax]));

% max compression,tension net section strengh of B
A_BcrS=max([FSn*abs(PsBmin)/FcyRod],[FSn*PsBmax/FtyRod]);
% max compression,tension net section strengh of B 
A_bcrS=max([FSn*abs(Psbmin)/FcyRod],[FSn*Psbmax/FtyRod]);

% epsilonS=sqrt(1-A_BcrS/(2*D^2*delta/tau));
% sigmaS=A_bcrS/A_BcrS;
LBS=hS/tand(betaS);
LbS=hS*tand(betaS);
epsilonS=0.8;
sigmaS=0.15;

%  lug strenght
% worst case tension load=
PlugSmax=sqrt(Psbmax^2+PsBmax^2);
[FS_totS,~,~,massIndexS] = LugStrength(PlugSmax/2,0,D*100,delta,tau,psi,materialindexRod);
% nessun problema

% buckling BS
tBS=D/tau;
wBS=D*2*delta;
dBS=epsilonS*tBS;
sBS=epsilonS*wBS;
[GyRadBS,ABS] = gyrationRadiusIsection(tBS,wBS,dBS,sBS);

PbuckBS=pi^2*Erod*GyRadBS^2/(LBS*0.65)*ABS;
FSbuckBS=PbuckBS/-PsBmin;

% buckling bS
tbS=D/tau*(sigmaS)^.5;
wbS=D*2*delta*(sigmaS)^.5;
dbS=epsilonS*tbS;
sbR=epsilonS*wbS;
[GyRadbS,AbS] = gyrationRadiusIsection(tbS,wbS,dbS,sbR);

PbuckbS=pi^2*Erod*GyRadbS^2/(LbS*0.65)*AbS;
FSbuckbS=PbuckbS/-Psbmin;
%% pin strength > peggior caso comp o traz
Dp=D*psi;
PuJ_nom=min([FSbrS,FSbrR,FSn]*max(abs([Psbmin,Psbmax,PsBmin,PsBmax])));

FsuPin=materials(materialindexPin).properties(4)*6.89476;
FtuPin=materials(materialindexPin).properties(2)*6.89476;
rhoPin=materials(materialindexPin).properties(1)*27679.9047;
e_Pin=materials(materialindexPin).properties(7);

if e_Pin > 5
    kb=1.56;
else
    kb=1;
end

% shear critical value
Pus_P=2*(pi/4*(Dp)^2)*FsuPin;

% bending critical value
Mu_P=pi*(Dp)^3/32*kb*FtuPin; %
Larm=Dp/tau/4+Dp/tau/4;
Pub_P=2*Mu_P/Larm;

strongPin=sum(Pub_P>=[PuJ_nom Pus_P]);
if strongPin>0 && min([FSbrR,FSbrS])>FSn
    FStot=min([FSbrR,FSbrS]);
    massIndexP=Dp^3*(2/tau)*rhoPin;
    massindexTOT=massIndexP+massIndexS+massIndexR+(LbR*AbR+LbS*AbS+LBR*ABR+LBS*ABS)*rhoRod;
    disp(['    D     ','    delta ','    tau   ','   epsilonR ',' epsilonS ','  sigmaR    ','sigmaS  ','  FStot   ','massindexTOT ']) 
    disp([D,delta,tau,epsilonR,epsilonS,sigmaR,sigmaS,FStot,massindexTOT]) 
end





function [Pbmin,Pbmax,PBmin,PBmax]= rodForces(phis,phic,Ps,Pc)
 %90-phiPS o betaS%

 PBs=Ps;
 Pbsm=Ps*sind(-phis);
 PbsM=Ps*sind(phis);
 
 PBc=Pc*cosd(phic);
 Pbc=Pc*sind(phic);
 
 Pbmin=min([Pbsm PbsM Pbc]);
 PBmin=min([PBs PBc]);
 Pbmax=max([Pbsm PbsM Pbc]);
 PBmax=max([PBs PBc]); % max traction force
end

function [GyRad,A]= gyrationRadiusIsection(B,H,b,h)
% ref https://www.edutecnica.it/meccanica/inerzia/inerzia.htm
A=B*H-b*h;
Ix=1/12*(B*H^3-b*h^3);
Iy=1/12*(B^3*H-2*((b/2)^3*h+(h/2-b/4)^2*(b/2*h)));
GyRad=((Ix+Iy)/A)^.5;
end
