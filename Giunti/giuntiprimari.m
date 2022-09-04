% Rod Sizing For Primay Joints
clear
materialdata;

%% inputs
%geometry more ore less, to size thichnesses accordingly
betaC=4; % primary to secondary, closed [deg]
betaS=23; % primary to secondary, static [deg]
phiPS=44; %  static primary to horizontal plane [deg]
alphaPR=2.5; % close primary to zy plane [deg]
SSR=0.15; % [m]
LHPR=0.37; % [m]
hR=LHPR; % true value
hS=0.06; % at max (more severe buckling condition)
v_buffer=0.02; %[m^3] % volume of material
v_prebuffer=0.2; %[m^3]
primary_strut_density=8000; %[kg/m^3]
vertical_launch_acceleration=3; %[g]

% buffer details
rin=.125; % [m]
rext=.15; % [m]
Aa=rin^2*pi; %[m^2]
Ab=rext^2*pi;
Pa0=1.25; % initial pressure in air chamber [MPa]
Fb_max=0.3; %[10^6 N]

% buffer config
xin=1.5;
IBEs=1.10;
IBE0=1.45;
IBEC=1.34;

% materials of bushing , biella and pin sizing
materialindexRod=5;
materialindexBush=1;
materialindexPin=1;
D=.04; %hole diameter []
delta=0.8; %e/D
tau=1.4; %D/t
psi=0.9; %Dp/D
epsilonR=0.5; % adimensional parameters to shape the I section of the rods (form factor)
sigmaR=0.5; % size decrease factor 
epsilonS=0.7;
sigmaS=0.15;

% inputs processing

FcyRod=materials(materialindexRod).properties(5)*6.89476; % material properties
FtyRod=materials(materialindexRod).properties(3)*6.89476;
rhoRod=materials(materialindexRod).properties(1)*27679.9047;
Erod=materials(materialindexRod).properties(6)*6894.76; %[MPa]

FcyBush=materials(materialindexBush).properties(5)*6.89476; % material properties
FtyBush=materials(materialindexBush).properties(3)*6.89476;
rhoBush=materials(materialindexBush).properties(1)*27679.9047;
EBush=materials(materialindexBush).properties(6)*6894.76; %[MPa]

xc=xin-Ab/Aa*(IBE0-IBEC); % compression of gas chamber due to shortening during closure (closed -> 0: adiabatic)
Pac=Pa0*(xin/xc)^1.4;

launch_weight_buffer=v_buffer*primary_strut_density*9.81*(1+vertical_launch_acceleration);
launch_weight_prebuffer=v_prebuffer*primary_strut_density*9.81*(1+vertical_launch_acceleration);

%loading conditions
Nc_rocket=Pac*Ab+launch_weight_prebuffer/1e6; %[MN]
Nc_secondary=Pac*Ab-launch_weight_buffer/1e6;
Ns=Fb_max;

%% primario-razzo

[Prbmin,Prbmax,PrBmin,PrBmax]= rodForces(5,180-(90-phiPS)+alphaPR,-Ns/2,-Nc_rocket/2);

Dr=D*1;

% pin-bushing bearing force (bushing same material as rod)
Pbr_Bush=FcyBush*Dr^2*psi/tau; % Critical Bushing bearing load [MPa]*[m^2]=[MN]
% Pbr_Rod=FcyRod*D^2/tau; surely higher than previous
FS_bushingbearing=Pbr_Bush/max(abs([Ns,Nc_rocket])); %safety factor of bushing

% lug strenght
% worst case tension load=
P_lug_R_max=sqrt(Prbmax^2+PrBmax^2); % I take the maximum tensile value and apply it as an axial load for lug analysis

% checks for: lug bearing , net section and bushing bearing (overwritten by the more conservative Pbr_Bush) failures 
[FS_lugbearing_Rocket,~,~,massIndexR] = LugStrength(P_lug_R_max/2,0,Dr*100,delta,tau,psi,materialindexRod);

% buckling BR (B: bigger rod -BIELLA-; R: Rocket side )
tBR=Dr/tau;
wBR=Dr*2*delta;
dBR=epsilonR*tBR;
sBR=epsilonR*wBR;
[GyRadBR,ABR] = gyrationRadiusIsection(tBR,wBR,dBR,sBR);

FS_netsection_B_Rocket=min([FcyRod*ABR/abs(PrBmin),FtyRod*ABR/PrBmax]) % safety factor net section strenght

LBR=hR/tand(90-phiPS);
PbuckBR=pi^2*Erod*GyRadBR^2/(LBR*1.2)*ABR;  % Critical load for buuckling (fixed-rotationfixed eulero bernoulli beam)

FS_buckling_B_Rocket=PbuckBR/-PrBmin

% buckling bR (b: slimmer rod -biella-; R: Rocket side )
tbR=Dr/tau*(sigmaR)^.5;
wbR=Dr*2*delta*(sigmaR)^.5;
dbR=epsilonR*tbR;
sbR=epsilonR*wbR;
[GyRadbR,AbR] = gyrationRadiusIsection(tbR,wbR,dbR,sbR);

FS_netsection_b_Rocket=min([FcyRod*AbR/abs(Prbmin),FtyRod*AbR/Prbmax]) % safety factor net section strenght

LbR=hR*tand(90-phiPS);
PbuckbR=pi^2*Erod*GyRadbR^2/(LbR*1.2)*AbR; % Critical load for buuckling (fixed-rotationfixed eulero bernoulli beam)

FS_buckling_b_Rocket=PbuckbR/-Prbmin

FS_overall_PrimaryRocket=min([FS_netsection_b_Rocket,FS_buckling_b_Rocket,...
    FS_buckling_B_Rocket,FS_netsection_B_Rocket,FS_lugbearing_Rocket,...
    FS_bushingbearing]);

%% primario-secondario
% we extract the component of the forces in the two loading scenarios along
% the two perpendicular secondary joint rods. B (BIELLA) and b (biella)
[Psbmin,Psbmax,PsBmin,PsBmax]= rodForces(5,-betaS-betaC,-Ns/2,-Nc_secondary/2); % forces halved becouse of double rod

% % pin-bushing bearing force (bushing same material as rod)
% Pbr_Bush=FcyBush*D^2*psi/tau; %[MPa]*[m^2]=[MN] the bearing strength of the bushing is lower, when of the same material : Pbr_Rod=FcyRod*D^2/tau
% 
% % safety factor for secondary bearing strength
% FS_bushingbearing=Pbr_Bush/Ns;

% lug strenght
P_lug_S_max=sqrt(Psbmax^2+PsBmax^2); % worst case tension load
[FS_lugbearing_Secondary,~,~,massIndexS] = LugStrength(P_lug_S_max/2,0,D*100,delta,tau,psi,materialindexRod);

% cross section BS
tBS=D/tau;
wBS=D*2*delta;
dBS=epsilonS*tBS;
sBS=epsilonS*wBS;
[GyRadBS,ABS] = gyrationRadiusIsection(tBS,wBS,dBS,sBS);

FS_netsection_B_Secondary=min([FcyRod*ABS/abs(PsBmin),FtyRod*ABS/abs(PsBmax)]); % safety factor net section strenght

% buckling BS
LBS=hS/tand(betaS);
PbuckBS=pi^2*Erod*GyRadBS^2/(LBS*1.2)*ABS; % Critical load for buuckling (fixed-freerotationfixed eulero bernoulli beam)
FS_buckling_B_Secondary=PbuckBS/-PsBmin;

% cross section bS
tbS=D/tau*(sigmaS)^.5;
wbS=D*2*delta*(sigmaS)^.5;
dbS=epsilonS*tbS;
sbR=epsilonS*wbS;
[GyRadbS,AbS] = gyrationRadiusIsection(tbS,wbS,dbS,sbR);

FS_netsection_b_Secondary=min([FcyRod*AbS/abs(Psbmin),FtyRod*AbS/Psbmax]); % safety factor net section strenght

LbS=hS*tand(betaS);
PbuckbS=pi^2*Erod*GyRadbS^2/(LbS*1.2)*AbS;
FS_buckling_b_Secondary=PbuckbS/-Psbmin;

FS_overall_SecondaryRocket=min([FS_netsection_b_Secondary,FS_buckling_b_Secondary,...
    FS_buckling_B_Secondary,FS_netsection_B_Secondary,FS_lugbearing_Secondary,...
    FS_bushingbearing]);


%% pin strength > peggior caso comp o traz

Dp=D*psi; % pin diameter 

Pultimate_joint=min([FS_bushingbearing,FS_bushingbearing,...
    FS_lugbearing_Rocket,FS_lugbearing_Secondary]*max(abs([Prbmin,Prbmax,PrBmin,PrBmax,...
    Psbmin,Psbmax,PsBmin,PsBmax])));

% pin material properties
FsuPin=materials(materialindexPin).properties(4)*6.89476;
FtuPin=materials(materialindexPin).properties(2)*6.89476;
rhoPin=materials(materialindexPin).properties(1)*27679.9047;
e_Pin=materials(materialindexPin).properties(7);

if e_Pin > 5
    kb=1.56;
else
    kb=1;
end

% ultimate shear load
Pus_P=2*(pi/4*(Dp)^2)*FsuPin;

% bending ultimate load
Mu_P=pi*(Dp)^3/32*kb*FtuPin; %
Larm=D/tau/2; % tauM = 2tauF
Pub_P=2*Mu_P/Larm;

FS_Shear_Pin=Pus_P/Ns;
FS_Bending_Pin=Pub_P/Ns;
FS_overall_Pin=min(FS_Shear_Pin,FS_Shear_Pin) ;
strongPin=sum(Pub_P>=[Pultimate_joint Pus_P]);
massIndexP=Dp^3*(2/tau)*rhoPin;
massindexTOT=massIndexP+massIndexS+massIndexR+(LbR*AbR+LbS*AbS+LBR*ABR+LBS*ABS)*rhoRod;
%     disp(['    D     ','    delta ','    tau   ','   epsilonR ',' epsilonS ','  sigmaR    ','sigmaS  ','  FStot   ','massindexTOT ']) 
%     disp([D,delta,tau,epsilonR,epsilonS,sigmaR,sigmaS,FStot,massindexTOT]) 





%% functions

function [Pbmin,Pbmax,PBmin,PBmax]= rodForces(phis,phic,Ps,Pc)
% phis: angle between BIELLA and load direction in static conditions
% phic: angle between biella and load direction in closed configuration
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
