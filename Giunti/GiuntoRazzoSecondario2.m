% Design giunto strutsecondario-razzo, sollecitato in trazione. 
% Air Force Method (https://mechanicalc.com/reference/lug-analysis#air-force-method)


% function [FS_tot,massIndex] = LugStrength(alpha,D,delta,tau,psi,materialindex)
clear
materialdata % load material properties (info >> help materialdata)

%% parametri Maschio (Gamba)

P = .6; % force [MN] 
Dcm = 3.5;% hole diameter [cm]

PM=P;
alpha = 0; % angle between transversal and axial components (atan(Ftr/Fax)) [deg] (25 per la femmina, 0 per il maschio)
materialindexM = 5;


delta = 1.5; % typical (e/D), e: outer lug radius 
tauM = 2.2; % D/t, t: lug thickness
psi = 1; % Dp/D, Dp: pin diameter, psi=1 when no bushing is installed

[FS_totM,FS_all_axM,~,massIndexM] = LugStrength(P,alpha,Dcm,delta,tauM,psi,materialindexM);

%% parametri Femmina (Razzo)

PF = P/2; % force [MN] 
alphaF = 25; % angle between transversal and axial components (atan(Ftr/Fax)) [deg] (25 per la femmina, 0 per il maschio)

materialindexF = 5;
tauF = 4; % D/t, t: lug thickness

[FS_totF,FS_all_axF,FS_all_trF,massIndexF] = LugStrength(PF,alphaF,Dcm,delta,tauF,psi,materialindexF);
massindexF=2*massIndexF;
%% nominal Joint strength

PuJ_nom=min([FS_totM,FS_totF]*P);

%% pin strength
materialindexP=1;
Dm=Dcm/100;
gap=0;

Pus_P=2*(pi/4*(Dm*psi)^2)*materials(materialindexP).properties(4)*6.89476;


Mu_P=pi*(Dm*psi)^3/32*1.56*materials(materialindexP).properties(2)*6.89476;
Larm=Dm*tauM/2+Dm*tauF/4+gap;
Pub_P=2*Mu_P/Larm;

strongPin=sum(Pub_P>=[PuJ_nom Pus_P]);

%problemo+pin debole



