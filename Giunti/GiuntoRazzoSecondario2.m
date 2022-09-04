% Design giunto strutsecondario-razzo, sollecitato in trazione. 
% Air Force Method (https://mechanicalc.com/reference/lug-analysis#air-force-method)

% function [FS_tot,massIndex] = LugStrength(alpha,D,delta,tau,psi,materialindex)
clear
materialdata % load material properties (info >> help materialdata)

%% parameters

P = .1; % force [MN]  
Dcm = 4.5;% hole diameter [cm]

alphaM = 0; % angle between transversal and axial components (atan(Ftr/Fax)) [deg] (25 per la femmina, 0 per il maschio)
alphaF = 30; % angle between transversal and axial components (atan(Ftr/Fax)) [deg] (25 per la femmina, 0 per il maschio)
materialindexM = 5; % 5: titanium plate
materialindexF = 5; 
materialindexP= 6 ; % 6: titanium rod
deltaM = 1.5; % typical (e/D), e: outer lug radius 
deltaF= 1.5;
tauM = 2; % D/t, t: lug thickness
tauF = 4; % D/t, t: lug thickness
psi = 1; % Dp/D, Dp: pin diameter, psi=1 when no bushing is installed (or bushing of the same material)
gap=0; % firt approximation clearence between male and female

%% Maschio (Gamba)

PM=P;
[FS_totM,FS_all_axM,~,massIndexM] = LugStrength(P,alphaM,Dcm,deltaM,tauM,psi,materialindexM);

%% Femmina (Razzo)

PF = P/2; % force [MN] 
[FS_totF,FS_all_axF,FS_all_trF,massIndexF] = LugStrength(PF,alphaF,Dcm,deltaF,tauF,psi,materialindexF);

massIndexF=2*massIndexF;
%% nominal Joint strength

PuJ_nom=min([FS_totM,FS_totF]*P);

%% pin strength

Dm=Dcm/100;
Dp=Dm*psi;

Fsu_P=materials(materialindexP).properties(4)*6.89476;
Ftu_P=materials(materialindexP).properties(2)*6.89476;
rho_P=materials(materialindexP).properties(1)*27679.9047;
e_P=materials(materialindexP).properties(7);

if e_P > 5
    kb=1.56;
else
    kb=1;
end

% shear critical value
Pus_P=2*(pi/4*(Dp)^2)*Fsu_P;

% bending critical value
Mu_P=pi*(Dp)^3/32*kb*Ftu_P; %
Larm=Dm/tauM/4+Dm/tauF/2+gap;
Pub_P=2*Mu_P/Larm;

strongPin=sum(Pub_P>=[PuJ_nom Pus_P]);

%the pin in strong 
massIndexP=Dp^3*(1/tauM+2/tauF)*rho_P;
%problemo+pin debole, risolto: scritta male la formla di Larm

FSJoint=PuJ_nom/P
massIndexJoint=massIndexF+massIndexM+massIndexP




