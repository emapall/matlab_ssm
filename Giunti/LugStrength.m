% Design giunto strutsecondario-razzo, sollecitato in trazione. 
% Air Force Method (https://mechanicalc.com/reference/lug-analysis#air-force-method)
% sintax;
% function [FS_tot,FS_all_ax,FS_all_tr,massIndex] = LugStrength(alpha,D,delta,tau,psi,materialindex)

function [FS_tot,FS_all_ax,FS_all_tr,massIndex] = LugStrength(P,alpha,D,delta,tau,psi,materialindex)

materialdata % load material properties (info >> help materialdata)

%% parametri [inches, pounds]

% P = .6; % force [MN] 
% alpha = 25; % angle between transversal and axial components (atan(Ftr/Fax)) [deg] (25 per la femmina, 0 per il maschio)
% materialindex = 5;

% D = 3.5;% hole diameter [cm]
% delta = 1.5; % typical (e/D), e: outer lug radius 
% tau = 2; % D/t, t: lug thickness
% psi = 1; % Dp/D, Dp: pin diameter, psi=1 when no bushing is installed


% data processing
D=D/100;
rho=materials(materialindex).properties(1)*27679.9047; %[kg/m^3]
Ftu=materials(materialindex).properties(2)*6.89476; %[MPa]
Fty=materials(materialindex).properties(3)*6.89476; %[MPa]
Fcy=materials(materialindex).properties(5)*6.89476; %[MPa]
E=materials(materialindex).properties(6)*6.89476; %[GPa]
e=materials(materialindex).properties(7);

x=1/(2*delta);
phi1=Ftu/Fty;
phi2=Ftu/E/e;

%% Kn data
% fty/ftu=0.6
% x=D/w=1/2delta

D_wv=[0 .2 .4 .6 .8];
phi2v=[.2 .4 .6 .8 1];
phi1v=[.6 .8 1];

[X,Y,Z]=meshgrid(D_wv,phi2v,phi1v);

 V(:,:,1)=[0 .6 .68 .74 .85
           0 .4 .61 .68 .82
           0 .3 .5 .62 .75
           0 .25 .4 .52  .71
           0 .2  .33 .46 .64];
       
 
 V(:,:,2)=[0 .67 .82 .86 .92
           0 .42 .67 .77 .88
           0 .30 .53 .65 .83
           0 .24 .42 .65 .75
           0 .2  .34 .47  65]; 

 V(:,:,3)=[0 .72 .96 .96 1 
           0 .43 .72 .84 .96
           0 .31 .54 .7 .92
           0 .23 .42 .57 .76
           0 .2  .35 .47 .63]; 
       
Kn=interp3(X,Y,Z,V,x,phi2,1/phi1,'spline');

%% Kbr data
if delta>=1.5
    Kbr=interp1([1.5 1.6 1.8 2 2.2 2.4],[1.34 1.42 1.56 1.7 1.83 1.94],delta,'spline');
else
    Kbr=interp1([0.5 .6 .8 1 1.2 1.4 1.5],[2 1.91 1.75 1.6 1.48 1.37 1.34],delta,'spline');
end
%%

if phi1>1.304
    Ftus=Fty*1.304;
else
    Ftus=Ftu;
end

Fcu=Fcy*1.304;

%% axial force (MALE)

Pax=P*cosd(alpha);
Pu_ax_expected=Pax;

% net section tension failure
Pnu_L=Ftus*Kn*D^2*(2*delta-1)/tau;
FS_NST=Pnu_L/Pu_ax_expected;

% lug bearing faiulure
if delta<1.5
    Pbru_L=Ftus*Kbr*D^2*(delta-1/2)/tau;
else
    Pbru_L=Ftus*Kbr*D^2/tau;
end
FS_LBR=Pbru_L/Pu_ax_expected;

% bushing bearing failure
Pu_B=Fcu*D^2*psi/tau;
FS_BBR=Pu_B/Pu_ax_expected;

% Margin of Safeties
% NST_MoS=Kn/Kn_crit-1
% LBR_MoS=Kbr/Kbr_crit-1
% BBR_Mos=psi/psi_crit-1

Paxu_L_B=min([Pu_B Pbru_L Pnu_L]); 
FS_all_ax=[FS_NST,FS_LBR,FS_BBR];
% mass index
massIndex=D^3*(delta-1/2)*(delta+1/2)*rho/tau;

%% transverse - oblique loading
if alpha~=0
    Ptr=P*sind(alpha);
    
    hav=D*(delta-0.5)*(delta-.354)/(delta-.451);
    
    Ktr=interp1([0 .3 .67 1.16 1.5],[0 .4 .8 1.2 1.33],hav/D,'spline');
    
    if phi1<=1.304 && (hav/D)>0.8
        Ktr=Ktr+(hav/D-.8)*.2;
    end
    
    Ptru_L=Ktr*Ftus*D^2/tau;
    FS_TRL=Ptru_L/Ptr;
    
    Ptru_L_B=min([Pu_B Ptru_L]); 
    
    Pax_ult=( 1 / ((1/Paxu_L_B)^1.6+(tand(alpha)/Ptru_L_B)^1.6) )^0.625;
    
    Pult=Pax_ult*(1+(tand(alpha))^2)^.5;
    
    FS_all_tr=[FS_TRL,FS_BBR];

else 
    FS_all_tr=[];
    Pult=Paxu_L_B;
    
end
FS_tot=Pult/P;

% end
