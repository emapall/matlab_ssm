clear
close 

%input globali
RR = 1.5;
Ls = 9;
LHS = 0.20;
IBSR= .15;
tp=.015;
sw=.01;
PSR=IBSR+4*tp+3*sw;

SSR = 0.15;
CCL = IBSR+SSR;
 
% da altre simul
dLOE0S=0.3243;
SSR=.15;
phiSS=20.7494;

%nuovi input, da cambiare se viene suggerito un cambio di design
LHPR=PSR+0.12;
CT = 0.5;
CA =0;%e' di design o e' un risultato? dipende da come voglio
IBEs=1;% test
IBEc=IBEs+0.9*dLOE0S; %design


% geometria gambe secondarie
Lss =Ls - CA - CT;
Lsp = Ls - CT;
RRp = 1*SSR+RR; %Radius Rocket prime (')
DO = sqrt(RRp.^2+CCL.^2);
alpha_DT = acosd(RRp./DO);
alpha_CL = atand(CCL./RRp);
alpha_p = alpha_CL+alpha_DT;

BP = RR/2^0.5 - CCL;
xP = RRp-BP.*tand(alpha_p);
phiSC = acosd((RRp-xP)/(Lsp-LHS));
LHSR = xP - RR/2^.5 - LHS*cosd(phiSC);
theta_s=atand((RR/2^.5-CCL)/(Lsp-LHS));

%  theta % rastremation angle of secondary leg, seen in "its" plane.
dh=RR*(1-1/2^.5)-LHSR+LHPR;
dv=Lsp*0.5;

clear BP RRp xP DO alpha_DT alpha_CL alpha_p

% alphaPRmin=atand((PSR-IBSR)/IBEc) %[deg]
htA=SSR-CA*tand(90-phiSC);
%da simulazioni precedenti
%%

Gp_vect=[dh;dv;0];

hGsp_arr=linspace(-0.3*SSR,0.95*(htA-IBSR),20);
Ls_vect=rotz(deg2rad(phiSC))'*[Ls;0;0];
LssC_vect=rotz(deg2rad(phiSC))'*[ones(1,20)*Lss;hGsp_arr;zeros(1,20)];
alphaPR_arr=atand((Gp_vect(1)-LssC_vect(1,:))./(LssC_vect(2,:)-Gp_vect(2)));
A2spigoloPSR_vect=[-PSR;-IBEc;0];

plot(dh,dv,'o')
plot([0 Ls_vect(1)],[0 Ls_vect(2)],'--k')
xline(dh-LHPR);
hold on
axis equal
for i=1:20
A2spigoloPSR_points(:,i)=LssC_vect(:,i)+rotz(deg2rad(alphaPR_arr(i)))'*A2spigoloPSR_vect;
if A2spigoloPSR_points(1,i)>(dh-LHPR)
    disp('sku sku')
else
    disp('fuck')
    i=i-1;
    break
end
plot([LssC_vect(1,i) A2spigoloPSR_points(1,i)],[LssC_vect(2,i) A2spigoloPSR_points(2,i)]);
plot([LssC_vect(1,i) dh],[LssC_vect(2,i) dv],'--k');
end
if i==0
    disp('change design you piece of shit')
    close
    return
end
hGsp=hGsp_arr(i);
alphaPR=alphaPR_arr(i);
betaC=90-phiSC+alphaPR;

%%
Gps_vect=[Lss;hGsp;0]; % when the secondary leg is aligned with the horizontal plane
GpsC_vect=rotz(deg2rad(phiSC))'*Gps_vect;

Lpc_vect=Gp_vect-GpsC_vect;
Lpc=norm(Lpc_vect);

GpsS_vect=rotz(deg2rad(-phiSS))'*Gps_vect;
Lps_vect=GpsS_vect-Gp_vect;
Lps=norm(Lps_vect);



LssS_vect=rotz(deg2rad(-phiSS))'*[Lss;0;0];

phiPS=acosd(Lps_vect'*[1;0;0]/Lps);
betaS=acosd((LssS_vect'*Lps_vect)/(Lss*Lps));

%% 
gammaLEmax=90-betaS-betaC;
LHPS=ceil(110*IBSR/tand(gammaLEmax))/100;
% gammaTE+alphaPR<90-phiPS
% gammaTE=atand(PSR/LHP)
gammaTEmax=90-phiPS-alphaPR;
LHP=ceil(110*PSR/tand(gammaTEmax))/100;

lpm=Lpc-IBEc-LHP;

