%% GeometriaGeneraleFinale is used to determine the precise geometry of the legs.
% It is function of both design choiches (RR,Ls,LHS,ISBR,sw,SSR,) and sizes coming
% from stress analysis and material choice (tp)

clear
close 

%% INPUTS

% design choiches [m]
RR = 1.5; % Rocket Radius
Ls = 9; % Length of secondary leg
LHS = 0.20; % length of paralle lections of secondary leg 
            % (from intersection with converging part to joint axis)
IBSR= .14; % internal buffer sleeve radius: the outer radius of the slimmer segment
sw=.01; % telescopic shoulder width
SSR = 0.15; % circumradius of secondary strut beam
CCL = IBSR+SSR; % clearence at the closed secondary strut contact point

% telescopic locking mechanism specifics 
ring_width=.051; % axial length locking mechanism casing
key_width=.025; % key width
lid_width=key_width/2; % axial length of compressed lid-spring mechanism
shoulder_length=.05; % shoulder length, the part of the telescopic segment that catches with the next/previous one duri opening 

% simulation results
tp=.015; % thickness of telescopic segment walls
dLOE0S=0.5612; % length contration of pneumatic segment from 
               % 0 (fully extended, instant before touchdown), 
               % to S (static condition, legs support the weight of the rocket)
dLOEOmax_stroke=.5739; % maximum contraction
phiSS=20.0699; % inclination of secondary strut plane wrt ground once the rocket is settled
Lp0=10.926; % length st touchdown (Lp_extended from main_sim)
PSR=IBSR+4*(tp+sw);

% new inputs, to change iteratively untill a design works
LHPR=PSR+.12; % distance of Jp axis from rocket plane, the higher the more the buckling risk
CT = 0.5; % horizontal distance (in the sec leg ref frame) from secondary leg tip the closure contact point 
AT = 0.5; % horizontal distance (in the sec leg ref frame) from secondary leg tip to primary anchor point 
dv=Ls*0.475; % vertical joint offset (design choice)
IBEs=1.1; % Internal Buffer Excess in the static position 
IBEc=IBEs+0.5*dLOE0S; % Internal Buffer Excess in the closed position,
                      % it must be higher than the static one, otherwise the compressive force wouldn't 
                      % decrease sufficiently, and joints (both J_P and
                      % J_SP) would need to react to a force in a direction
                      % they're not designed to account for. (I mean, they could, 
                      % but it would require more material, hence more
                      % weight). It should however be lower than the excess
                      % at 0 condition, when the leg is fully extended,
                      % because there's no much more internal buffer inside
                      % the external one, and it could slip out, if not constained 
SO=0.015; % [m] Safety Offsett, from the rocket surface to account for displacement caused by launch acoustic environment



%% SECONDARY STRUT GEOMETRY
% ref fig. 3.11
Lsp = Ls - AT; % Ls'
Lspp = Ls - CT; % Ls''

% ref fig. 3.13
RRp = 1*SSR+RR; %Radius Rocket prime, augmented by the secondary strut radius
DO = sqrt(RRp.^2+CCL.^2); 
alpha_T = acosd(RRp./DO);
alpha_CL = atand(CCL./RRp);
alpha_C = alpha_CL+alpha_T;
EC=(RR/2^0.5 - CCL).*tand(alpha_C);
xF = RRp - EC; % |OE| = |OC| - |CE|

% results, 
% ref figs. 3.14, 3.11
phiSC = acosd((RRp-xF)/(Lspp-LHS)); % closure angle of secondary strut
LHSs = LHS*cosd(phiSC); % LHS* : projection of the LHS part of the S.Strut on the xz plane when the leg is colsed
LHSR = xF - RR/2^.5 - LHSs; % protruding length of the Secondary Joint (Length Hinge Secondary-Rocket)

theta_s=atand((RR/2^.5-CCL)/(Lspp-LHS)); % secondary strut beams convergence semi-angle
dh=RR*(1-1/2^.5)-LHSR+LHPR; % horizontal joint offsett

clear  RRp xF DO alpha_T alpha_CL alpha_C

%% Secondary-Primary JOINT POSITIONING on secondary strut

% alphaPRmin=atand((PSR-IBSR)/IBEc)
htA=SSR-(CT-AT)*tand(90-phiSC); %available clearence at contact point
Jp_vect=[dh;dv;0]; % position vector of primary joint in the main reference frame centered in Js

% candidates
k=15; %n. of candidates
hJsp_arr=linspace(-0.4*SSR,0.95*(htA-IBSR),k); % array of differend heights to try, stayng inside 
                                                % the footprint of the secondary strut on the xz plane

Ls_vect=rotz(deg2rad(phiSC))'*[Ls;0;0];
Jsp_closed_vect=rotz(deg2rad(phiSC))'*[ones(1,k)*Lsp;hJsp_arr;zeros(1,k)]; % position of the closed leg PSjoint 
alphaPR_arr=atand((Jp_vect(1)-Jsp_closed_vect(1,:))./(Jsp_closed_vect(2,:)-Jp_vect(2))); % angle between the rocket axis and the closed primary strut axis
Jsp2PSR_vect=[-PSR;-IBEc;0]; % vector connecting the joint opint to the outermost edge og the closed primary strut, at risk of touching the rocket

% lil plot
plot(dh,dv,'ok') % Jp position
hold on 
plot(0,0,'ok') % Js position
plot([0 Ls_vect(1)],[0 Ls_vect(2)],'--','Color',[0.9 0.5 0.1],'LineWidth',1) % secondary strut axis
xline(dh-LHPR,'LineWidth',1.5,'Color',[0.6 0 0.8]); % rocket surface
xline(dh-LHPR+SO,'--','LineWidth',.75,'Color',[0.6 0 0.8]); % rocket surface + safety offsett
% axis equal
for i=1:k
Jsp2PSR_points(:,i)=Jsp_closed_vect(:,i)+rotz(deg2rad(alphaPR_arr(i)))'*Jsp2PSR_vect; % coordinates of problematic edge point
if Jsp2PSR_points(1,i)>(dh+SO-LHPR) % is it on the left or on the right of the rocket surface line (plus Safety Offset)?
    disp('ok')
else
    disp('compenetration') 
    i=i-1;
    break %as soon as there's contact the cycle stops, and the previous candidate position is selected as ok
end
plot([Jsp_closed_vect(1,i) Jsp2PSR_points(1,i)],[Jsp_closed_vect(2,i) Jsp2PSR_points(2,i)]);
plot([Jsp_closed_vect(1,i) dh],[Jsp_closed_vect(2,i) dv],'--','Color',[0 0.7 0]); % closed primary strut axis
end

if i==0 % no possible Jsp position was accettable
    disp('This configuration is not feasible, change the design, for example by increasing CT (to increase phiSC), decreasing AT \(to decrease alphaPR) or increase ISBC, to thave a shallower angle to work with')
    close
    return
else % the second to last candidate position was ok, I choose that on
hJsp=hJsp_arr(i);
alphaPR=alphaPR_arr(i);
betaC=90-phiSC+alphaPR; % amgle between primary and secondary strut axis in the closed position
Jsp_closed_vect=Jsp_closed_vect(:,i);
end

%% PRIMARY STRUT GEOMETRY

Jsp_vect=[Lsp;hJsp;0]; % when the secondary leg is aligned with the horizontal plane
% Jsp_closed_vect=rotz(deg2rad(phiSC))'*Jsp_vect;

Lpc_vect=Jp_vect-Jsp_closed_vect;
Lpc=norm(Lpc_vect); % length of closed primary strut 

Jsp_static_vect=rotz(deg2rad(-phiSS))'*Jsp_vect;
Lps_vect=Jsp_static_vect-Jp_vect;
Lps=norm(Lps_vect); % length of opened primary strut, in static conditions

Lsp_static_vect=rotz(deg2rad(-phiSS))'*[Lsp;0;0];

phiPS=acosd(Lps_vect'*[1;0;0]/Lps); % angle of primary strut wrt the horizontal plane 
betaS=acosd((Lsp_static_vect'*Lps_vect)/(Lsp*Lps)); % amgle between primary and secondary strut axis in the static position

%% FINE TUNUING, Rocket Joints Contraints

hfs=1.1; %hinge factor of safety, to account for hinge structure width

% leading edge
gammaLEmax=90-betaS-betaC; % semi angle of the cone available for the leading edge cap (without touching the hinge structure)
LHPS=ceil(100*hfs*IBSR/tand(gammaLEmax))/100; % lenght of the leading edge cap (Length of Hinge Primary to Secondary)

% tailing edge
gammaTEmax=90-phiPS-alphaPR; % semi angle of the cone available for the tailing edge cap (without touching the hinge structure)
LHP=ceil(100*hfs*PSR/tand(gammaTEmax))/100; % lenght of the tailing edge cap (Length of Hinge Primary)

%% PRIMARY STRUT SEGMENTS DIMENSION
lpm=Lpc-IBEc-LHP; %length of main segment

%y=t+w/2+2sltot+kw/2, offset from previous segment
% y=(3*lpm-(Lps-Lpc))/3;
tel_interference=2*shoulder_length+lid_width; % interference between 2 telescopic segments 

l_offset_tel=tel_interference+(ring_width+key_width)/2; % initial offset
l_offset_main=(IBEs+LHP+4*lpm-3*l_offset_tel-3*tel_interference-Lps)/3;
if l_offset_main<=0
    disp('The primary segment is too short, adjust measures to have a positive initial main offset')
    return
end

% lunghezze segmenti

lpm
lt1=lpm-l_offset_main
lt2=lt1-l_offset_tel
lt3=lt2-l_offset_tel
lb_min=IBEs+dLOE0S+2*shoulder_length % minimum internal buffer lenght

%% Length of segments for buckling analysis

L1=lpm+LHP-tel_interference/2
L2=lt1-tel_interference
L3=lt2-tel_interference
L4=Lps-L1-L2-L3;
minimum_buffer_interference_static=lt3+lb_min-L4
% L4=lt3+IBEs-tel_interference/2
IBE0=Lp0-L1-L2-L3-lt3+tel_interference/2
IBEmin=IBE0-dLOEOmax_stroke % some inconsistencies with main_sim due to 4mm discrepancy in total length