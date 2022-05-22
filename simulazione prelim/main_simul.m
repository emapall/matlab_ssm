clear all; close all;
%% Variables initialization
global rocketMass g d Lss Ls mu_din  h0 vxStop; %alpha0
rocketMass = 10000; % 10 tons?
g = 10; %m/s^2
vxStop = 1e-1;
% geometric parameters
d = 4; %m
Ls = 8; % m, estimated from picture
h0 = 4; % m, distance of secondary strut joint from footpads plane
a = 2; % m
Lss = Ls-a;
mu_din = 0.7;

% initial conditions
alpha0 = asin(h0/Ls); %estimated from picture
v_vert0 = -3; %m/s
% alphadot0 = v_vert0/Ls/cos(alpha0);
%% damper
global pIn xIn Aa Ab Kf Kp Lp_extended; 
pIn = .5e4; % 2 atm
xIn = 1; % m
Aa = pi*.3^2;
Ab = pi*.35^2;
Kf = 0;
Kp = 1e4; % 0.5 atm per 1 m/s of stroke compression speed;
Lp_extended =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha0));

%% differential equation
time_extremes = [0 2];
y0 = zeros(6,1);
%y is a coloumn vector holding the state of the system, so composed:
% 1 2 ) v_x/y rocket 3 4) x/y rocket
% 5) alpha_dot 6) alpha 
% 7) gamma_dot 8) gamma only if non-vertical
y0 = [0; v_vert0; 0; 4;];
opt = odeset('AbsTol',1e-3,'RelTol',1e-3);
[t,y] = ode45(@odeFunRocket3D,time_extremes,y0,opt);

%% debugging and visual - re-calculating everything 
N = length(t);
Lp=zeros(N,1);
alpha=Lp; alphadot=Lp; phi=Lp; Lpdot=Lp; Fp=Lp; pa=Lp; pb=pa; x=Lp; R=Lp;


for i=1:N
    alpha(i) = asin(y(i,4)/Ls);
    alphadot(i) = y(i,2)/Ls/cos(alpha(i));

    [cLp, cLpdot,cphi,cFp,cpa,cpb,cFsy,cFsx,cR,cRx,cx] = ...
                determineStateEvolution(t(i),y(i,:));
    Lp(i)=cLp; Lpdot(i)=cLpdot; phi(i)=cphi; Fp(i)=cFp; pa(i)=cpa; pb(i)=cpb; Fsy(i)=cFsy; Fsx(i)=cFsx; R(i)=cR; Rx(i)=cRx; x(i)=cx;
end
    
    



%%
Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
phi = acos((Lss^2-d^2-Lp.^2)./Lp/d/2); % cosine theorem: Lss^2 = d^2+Lp^2-2Lp*d*cos(phi)
Lpdot = d*Lss*cos(alpha)./Lp.*alphadot;
[Fp, pa, pb,x] = damperForce(Lp,Lpdot);
R = (-Fp).*(sin(phi-alpha))./(...
                    (cos(alpha)+sign(alphadot).*mu_din)*(1-(Ls-Lss)/Lss)...
            );


% FROM HERE one can call any visual debug function one might like


% IMPORTANT NOTE: X LIMIT FOR WHICH COMPUTATION EXPLODES IS ~5 to 0.1 mm
% (3-5 computation steps).
