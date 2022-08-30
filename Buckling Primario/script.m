% symbolic variables initialization
clear all;
close all;

Li = sym('L_i','positive');
L1 = sym('L_1','positive');
L2 = sym('L_2','positive');
L3 = sym('L_3','positive');
L4 = sym('L_4','positive');

ki = sym('k_i','positive');
k1 = sym('k_1','positive');
k2 = sym('k_2','positive');
k3 = sym('k_3','positive');
k4 = sym('k_4','positive');

t1 = sym('tau_1','positive');
t2 = sym('tau_2','positive');
t3 = sym('tau_3','positive');

g = sym('gamma','positive');
gi = sym('gamma_i','positive');
g1 = sym('gamma_1','positive');
g2 = sym('gamma_2','positive');
g3 = sym('gamma_3','positive');

% At first we write the BC mixed with the displacement CE

BCMatrix=[1, 0, 0, 0, 0, 0, 0, 0
    cos(k1*L1), sin(k1*L1), -1, 0, 0, 0, 0, 0
    0, 0, cos(k2*L2), sin(k2*L2), -1, 0, 0, 0
    0, 0, 0, 0, cos(k3*L3), sin(k3*L3), 0, -1
    0, 0, 0, 0, 0, 0, L4, 1]

%%
% we write the inclination CE

% afret soving discontinuity equations in terms of v_i e v_i', i substitute
% the general expression and group the same coefficients, which multiply
% the following functions:

fAi=-ki*sin(ki*Li)-ki^2*gi*cos(ki*Li);
fBi=ki*cos(ki*Li)-ki^2*gi*sin(ki*Li);

fA11=subs(fAi,[ki Li gi],[k1 L1 g1]);
fB11=subs(fBi,[ki Li gi],[k1 L1 g1]);
fA21=k2*sin(k2*L2);
fB21=-k2*cos(k2*L2);

fA22=subs(fAi,[ki Li gi],[k2 L2 g2]);
fB22=subs(fBi,[ki Li gi],[k2 L2 g2]);
fA32=k3*sin(k3*L3);
fB32=-k3*cos(k3*L3);

fA33=subs(fAi,[ki Li gi],[k3 L3 g3]);
fB33=subs(fBi,[ki Li gi],[k3 L3 g3]);

CEMatrix=[fA11 fB11 fA21 fB21 0 0 0 0
    0 0 fA22 fB22 fA32 fB32 0 0
    0 0 0 0 fA33 fB33 -1 0]
%%
% so, we have the matrix we want to find the determinant of:

M=[BCMatrix;CEMatrix];

% e mo famo ildeterminante

detM= det(M);
detM=simplify(detM)
g = 1;
%%
detM=subs(detM,[g1 g2 g3],[g g g])
L1=2.5 %[m]

rho=0.9
DL = L1/10

L2n = L1-DL
L3n = L1-2*DL
L4n = 2*L1 -7*DL

k2n = k1*rho^(-1/2)
k3n = k1*rho^(-1)


% detM=subs(detM,[L2 L3 L4 k2 k3],[L2n L3n L4n k2n k3n])
% detM=subs(detM,L2,L1-DL)
detM=subs(detM,"L_1",L1)
detM=subs(detM,L2,L2n)
detM=subs(detM,L3,L3n)
detM=subs(detM,L4,L4n)
detM=subs(detM,k2,k2n)
detM=subs(detM,k3,k3n)

%%
k_1 = 0:0.01:1
detEval = eval(subs(detM,"k_1",k_1));
plot(k_1,detEval);