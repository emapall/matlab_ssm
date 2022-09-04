% struttura materiali [refs: lezione 3, pdf viti, MIL-HDBK-5H]
% campi:
% .name: nome materiale 
% .form: available forms 
% .properties: array di proprieta', quella associata ad ogni indice e':
%   (1) density [lbm/in^3]
%   (2) Ftu [ksi]
%   (3) Fty [ksi]
%   (4) Fsu [ksi]
%   (5) Fcy [ksi]
%   (6) E [10^3 ksi]
%   (7) epsilon [%] allungamento percentuale a rottura (misura duttilita')   
%
% coverison factors:
% 1 ksi = 6.89476 MPa
% 1 lbm/in^3 = 27679.9047 kg/m^3

%% steel
i=1;
materials(i).name='Steel_4340 - heat treated'     ;
materials(i).form='all wrought forms';
materials(i).properties=[.283 200 176 119 198 29 10];

i=i+1;
materials(i).name='Steel_AISI 301 - annealed';
materials(i).form='plate,sheet,strip';
materials(i).properties=[.286 150 110 40 58 26 18];

%% aluminium

i=i+1;
materials(i).name='Aluminium_2024 - T4';
materials(i).form='bar,rod,plate';
materials(i).properties=[.100 57 42 30 38 10.7 10];

i=i+1;
materials(i).name='Aluminium_7075 - T6';
materials(i).form='bar,rod,plate';
materials(i).properties=[.101 76 64 45 71 10.5 7];

%% titanium

i=i+1;
materials(i).name='Titanium_6Al-4V - heat treated';
materials(i).form='plate,sheet';
materials(i).properties=[.160 157 143 95 152 16.4 10];

i=i+1;
materials(i).name='Titanium_6Al-4V - heat treated';
materials(i).form='bar,rod';
materials(i).properties=[.160 150 140 90 145 16.9 10];

clear i