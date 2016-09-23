function [P1, P2, c1, c2] = CAERO_neo2nas(DIH, SPN, CHD, CX, CY, CZ, TPR, SWP);
%
% DIH : angolo tra il piano x-y (piano orizzontale) e il piano della superficie
%       aerodinamica ( piano contenente l'asse x e passante per i punti P1 e P2)
%       angolo misurato in gradi
%
% SPN : apertura misurata nel piano della superficie aerodinamica
%
% TPR : rapporto corda di estremità - corda di radice
%
% SWP : angolo di freccia al 25% della corda, misurato nel piano della 
%       superficie aerodinamica (angolo misurato in gradi)

%
%
%-------------------------------------------------------------------------------
% 15-04-2016
%


P1 = [CX; CY; CZ];
c1 = CHD;

c2 = c1*TPR;

P2 = zeros(3,1);

P2(1) = P1(1) + SPN*tan(SWP*pi/180) + 0.25*(c1-c2);
P2(2) = P1(2) + SPN*cos(DIH*pi/180);
P2(3) = P1(3) + SPN*sin(DIH*pi/180);




return
