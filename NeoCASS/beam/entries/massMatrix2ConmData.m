function [M, offset, Icg] = massMatrix2ConmData(massMatrix);
%
%
%
%-------------------------------------------------------------------------------
% 19-04-2016
%





% Total mass
M = massMatrix(1,1);


% Static unbalance
Svec = [massMatrix(2,6); massMatrix(3,4); massMatrix(1,5)];

% Offset position
offset = Svec/M;

% Inertia matrix at the mass center of gravity
Icg(1) = massMatrix(4,4) - M*(offset(2)^2 + offset(3)^2);
Icg(3) = massMatrix(5,5) - M*(offset(3)^2 + offset(1)^2);
Icg(6) = massMatrix(6,6) - M*(offset(1)^2 + offset(2)^2);

Icg(2) = - ( massMatrix(5,4) + M*offset(1)*offset(2) );
Icg(4) = - ( massMatrix(6,4) + M*offset(1)*offset(3) );
Icg(5) = - ( massMatrix(6,5) + M*offset(3)*offset(2) );

Icg = Icg.*( abs(Icg>1e-10));















return
