%
%***********************************************************************************************************************
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%  Neo2RBEForce conservative load interpolation using BEAM2 spline
%   
%	Usage:
%
%	Input:
%
%       beam_model struct
%
%       filename output filename with Nastran FORCE and MOMENT cards
%       str_id structural node labels
%		    str_data	matrix with the structural nodes space position [Node_number x Dimension]
%                   [ x1 y1 z1;
%                     x2 y2 z2;
%                   ...]
%       CAERO_ID list with IDs of CAERO
%       toll 	maximum condition number accepted for the system matrix. 
%      	Useful for ill-conditioned problems (typical value 1.0E+5)
%
%	Output:        
% 		File ASCII file 
%
%	Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
%
function [FSG, MSG, FSL, MSL] = edge2rbeforce_beam2(filename, str_id, str_data, aero_id, aero_data, aero_force, toll, ORIG, END)
%
naer = size(aero_data,1);
%
F0 = aero_force;
%
Y = END - ORIG; Y = Y ./ norm(Y);
X = [1 0 0];
Z = (crossm(X) * Y')'; Z = Z ./ norm(Z);
X = (crossm(Y) * Z')'; X = X ./ norm(X);
Rmat = [X; Y; Z];
H = beam_interface2(str_data, aero_data, toll, ORIG, Rmat);
%
for i=1:naer
  F0R(i,1:3) = (Rmat * F0(i,:)')';
end
Fz = [F0R(:,3); zeros(2*naer,1)];
% Interpolate forces
STRF = H' * Fz;
strN = size(str_data,1);
dispS = str_data(:,2);
dispA = H * [dispS; zeros(2*strN,1)];
%----------------------------
figure(1); close; figure(1),
plot3(str_data(:,1), str_data(:,2), str_data(:,3), 'ks'); 
hold on;
plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3), 'ro');
axis equal
% test displacement
figure(2); close; figure(2); 
plot3(str_data(:,1), str_data(:,2), str_data(:,3), 'ks'); 
hold on;
plot3(str_data(:,1), str_data(:,2), str_data(:,3)+dispS, 'ks');
plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3)+dispA(1:naer), 'r.');
axis equal;
title('Dummy interpolation test');
%
fp = fopen(filename, 'w');
%export forces and moments 
LID = 1;
FSG = zeros(strN,3);
MSG = zeros(strN,3);
FSL = zeros(strN,1);
MSL = zeros(strN,2);
for i=1:strN
%
  FSL(i) = STRF(i);
  FORCE = (Rmat')* [0 0 STRF(i)]';
  FSG(i,:) = FORCE;
  magf = norm(FORCE);
  if (magf>eps)
    fprintf(fp, 'FORCE,%d,%d,,%f,%f,%f,%f\n',  LID, str_id(i), magf, FORCE(1)/magf,...
                 FORCE(2)/magf, FORCE(3)/magf);
  end
  MSL(i,1) = STRF(i+strN);
  MSL(i,2) = STRF(i+2*strN);
  MOM = (Rmat')*[STRF(i+strN) STRF(i+2*strN) 0]';
  MSG(i,:) = MOM;
  magf = norm(MOM);
  if (magf>eps)
    fprintf(fp, 'MOMENT,%d,%d,,%f,%f,%f,%f\n', LID, str_id(i), magf, MOM(1)/magf,...
                 MOM(2)/magf, MOM(3)/magf);
  end
%
end
%
fclose(fp);
%
%----------------------------
% test force
figure(3); close; figure(3); 
plot3(str_data(:,1), str_data(:,2), str_data(:,3), 'ks');
hold on;
plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3), 'k.');
scale = 1;
maxf = max(STRF(1:strN,1));
minf = min(STRF(1:strN,1));
maxf = max(abs(maxf), abs(minf));
maxfa = max(F0(1:naer,3));
minfa = min(F0(1:naer,3));
maxfa = max(abs(maxfa), abs(minfa));
plot3(str_data(:,1), str_data(:,2), str_data(:,3)+FSG(1:strN,3)./maxf*scale, 'rs');
plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3)+F0(:,3)./maxfa*scale, 'r.');
axis equal;
title('Force interpolation test (scaled)');
%
figure(4); close; figure(4);
plot3(str_data(:,1),str_data(:,2), str_data(:,3) + FSG(1:strN,3), 'ks'); 
hold on;
plot3(aero_data(:,1),aero_data(:,2), aero_data(:,3)+F0(:,3), 'r.');
title('Force interpolation test');
fprintf('\nTotal vertical force along structural mesh (local frame): %g [N].', sum(FSL));
fprintf('\nTotal vertical force along aero mesh: %g [N].\n', sum(Fz));
%
figure(5); close; figure(5);
plot3(str_data(:,1),str_data(:,2), str_data(:,3) + MSG(1:strN,1), 'ks'); 
hold on;
plot3(aero_data(:,1),aero_data(:,2), aero_data(:,3), 'r.');
title('Bending interpolation test');
%
figure(6); close; figure(6);
plot3(str_data(:,1),str_data(:,2), str_data(:,3) + MSG(1:strN,2), 'ks'); 
hold on;
plot3(aero_data(:,1),aero_data(:,2), aero_data(:,3), 'r.');
title('Torque interpolation test [Nm]');
%
rcoord = zeros(strN,3);
for i=1:strN
  rcoord(i,:) = (Rmat* (str_data(i,:)-ORIG)')';
end
figure(7); close; figure(7);
plot(rcoord(:,2), cumsum(FSL));
title('Shear along local axis [Nm]');
%
figure(8); close; figure(8);
mtr = zeros(strN,1);
for i=1:strN
  mtr(i) = sum(MSL(1:i,1)) + sum(FSL(1:i).*(rcoord(1:i,2) - rcoord(i,2)));
end
plot(rcoord(:,2), mtr);
title('Bending along local axis [Nm]');
%
figure(9); close; figure(9);
plot(rcoord(:,2), cumsum(MSL(:,2)));
title('Torque along local axis');
%
end
