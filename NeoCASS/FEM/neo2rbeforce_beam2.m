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
%       ORIG: coordinate of beam spline origin
%       END: coordinate of beam spline end.
%       beam spline goes from ORIG to END
%       LID: load set ID (used for nastran)
%       EXP_ACC: 1 to export GRAV and RFORCE cards with LID+1 and LID+2 IDs.
%       If calling this function for multiple spline segments, use EXP_ACC = 1 the first time only
%	Output:        
% 		File ASCII file
%
%     If the wing box axis changes, use this faction for each wing sector (for example if the wing has a winglet,
%     use the coordinates from its beam model)
%
%	Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
%
function [FSG, MSG, FSL, MSL] = neo2rbeforce_beam2(beam_model, filename, str_id, str_data, ...
                                                   CAERO_ID, toll, ORIG, END, LID, EXP_ACC, silentMode)

if ~exist('silentMode', 'var') || isempty(silentMode)
	silentMode = false;
end


% Recover aero forces along str nodes
fprintf('\nBulding aero mesh and forces...');
aero_data = [];
DOF = [];
for k=1:length(CAERO_ID)
  index = find(beam_model.Aero.ID == CAERO_ID(k));
  n1 = beam_model.Aero.lattice_vlm.DOF(index,1,1);
  n2 = beam_model.Aero.lattice_vlm.DOF(index,1,2);
  DOF = [DOF, [n1:n2]];
  for j=n1:n2
    coord = (beam_model.Aero.lattice_vlm.VORTEX([j],[4],:) + beam_model.Aero.lattice_vlm.VORTEX([j],[5],:)).*0.5;
    aero_data = [aero_data; [coord(:,:,1), coord(:,:,2), coord(:,:,3)]];
  end
end
nstr = size(str_data,1);
naer = size(aero_data,1);

[~, ordering] = sort(str_data(:,2));
str_id = str_id(ordering);
str_data = str_data(ordering,:);

%
ACC = [];

switch beam_model.Res.SOL
case 'Static linear unrestrained trim'
  % solve_free_lin_trim

  CREF = beam_model.Aero.ref.C_mgc;
  BREF = beam_model.Aero.ref.b_ref;
  SREF = beam_model.Aero.ref.S_ref;
  VREF2 = 2*beam_model.Aero.state.AS;
  UX(1) = beam_model.Res.Aero.RTrim_sol.Alpha * pi/180;
  UX(2) = beam_model.Res.Aero.RTrim_sol.Betha * pi/180;
  UX(3) = beam_model.Res.Aero.RTrim_sol.P * CREF/VREF2;
  UX(4) = beam_model.Res.Aero.RTrim_sol.Q * BREF/VREF2;
  UX(5) = beam_model.Res.Aero.RTrim_sol.R * BREF/VREF2;
  ACC = beam_model.Res.Aero.RTrim_sol.ACC;
  Ftot = beam_model.Res.CPaero.F0;
  for k=1:5
    Ftot(:,1) = Ftot(:,1) + beam_model.Res.CPaero.State(1:3:end,k).* UX(k);
    Ftot(:,2) = Ftot(:,2) + beam_model.Res.CPaero.State(2:3:end,k).* UX(k);
    Ftot(:,3) = Ftot(:,3) + beam_model.Res.CPaero.State(3:3:end,k).* UX(k);
  end
  for k=1:length(beam_model.Res.Aero.RTrim_sol.Control)
    Ftot(:,1)=   Ftot(:,1) + beam_model.Res.CPaero.Control(1:3:end,k).* beam_model.Res.Aero.RTrim_sol.Control(k) * pi/180;
    Ftot(:,2)=   Ftot(:,2) + beam_model.Res.CPaero.Control(2:3:end,k).* beam_model.Res.Aero.RTrim_sol.Control(k) * pi/180;
    Ftot(:,3)=   Ftot(:,3) + beam_model.Res.CPaero.Control(3:3:end,k).* beam_model.Res.Aero.RTrim_sol.Control(k) * pi/180;
  end

  % Take only selected panels
  F0 = Ftot(DOF,:);

case 'Static linear unrestrained trim rigid'
  % solve_free_lin_trim_guess_std

  F0 = beam_model.Res.Aero.results.F(DOF,:);
  CREF = beam_model.Aero.ref.C_mgc;
  BREF = beam_model.Aero.ref.b_ref;
  SREF = beam_model.Aero.ref.S_ref;
  VREF2 = 2*beam_model.Aero.state.AS;
  UX(1) = beam_model.Res.Aero.RTrim_sol.Alpha * pi/180;
  UX(2) = beam_model.Res.Aero.RTrim_sol.Betha * pi/180;
  UX(3) = beam_model.Res.Aero.RTrim_sol.P * CREF/VREF2;
  UX(4) = beam_model.Res.Aero.RTrim_sol.Q * BREF/VREF2;
  UX(5) = beam_model.Res.Aero.RTrim_sol.R * BREF/VREF2;
  ACC = beam_model.Res.Aero.RTrim_sol.ACC;

case 'Static linear aerodynamic'
  % solve_vlm_rigid

  F0 = beam_model.Res.Aero.results.F(DOF,:);

otherwise
  error('Unable to recover aero forces. Solver unknown.');
end
fprintf('done.');
%


% Get interpolation matrix

Y = [0,1,0];% END - ORIG; Y = Y ./ norm(Y);
X = [1 0 0];
Z = (crossm(X) * Y')'; Z = Z ./ norm(Z);
X = (crossm(Y) * Z')'; X = X ./ norm(X);
Rmat = [X; Y; Z];
H = beam_interface2(str_data, aero_data, toll, ORIG, Rmat);
%


for i=1:naer
  F0R(i,1:3) = (Rmat * F0(i,:)')';
end

% Interpolate forces
Fz = [F0R(:,3); zeros(2*naer,1)];
STRF = H' * Fz;
interpolatedForces = reshape(STRF, [nstr,3]);





fp = fopen(filename, 'w');
if (EXP_ACC)
  fprintf(fp,'%8s%8d%8.4f%8.4f%8d','LOAD    ',LID,1.0,1.0,LID*100);
  if (~isempty(ACC))
    fprintf(fp,'%8.4f%8d%8.4f%8d\n',1.0,LID*100+1,1.0,LID*100+2);
    fprintf(fp,'$\n$ Maneuver inertial loads\n$\n');
    fprintf(fp,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
    linacc = norm(ACC(1:3));
    angacc= -norm(ACC(4:6))/2/pi;
    omega = norm(UX(3:5))/2/pi;
    CENTER =   beam_model.Aero.geo.ref_point;
    CENTER_ID = max(str_id)+1;
    fprintf(fp,'%8s%8d%8d%8.4f%8.4f%8.4f%8d%8d\n','GRID    ',CENTER_ID,0,CENTER(1),CENTER(1),CENTER(3),0,123456);
    fprintf(fp,'%8s%8d%8d%8.4f%8.4f%8.4f%8.4f\n', 'GRAV    ',LID*100+1,0,linacc,-ACC(1)/linacc,-ACC(2)/linacc,-ACC(3)/linacc);
    fprintf(fp,'%8s%8d%8d%8d%8.4f%8.4f%8.4f%8.4f\n',   'RFORCE  ',LID*100+2,CENTER_ID,0,omega,1.,0.,0.);
    fprintf(fp,'        %8.4f\n',angacc);
    fprintf(fp,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
  end
end
%export forces and moments 
FSG = zeros(nstr,3);
MSG = zeros(nstr,3);
FSL = zeros(nstr,1);
MSL = zeros(nstr,2);
for i=1:nstr
%
  FSL(i) = STRF(i);
  FORCE = (Rmat')* [0 0 STRF(i)]';
  FSG(i,:) = FORCE;
  magf = norm(FORCE);
  if (magf>eps)
    fprintf(fp, 'FORCE,%d,%d,,%f,%f,%f,%f\n',  LID*100, str_id(i), magf, FORCE(1)/magf,...
                 FORCE(2)/magf, FORCE(3)/magf);
  end
  MSL(i,1) = STRF(i+nstr);
  MSL(i,2) = STRF(i+2*nstr);
  MOM = (Rmat')*[STRF(i+nstr) STRF(i+2*nstr) 0]';
  MSG(i,:) = MOM;
  magf = norm(MOM);
  if (magf>eps)
    fprintf(fp, 'MOMENT,%d,%d,,%f,%f,%f,%f\n', LID*100, str_id(i), magf, MOM(1)/magf,...
                 MOM(2)/magf, MOM(3)/magf);
  end
%
end
%
fclose(fp);
%


if ~silentMode
  % Check spline
  
  alpha = 0.1;
  beta = 0;
  Xs = str_data - ones(nstr,1)*str_data(1,:);
  Us = [ ...
            [  -beta*Xs(:,3),         zeros(nstr,1), alpha*Xs(:,2).^2 + beta*Xs(:,1)], ...
            [  alpha*Xs(:,2), - beta*ones(nstr,1),                     zeros(nstr,1)]  ...
       ];
  
  dispA =  H  * reshape(Us(:,[3,4,5]), [3*nstr,1]);
  
  %----------------------------
  % plot geometry
  figure(1); close; figure(1),
  plot3(str_data(:,1), str_data(:,2), str_data(:,3), 'ks'); 
  hold on;
  plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3), 'ro');
  axis equal
  grid on
  
  % test displacement
  figure(2); close; figure(2); hold on;
  plot3(str_data(:,1), str_data(:,2), str_data(:,3),       'bs'); 
  plot3(str_data(:,1)+Us(:,1), str_data(:,2)+Us(:,2), str_data(:,3)+Us(:,3), 'ks');
  
  plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3)+dispA(1:naer), 'r.');
  axis equal;
  grid on
  title('Dummy interpolation test');
  
  %----------------------------
  % test force
  figure(3); close; figure(3); 
  plot3(str_data(:,1), str_data(:,2), str_data(:,3), 'ks');
  hold on;
  plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3), 'k.');
  scale = 1;
  maxf = max(STRF(1:nstr,1));
  minf = min(STRF(1:nstr,1));
  maxf = max(abs(maxf), abs(minf));
  maxfa = max(F0(1:naer,3));
  minfa = min(F0(1:naer,3));
  maxfa = max(abs(maxfa), abs(minfa));
  plot3(str_data(:,1), str_data(:,2), str_data(:,3)+FSG(1:nstr,3)./maxf*scale, 'rs');
  plot3(aero_data(:,1), aero_data(:,2), aero_data(:,3)+F0(:,3)./maxfa*scale, 'r.');
  axis equal;
  title('Force interpolation test (scaled)');
  %
  figure(4); close; figure(4);
  plot3(str_data(:,1),str_data(:,2), str_data(:,3) + FSG(1:nstr,3), 'ks'); 
  hold on;
  plot3(aero_data(:,1),aero_data(:,2), aero_data(:,3)+F0(:,3), 'r.');
  title('Force interpolation test');
  fprintf('\nTotal vertical force along structural mesh: %g [N].', sum(FSG(1:nstr,3)));
  fprintf('\nTotal vertical force along aero mesh: %g [N].\n', sum(F0(:,3)));
  %
  figure(5); close; figure(5);
  plot3(str_data(:,1),str_data(:,2), str_data(:,3) + MSG(1:nstr,1), 'ks'); 
  hold on;
  plot3(aero_data(:,1),aero_data(:,2), aero_data(:,3), 'r.');
  title('Bending interpolation test');
  %
  figure(6); close; figure(6);
  plot3(str_data(:,1),str_data(:,2), str_data(:,3) + MSG(1:nstr,2), 'ks'); 
  hold on;
  plot3(aero_data(:,1),aero_data(:,2), aero_data(:,3), 'r.');
  title('Torque interpolation test [Nm]');
  %
  rcoord = zeros(nstr,3);
  for i=1:nstr
    rcoord(i,:) = (Rmat* (str_data(i,:)-ORIG)')';
  end
  % figure(7); close; figure(7);
  % plot(rcoord(:,2), cumsum(FSL));
  % title('Shear along local axis [Nm]  (from root)');
  %
  figure(8); close; figure(8);
  mtr = zeros(nstr,1);
  for i=1:nstr
    mtr(i) = sum(MSL(1:i,1)) + sum(FSL(1:i).*(rcoord(1:i,2) - rcoord(i,2)));
  end
  plot(rcoord(:,2), mtr);
  title('Bending along local axis [Nm] (from root)');
  %
  figure(9); close; figure(9);
  plot(rcoord(:,2), cumsum(MSL(:,2)));
  title('Torque along local axis (from root)');
  %------
  figure(10); close; figure(10);
  plot(rcoord(end:-1:1,2), cumsum(FSL(end:-1:1)));
  title('Shear along local axis [Nm] (from tip)');
  %
  figure(11); close; figure(11);
  mtr = zeros(nstr,1);
  for i=nstr:-1:1
    mtr(i) = sum(-MSL(end:-1:i,1)) + sum(FSL(end:-1:i).*(-rcoord(end:-1:i,2) + rcoord(i,2)));
  end
  plot(rcoord(:,2), mtr);
  title('Bending along local axis [Nm] (from tip)');
  %
  figure(12); close; figure(12);
  plot(rcoord(end:-1:1,2), cumsum(MSL(end:-1:1,2)));
  title('Torque along local axis (from tip)');
end

end
