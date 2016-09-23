%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

%
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
% determine flight mechanics residual
% x is the array with free parameters
function RES = get_rigid_body_res(x, ENABLE_DISP, NDOF)

TOTFM = 15;

global beam_model;
fid = beam_model.Param.FID;

RES = zeros(NDOF,1);

nc = beam_model.Aero.geo.nc;

% extract from 6 array x, all the parameters for trim
count = 0;

for n=1:TOTFM % flight mechanics parameters

	if ~beam_model.Res.FM.Fixed(n)
	
		count = count +1;
		beam_model.Res.FM.Value(n)	= x(count);
	end
end
% flight mechanics parameters
ROLL   = beam_model.Res.FM.Value(4); 
PITCH  = beam_model.Res.FM.Value(5); 
YAW    = beam_model.Res.FM.Value(6); 
%
PSI   = beam_model.Res.FM.Value(13);
TETA  = beam_model.Res.FM.Value(14);
PHI   = beam_model.Res.FM.Value(15);
%
R = Rrotmat(PSI, TETA, PHI);
VB = beam_model.Aero.state.AS.*[cos(TETA) 0 sin(TETA)]'; % rotate only to account for climb
OMEGAINER = [ROLL; PITCH; YAW];
OMEGAB = R*OMEGAINER;
%
for n=1:nc % master control surfaces parameters

	if ~beam_model.Res.CS.Fixed(n)

		count = count +1;
		beam_model.Res.CS.Value(n) = x(count);
	end

end

if count ~= NDOF
	error('Unable to set %d free parameters in flight mechanics residual evaluation.', NDOF);
end

% apply constraint equation to slave surfaces
beam_model.Res.CS.Value = apply_constr_eq(nc, beam_model.Res.CS.Fixed, beam_model.Aero.Trim.CS.MPC, beam_model.Aero.Trim.CS.Coeff, beam_model.Res.CS.Value);

% extract flight mechanics values
% tornado state
[beam_model.Res.state.alpha, beam_model.Res.state.betha, beam_model.Res.state.P, beam_model.Res.state.Q, beam_model.Res.state.R] = ...
get_state_trim_vars(beam_model.Res.FM); 
% give body angular velocities to VLM
beam_model.Res.state.P = OMEGAB(1);
beam_model.Res.state.Q = OMEGAB(2);
beam_model.Res.state.R = OMEGAB(3);
% update tornado mesh
% rotate control surfaces and update wake according to the new state
beam_model.Aero.lattice_defo = rotate_control_surf(beam_model.Aero.ref, beam_model.Res.state, beam_model.Aero.geo, ...
                                                   beam_model.Aero.lattice, beam_model.Res.CS.Value, ...
                                                   beam_model.Aero.lattice.Control.Hinge);
% solve aerodynamic forces
results = run_vlm_solver(beam_model.Res.state, beam_model.Aero.geo, beam_model.Aero.lattice_defo);
beam_model.Res.Aero.results = results;

% determine thrust force and moments
T_PERC = beam_model.Res.FM.Value(1);

F_thrust = zeros(3,1);
M_thrust = zeros(3,1);
F0 = zeros(3,1);
M0 = zeros(3,1);

for n = 1:beam_model.Info.nflw

	if (beam_model.F_FLW.ID(n) == beam_model.Param.LOAD)

		F0 = (T_PERC * beam_model.F_FLW.Mag(n)) .* beam_model.F_FLW.Orient(n, 1:3)';
    M0 = (crossm(beam_model.Node.Coord(beam_model.F_FLW.Node(n), 1:3) + ...
          beam_model.F_FLW.Offset(n,1:3) - beam_model.WB.CG) * F0);
		F_thrust = F_thrust + F0;
		M_thrust = M_thrust + M0;
	end
end

Fx = results.FORCES(1)  + F_thrust(1); 
Fy = results.FORCES(2)  + F_thrust(2);
Fz = results.FORCES(3)  + F_thrust(3);
% get moments from aircraft center of gravity
Mx = results.MOMENTS(2,1)  + M_thrust(1);
My = results.MOMENTS(2,2)  + M_thrust(2);
Mz = results.MOMENTS(2,3)  + M_thrust(3);
% inertial properties
M  = beam_model.WB.MCG(1,1);

Ixx = beam_model.WB.MCG(4,4);
Iyy = beam_model.WB.MCG(5,5);
Izz = beam_model.WB.MCG(6,6);
%
Ixy = beam_model.WB.MCG(4,5);
Ixz = beam_model.WB.MCG(4,6);
Iyz = beam_model.WB.MCG(5,6);
%
Iyx = beam_model.WB.MCG(5,4);
Izx = beam_model.WB.MCG(6,4);
Izy = beam_model.WB.MCG(6,5);

% determine residuals
G = beam_model.Param.G;
if (NDOF==2)
  % x equation NOT SOLVED
  %
  % z equation
  RES(1) = Fz - M * (G * cos(TETA) * cos(PHI) - OMEGAB(1)*VB(2) + OMEGAB(2)*VB(1));
  % y mom equation
  RES(2) = My + Izx * (OMEGAB(3)^2 - OMEGAB(1)^2) + (Izz-Ixx)*OMEGAB(3)*OMEGAB(1);

  if (ENABLE_DISP)
    fprintf('\n-- Residuals: --\n');
    fprintf(fid, '\n\tResidual norm: %.6e.', norm(RES));
    fprintf(fid, '\n\tZ Eq.:         %.6e.', RES(1)); 
    fprintf(fid, '\n\tY mom. Eq.:    %.6e.', RES(2)); 
  end

else
  % x equation NOT SOLVED
  %
  % y equation
  RES(1) = Fy - M * (G * cos(TETA) * sin(PHI) - OMEGAB(3)*VB(1) + OMEGAB(1)*VB(3));
  % z equation
  RES(2) = Fz - M * (G * cos(TETA) * cos(PHI) - OMEGAB(1)*VB(2) + OMEGAB(2)*VB(1));
  % x mom equation
  RES(3) = Mx - Izx * OMEGAB(1) * OMEGAB(2)       - (Iyy-Izz)*OMEGAB(2)*OMEGAB(3);
  % y mom equation
  RES(4) = My + Izx * (OMEGAB(3)^2 - OMEGAB(1)^2) + (Izz-Ixx)*OMEGAB(3)*OMEGAB(1);
  % z mom equation
  RES(5) = Mz + Izx * OMEGAB(2)*OMEGAB(3)         - (Ixx-Iyy)*OMEGAB(1)*OMEGAB(2);

  if (ENABLE_DISP)
    fprintf('\n-- Residuals: --\n');
    fprintf(fid, '\n\tResidual norm: %.6e.', norm(RES));
    fprintf(fid, '\n\tY Eq.:         %.6e.', RES(1)); 
    fprintf(fid, '\n\tZ Eq.:         %.6e.', RES(2)); 
    fprintf(fid, '\n\tX mom. Eq.:    %.6e.', RES(3)); 
    fprintf(fid, '\n\tY mom. Eq.:    %.6e.', RES(4)); 
    fprintf(fid, '\n\tZ mom. Eq.:    %.6e.', RES(5)); 
  end

end
%
% Output current solution
%
if (ENABLE_DISP)
  fprintf(fid, '\n\n-- Trim variables: --\n');
  fprintf(fid, '\n\tThrust percentage: %f.',   beam_model.Res.FM.Value(1));
  fprintf(fid, '\n\tClimb angle:       %f deg.',   rad2deg(beam_model.Res.FM.Value(14)));
  fprintf(fid, '\n\tBank angle:        %f deg.',   rad2deg(beam_model.Res.FM.Value(15)));
  %
  fprintf(fid, '\n\tAlpha:             %f deg.',   rad2deg(beam_model.Res.FM.Value(2)));
  fprintf(fid, '\n\tBeta:              %f deg.',   rad2deg(beam_model.Res.FM.Value(3)));
  fprintf(fid, '\n\tRoll rate:         %f [-].',   OMEGAB(1)*beam_model.Aero.ref.b_ref/(2*beam_model.Aero.state.AS));
  fprintf(fid, '\n\tPitch rate:        %f [-].',   OMEGAB(2)*beam_model.Aero.ref.C_mgc/(2*beam_model.Aero.state.AS));
  fprintf(fid, '\n\tYaw rate:          %f [-].',   OMEGAB(3)*beam_model.Aero.ref.b_ref/(2*beam_model.Aero.state.AS));
  %
  for ns = 1:length(beam_model.Res.CS.Value)
    fprintf(fid, '\n\tControl %d:         %f deg.',   ns, rad2deg(beam_model.Res.CS.Value(ns))); 
  end
  %
end

end
%***********************************************************************************************************************
function R = Rrotmat(PSI, TETA, PHI)

  R = [cos(TETA)*cos(PSI) cos(TETA)*sin(PSI) -sin(TETA);
       sin(PHI)*sin(TETA)*cos(PSI)-cos(PHI)*sin(PSI) sin(PHI)*sin(TETA)*sin(PSI)+cos(PHI)*cos(PSI) sin(PHI)*cos(TETA)
       cos(PHI)*sin(TETA)*cos(PSI)+sin(PHI)*sin(PSI) cos(PHI)*sin(TETA)*sin(PSI)-sin(PHI)*cos(PSI) cos(PHI)*cos(TETA)];
end
