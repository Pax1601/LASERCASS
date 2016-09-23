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
%                      Sergio Ricci           <ricci@aero.polimi.it>
%                      Luca Cavagna           <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari  <degaspari@aero.polimi.it>
%                      Luca Riccobene         <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
%
% Run this solver after a first solution from solve_free_lin_trim
%
function DALPHA = solve_free_lin_guess_std_gust(TRIM_INDEX, outp)

global beam_model;
%
fid = beam_model.Param.FID; 
Kax = beam_model.Res.Aero.Kax;
%
%-------------------------------------------------------------------------
%
%   select trim case
%
index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
%
fprintf(outp,'\n\nGUST MANEUVER\n');
NDOF = 1;
STRIM = 1; 
FMDOF = [3];
%
ngrid = beam_model.Info.ngrid;
nbar  =  beam_model.Info.nbar;
nbeam =  beam_model.Info.nbeam;
ndof  =  beam_model.Info.ndof;
MSRR = beam_model.WB.MRP;
%
%   GUST DATA
%
CREF = beam_model.Aero.ref.C_mgc;
SREF = beam_model.Aero.ref.S_ref;
cn_ALPHA = beam_model.Res.Aero.RStab_Der.Alpha.dcl_dalpha;
MASS = beam_model.WB.MCG(1,1); 
mu_g = 2 * MASS / (CREF*SREF*cn_ALPHA*beam_model.Aero.state.rho);
Kg = (0.88 * mu_g) / (5.3 + mu_g);
KEAS = sqrt(beam_model.Aero.state.rho/1.225);
%
VEAS = KEAS * beam_model.Aero.state.AS;
VG   = beam_model.Aero.Trim.Extra.Value(1);
DELTAN = 0.5 * 1.225 * (SREF/MASS) * cn_ALPHA * VEAS * Kg * VG;
%
% Determine trim unknows
%
% Access to previous acceleration values
UDD(:,1) = beam_model.Aero.Trim.FM.Value(7:12)';
% Update URDD3
UDD(FMDOF,1) = UDD(FMDOF,1) + DELTAN;
fprintf(fid,'\n\t\t - Flight speed (EAS):    %g [m/s].', VEAS);
fprintf(fid,'\n\t\t - Gust speed (EAS):      %g [m/s].', VG);
fprintf(fid,'\n\t\t - Attenuation factor:    %g [].', Kg);
fprintf(fid,'\n\t\t - Delta load factor:     %g [].', DELTAN/beam_model.Param.G);
fprintf(fid,'\n\t\t - Total load factor:     %g [].\n', UDD(FMDOF,1)/beam_model.Param.G);
% Load previous solution and fix all trim variables
beam_model.Aero.Trim.FM.Fixed(1:end) = 1;
% SET ANGLEA as free
beam_model.Aero.Trim.FM.Fixed(2) = 0; 
%
Z1ZX_FM = Kax(:,1:5);
%   find ANGLEA
TMAT = -Z1ZX_FM(FMDOF,1); % alpha term wrt plunge
DRHS = -MSRR(:, FMDOF) .* DELTAN; % DELTA N * MASS = DELTA_LIFT
sol = DRHS(FMDOF)/TMAT;
DALPHA = sol;
%   update previous solution
beam_model.Res.FM.Value(1, 2) = beam_model.Res.FM.Value(1, 2) + sol;
beam_model.Res.FM.Value(1, 9) = UDD(FMDOF,1);
beam_model.Res.Aero.RTrim_sol.ACC = UDD(:,1);
%
print_trim_sol(fid, beam_model.Res.FM.Value(1, 9),beam_model.Res.FM.Value(1, 2));
fprintf(outp, '\n\nCORRECTIONS TO RIGID TRIM\n');
print_trim_sol(outp, beam_model.Res.FM.Value(1, 9),beam_model.Res.FM.Value(1, 2));
beam_model.Res.Aero.RTrim_sol.Alpha = beam_model.Res.FM.Value(1, 2) * 57.3;
%
end		
%***********************************************************************************************************************
function print_trim_sol(fid, UDD, AOA)

  fprintf(fid,'\n\t\t - Z acc:      %g [m/s^2].', UDD);
  fprintf(fid,'\n\t\t - Alpha:      %g [deg].', AOA*180/pi);
 
end
