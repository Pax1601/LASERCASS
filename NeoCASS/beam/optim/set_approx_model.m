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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080308      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
%
function upd_beam_model = set_approx_model(beam_model, varargin)

fid = beam_model.Param.FID;
% restart from a previous solution
if nargin > 1
  beam_model.Optim.X = varargin{1};
else
  % set DESVAR variables call for user defined function
  if (~isempty(beam_model.Param.DESVAR))
      
     if (~isempty(find(beam_model.Param.MSOL == 145,1)))
          fprintf(fid, '\n\tSetting design variables...');
         [beam_model.Optim.Desvar.Name, beam_model.Optim.X0, beam_model.Optim.XL,  beam_model.Optim.XU, ...
             beam_model.Optim.Cstr.In.Name, beam_model.Optim.Cstr.In.Value, ...
             beam_model.Optim.Cstr.Eq.Name, beam_model.Optim.Cstr.Eq.Value, ...
             beam_model.Optim.Cstr.In.CT, beam_model.Optim.Cstr.Eq.CT,...
             beam_model.Param.follow, beam_model.Param.curve] = beam_model.Param.DESVAR();
         fprintf(fid, 'done.');
         
         beam_model.Optim.Desvar.n_desvar = length(beam_model.Optim.Desvar.Name);
         beam_model.Optim.Cstr.n_in = length(beam_model.Optim.Cstr.In.Name);
         beam_model.Optim.Cstr.n_eq = length(beam_model.Optim.Cstr.Eq.Name);
         
     else
         fprintf(fid, '\n\tSetting design variables...');
         [beam_model.Optim.Desvar.Name, beam_model.Optim.X0, beam_model.Optim.XL,  beam_model.Optim.XU, ...
             beam_model.Optim.Cstr.In.Name, beam_model.Optim.Cstr.In.Value, ...
             beam_model.Optim.Cstr.Eq.Name, beam_model.Optim.Cstr.Eq.Value, ...
             beam_model.Optim.Cstr.In.CT, beam_model.Optim.Cstr.Eq.CT] = beam_model.Param.DESVAR();
         fprintf(fid, 'done.');
         
         beam_model.Optim.Desvar.n_desvar = length(beam_model.Optim.Desvar.Name);
         beam_model.Optim.Cstr.n_in = length(beam_model.Optim.Cstr.In.Name);
         beam_model.Optim.Cstr.n_eq = length(beam_model.Optim.Cstr.Eq.Name);
     end
  else
    error('No design variable function given.');
  end
  % Set DESVAR initial solution
  beam_model.Optim.X = beam_model.Optim.X0; 
%
%-------------------------------------------------------------------------------
% set normalization factors to have unitary design variables
% and improve optimization process
  fprintf(fid, '\n\tNormalizing design variables...');
  for k = 1:length(beam_model.Optim.Desvar.Name)
    beam_model.Optim.Xnorm(k) = eval(beam_model.Optim.Desvar.Name{k});
    fprintf(fid, '\n\t\t - %s: %.3e',...
            beam_model.Optim.Desvar.Name{k}, beam_model.Optim.Xnorm(k));
  end
  fprintf(fid, '\n\tdone.');
end
%-------------------------------------------------------------------------------
%
% Update model with DESIGN VARIABLES VALUE
%
beam_model = update_model_desvar(beam_model, beam_model.Optim.X);
%-------------------------------------------------------------------------------
%
% Run user defined function to execute links for approximate model
%
if (~isempty(beam_model.Param.DLINK))
  beam_model = beam_model.Param.DLINK(beam_model);
else
  fprintf(fid, '\n\tWarning: no approximate link function given.');
end
%-------------------------------------------------------------------------------
%
% Update all mechanical properties
%
  if (beam_model.Info.npbarsm)
%   assembly missing data for user defined section properties
    beam_model.PBar = smonoq1_fuse_stiff(fid, beam_model.PBar);    % unframed fuselage from GUESS
    beam_model.PBar = smonoq2_fuse_stiff(fid, beam_model.PBar);    % framed fuselage from GUESS
%
%   lifting surfaces
    beam_model.PBar = smonoq_mweb_stiff(fid, beam_model.PBar);     % multi web box beam
%
    beam_model.PBar = smonoq1_stiff(fid, beam_model.PBar);         % bi-symmetric wing-box
    beam_model.PBar = smonoqwb1_stiff(fid, beam_model.PBar, 1);    % bi-symmetric wing-box with stringers
  end
% update bar stifness and mass matrix
  beam_model.Bar = set_cbar_database(beam_model.Info.nbar, beam_model.Bar, ...
                         beam_model.PBar, beam_model.Mat, beam_model.Node, ...
                         beam_model.Coord);
% update lumped masses
%
% NOT AVAILABLE YET
%

% run WB
beam_model.WB.CG = zeros(1,3);     % center of gravity coords
beam_model.WB.MCG = zeros(6,6);    % mass matrix 6x6 collocated in cg
beam_model.WB.MRP = zeros(6,6);    % mass matrix 6x6 collocated in the grid point required by the user
beam_model.WB.MCG_pa = zeros(6,6); % mass matrix 6x6 collocated in cg along principal axes
beam_model.WB.R_pa = zeros(3,3);   % principal axes cosines
%
[beam_model.WB.CG, beam_model.WB.MCG, beam_model.WB.MRP] = ...
   wb_set_conm_mass(beam_model.Info.nconm, beam_model.Node.Index, beam_model.Node.Coord, ...
                    beam_model.Node.R, beam_model.Param.GRDPNT, beam_model.ConM);
% set bar mass CG
[beam_model.WB.CG, beam_model.WB.MCG, beam_model.WB.MRP] =...
   wb_add_bar_mass(beam_model.Info.nbar, beam_model.Node.Coord, beam_model.Node.R, ...
                   beam_model.WB.CG, beam_model.Param.GRDPNT, beam_model.WB.MCG, ...
                   beam_model.WB.MRP, beam_model.Bar);
% get principal axes
[beam_model.WB.MCG_pa, beam_model.WB.R_pa] = wb_principal_axis(beam_model.WB.MCG);

upd_beam_model = beam_model;

end
