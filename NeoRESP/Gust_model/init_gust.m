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
%  FFAST Project
%
%  NeoSYM
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Lorenzo Travaglini   <>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by FFAST partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Lorenzo Travaglini
%***********************************************************************************************************************
function init_gust

global dyn_model
%
if (~isempty(find(dyn_model.beam.Param.MSOL == 145,1))) && isfield(dyn_model.beam,'Gust') && ~isempty(dyn_model.beam.Gust.ID)

  fid = dyn_model.beam.Param.FID;
  MODACC = dyn_model.beam.Param.MODACC;
  CONTR_SURF = dyn_model.beam.Aero.geo.nc;
  ngust = length(dyn_model.beam.Gust.ID);
  dyn_model.gust.dwnwash = zeros(dyn_model.beam.Aero.lattice_dlm.np,ngust);
%
  for i = 1 : ngust
      dyn_model.gust.dwnwash(:,i) = get_gust_mode_mod(dyn_model.beam.Aero.lattice_dlm,dyn_model.beam.Gust, ...
                                       dyn_model.dlm.aero.cref, dyn_model.dlm.aero.k, dyn_model.beam.Node ,i);
  end
%   
  NMODES  = size(dyn_model.dlm.data.c_displ,3);
  NMODES2 = size(dyn_model.dlm.data.Qhh,1);
  NMACH   = length(dyn_model.dlm.aero.M);
  NK      = length(dyn_model.dlm.aero.k);
  np      = dyn_model.beam.Aero.lattice.np;
%                
  dyn_model.gust.Cp = dyn_model.dlm.data.invD;
% MODAL GUST LOADS HAG
  dyn_model.gust.Qhg = zeros(NMODES, np, NK, NMACH);
  dyn_model.gust.Qhg = set_generalized_f2(fid, np, NMODES , dyn_model.dlm.data.c_displ, NK, NMACH, ...
                         dyn_model.beam.Aero.lattice.area(1:np), dyn_model.beam.Aero.lattice.N(1:np,:), dyn_model.gust.Cp);
% NODAL GUST LOADS HAG*
  if (MODACC == 0)
          dyn_model.gust.Qng = set_nodal_f(fid, dyn_model.beam.Info, dyn_model.beam.Node, np, ...
                                           dyn_model.beam.Aero, NK, NMACH, dyn_model.gust.Cp);
  end
% HINGE LOADS
  if (CONTR_SURF)
    dyn_model.gust.Qdg = dyn_model.gust.Qhg(NMODES2+1:end,:,:,:);
    dyn_model.gust.Qhg = dyn_model.gust.Qhg(1:NMODES2,:,:,:);
  else
    dyn_model.gust.Qdg = [];
  end
%
end
