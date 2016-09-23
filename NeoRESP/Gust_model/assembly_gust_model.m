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
function assembly_gust_model

global dyn_model

if (~isempty(find(dyn_model.beam.Param.MSOL == 145,1))) && isfield(dyn_model.beam,'Gust') && ~isempty(dyn_model.beam.Gust.ID)
    fid = dyn_model.beam.Param.FID;
    if dyn_model.beam.Param.SOL == 146
        
        ngust = length(dyn_model.beam.Gust.ID);
%         dyn_model.gust.NDispl = zeros(dyn_model.beam.Info.ngrid,6,ngust);
%         dyn_model.gust.dwnwash = zeros(dyn_model.beam.Aero.lattice_dlm.np,ngust,length(dyn_model.dlm.aero.k));   
        dyn_model.gust.dwnwash = zeros(dyn_model.beam.Aero.lattice_dlm.np,ngust);

        for i = 1 : ngust
%             dyn_model.gust.NDispl(:,:,i) = get_gust_mode(dyn_model.beam.Node,dyn_model.beam.Info.ndof,dyn_model.beam.Gust.funs{i},dyn_model.beam.Gust.DIR(i));
            dyn_model.gust.dwnwash(:,i) = get_gust_mode_mod(dyn_model.beam.Aero.lattice_dlm,dyn_model.beam.Gust, dyn_model.dlm.aero.cref, dyn_model.dlm.aero.k, dyn_model.beam.Node ,i);
        end
   
%         [dyn_model.gust.c_displ, dyn_model.gust.dwnwash, dyn_model.gust.n_displ] = set_boundary_condition_Gust(fid, dyn_model.beam.Aero.geo, dyn_model.beam.Aero.lattice, dyn_model.dlm, dyn_model.beam.Node.Coord, ...
%             dyn_model.gust.NDispl, dyn_model.beam.Aero, dyn_model.beam.Gust.X0, dyn_model.beam.Gust.Vinfty, true, false);
        
        NMODES = size(dyn_model.dlm.data.c_displ,3);
        NMODES2 = size(dyn_model.dlm.data.Qhh,1);
        NMACH  = length(dyn_model.dlm.aero.M);
        NK     = length(dyn_model.dlm.aero.k);
        np = dyn_model.beam.Aero.lattice.np;
                
%         dyn_model.gust.Cp = solve_system_CP(fid, np, NK, ngust, NMACH, dyn_model.dlm.data.D, dyn_model.gust.dwnwash);
                
        dyn_model.gust.Cp = dyn_model.dlm.data.invD;
        dyn_model.gust.Qhg = zeros(NMODES, np, NK, NMACH);
        
        dyn_model.gust.Qhg = set_generalized_f2(fid, np, NMODES , dyn_model.dlm.data.c_displ, NK, NMACH, dyn_model.beam.Aero.lattice.area(1:np), ...
            dyn_model.beam.Aero.lattice.N(1:np,:), dyn_model.gust.Cp);
        
        CONTR_SURF = dyn_model.beam.Aero.geo.nc;
        
        if (CONTR_SURF)
            
            dyn_model.gust.Qdg = dyn_model.gust.Qhg(NMODES2+1:end,:,:);
            dyn_model.gust.Qhg = dyn_model.gust.Qhg(1:NMODES2,:,:);
        else
            dyn_model.gust.Qdg = [];
        end
        
    elseif dyn_model.beam.Param.SOL == 147 % acceleration mode
        
        ngust = length(dyn_model.beam.Gust.ID);
        dyn_model.gust.NDispl = zeros(dyn_model.beam.Info.ngrid,6,ngust);
        
        for i = 1 : ngust
%             dyn_model.gust.NDispl(:,:,i) = get_gust_mode(dyn_model.beam.Node,dyn_model.beam.Info.ndof,dyn_model.beam.Gust.funs{i},dyn_model.beam.Gust.DIR(i));
            dyn_model.gust.dwnwash(:,i) = get_gust_mode_mod(dyn_model.beam.Aero.lattice_dlm,dyn_model.beam.Gust, dyn_model.dlm.aero.cref, dyn_model.dlm.aero.k, dyn_model.beam.Node ,i);
        end
        
        NMODES = size(dyn_model.dlm.data.c_displ,3);
        NMODES2 = size(dyn_model.dlm.data.Qhh,1);
        NMACH  = length(dyn_model.dlm.aero.M);
        NK     = length(dyn_model.dlm.aero.k);
        np = dyn_model.beam.Aero.lattice.np;
        ndof = size(dyn_model.beam.Struct.K,1);
                
%         dyn_model.gust.Cp = solve_system_CP(fid, np, NK, ngust, NMACH, dyn_model.dlm.data.D, dyn_model.gust.dwnwash);
%         
%         dyn_model.gust.Qhg = set_generalized_f2(fid, ngust, NMODES , dyn_model.dlm.data.c_displ, NK, NMACH, dyn_model.beam.Aero.lattice.area(1:np), ...
%             dyn_model.beam.Aero.lattice.N(1:np,:), dyn_model.gust.Cp);

        dyn_model.gust.Cp = dyn_model.dlm.data.invD;
        dyn_model.gust.Qhg = zeros(NMODES, np, NK, NMACH); 
        
        dyn_model.gust.Qhg = set_generalized_f2(fid, np, NMODES , dyn_model.dlm.data.c_displ, NK, NMACH, dyn_model.beam.Aero.lattice.area(1:np), ...
            dyn_model.beam.Aero.lattice.N(1:np,:), dyn_model.gust.Cp);
        
        dyn_model.gust.Qng = set_generalized_f2(fid, np, ndof , dyn_model.dlm.data.c_displDof, NK, NMACH, dyn_model.beam.Aero.lattice.area(1:np), ...
            dyn_model.beam.Aero.lattice.N(1:np,:), dyn_model.gust.Cp);
        
        CONTR_SURF = dyn_model.beam.Aero.geo.nc;
        
        if (CONTR_SURF)
            
            dyn_model.gust.Qdg = dyn_model.gust.Qhg(NMODES2+1:end,:,:);
            dyn_model.gust.Qhg = dyn_model.gust.Qhg(1:NMODES2,:,:);
        else
            dyn_model.gust.Qdg = [];
        end
    else
        
    end
    
else
    %
    error('dynamic parameters missing or not gust defined.');
    %
end