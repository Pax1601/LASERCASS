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
function solve_dynder

global beam_model;
global dlm_model;

fid = beam_model.Param.FID;
MASTER = [];

if (~isempty(find(beam_model.Param.MSOL == 701)))

		fprintf(fid,'\nRunning DLM solver for rigid body dynamic derivatives...\n');
%
%   set DLM struct
%
    dlm_model = set_struct(fid, beam_model.Aero.state.Mach_qhh, beam_model.Aero.state.Kfreq, beam_model.Aero.ref.C_mgc, beam_model.Aero.state.SIMXZ, ...
                           beam_model.Param.DLM_ORDER, beam_model.Aero.lattice_dlm.dx, beam_model.Param.DLM_NP);
%
		NMACH  = length(dlm_model.aero.M);
    NK     = length(dlm_model.aero.k); 
    beam_model.Aero.lattice = beam_model.Aero.lattice_dlm;
    np = beam_model.Aero.lattice.np;
    
%   control surface
    CONTR_SURF = beam_model.Aero.geo.nc; 
%
    if (beam_model.Param.GRDPNT==0)
      REF_POINT = zeros(1,3);
    else
      REF_POINT = beam_model.Node.Coord(beam_model.Param.GRDPNT,:);
    end
    fprintf(fid,'\n\t - Reference point (GRDPNT) coordinates: %g, %g, %g', REF_POINT(1), REF_POINT(2), REF_POINT(3));  
%   boundary condition for rigid body motion   
%    beam_model.Aero.lattice = GetDLMdisplacement(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node.Coord, str_disp, beam_model.Aero, 1);
      
    [dlm_model.data.c_displ, dlm_model.data.dwnwash, dlm_model.data.n_displ] = ...
    set_rigid_boundary_condition(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, REF_POINT, beam_model.Param.SUP_MAMPL,  ...
                                 true, false);
%   boundary condition for control surfaces
    counter = 0;
    if (CONTR_SURF)
      NMODES = size(dlm_model.data.c_displ,3);
      index = find(beam_model.Aero.geo.flapped);
%
      CSURF_MODES = zeros(size(dlm_model.data.c_displ,1), 3, length(index));
      DSURF_MODES = zeros(size(dlm_model.data.dwnwash,1), length(index), NK);
      NSURF_MODES = zeros(size(dlm_model.data.n_displ,1), 3, length(index));
%
      for n=1:length(index)
        REF_POINT(1:3) = beam_model.Aero.lattice.Control.Hinge(n,1,:);
        HINGE_AXIS(1:3) = beam_model.Aero.lattice.Control.Hinge(n,2,:) - beam_model.Aero.lattice.Control.Hinge(n,1,:);
        HINGE_AXIS = HINGE_AXIS ./norm(HINGE_AXIS);
%       construct fictious modes for control surfaces 
        [CSURF_MODES(:,:,n), DSURF_MODES(:,n,:), NSURF_MODES(:,:,n)] = ...
          set_patch_crigid_bc(dlm_model.aero.cref, dlm_model.aero.V, beam_model.Aero.lattice.MID_DPOINT(1:np,:), beam_model.Aero.lattice.COLLOC(1:np,:),...
          beam_model.Aero.lattice.XYZ(1:np,:,:), beam_model.Aero.lattice.N(1:np,:), REF_POINT, beam_model.Aero.lattice.Control.DOF(n).data, HINGE_AXIS, ...
          beam_model.Param.SUP_MAMPL, dlm_model.aero.k, true, false);
      end
%
      if (beam_model.Info.nlink)

        counter = length(unique(beam_model.Aero.Trim.Link.Master));
        MASTER = unique(beam_model.Aero.Trim.Link.Master);
        master = zeros(counter,1);
        sl_control_index = zeros(counter, 2);    
        added_surf = [];
%       get master
        for nmas = 1:length(MASTER)
          for ncontrols = 1:beam_model.Aero.geo.nc
            if (isequal(MASTER{nmas}, beam_model.Aero.lattice.Control.Name{ncontrols}))
              master(nmas) = ncontrols;
              added_surf = [added_surf, ncontrols];
              break;
            end
          end
        end
%       get slaves
        for m=1:length(MASTER) 

          dlm_model.data.c_displ(:,:,NMODES+m) = CSURF_MODES(:,:,master(m));
          dlm_model.data.dwnwash(:,NMODES+m,:) = DSURF_MODES(:,master(m),:);
          dlm_model.data.n_displ(:,:,NMODES+m) = NSURF_MODES(:,:,master(m));

          for k=1:length(beam_model.Aero.Trim.Link.Slave)
            if (isequal(beam_model.Aero.Trim.Link.Master{k}, MASTER{m}))
              for ncontrols = 1:beam_model.Aero.geo.nc
                if (isequal(beam_model.Aero.Trim.Link.Slave{k}, beam_model.Aero.lattice.Control.Name{ncontrols}))
                  control_index(nmas,ncontrols) = ncontrols;
                  added_surf = [added_surf, ncontrols];
                  break;
                end
              end

              dlm_model.data.c_displ(:,:,NMODES+m) = dlm_model.data.c_displ(:,:,NMODES+m)+CSURF_MODES(:,:,ncontrols).*beam_model.Aero.Trim.Link.Coeff(k);
              dlm_model.data.dwnwash(:,NMODES+m,:) = dlm_model.data.dwnwash(:,NMODES+m,:)+DSURF_MODES(:,ncontrols,:).*beam_model.Aero.Trim.Link.Coeff(k);
              dlm_model.data.n_displ(:,:,NMODES+m) = dlm_model.data.n_displ(:,:,NMODES+m)+NSURF_MODES(:,:,ncontrols).*beam_model.Aero.Trim.Link.Coeff(k);

            end
           end
        end
%       add independent surfaces
        missing_surf = setdiff([1:beam_model.Aero.geo.nc], added_surf);
        if (~isempty(missing_surf))
          NMODES = NMODES + length(MASTER);
          for m=1:length(missing_surf)
            dlm_model.data.c_displ(:,:,NMODES+m) = CSURF_MODES(:,:,missing_surf(m));
            dlm_model.data.dwnwash(:,NMODES+m,:) = DSURF_MODES(:,missing_surf(m),:);
            dlm_model.data.n_displ(:,:,NMODES+m) = NSURF_MODES(:,:,missing_surf(m));
          end
          MASTER = [MASTER, beam_model.Aero.lattice.Control.Name{missing_surf}];        
          NMODES = NMODES + length(missing_surf);
          counter = counter + length(missing_surf);
        end
%
      else
        m = [1:length(index)];
        dlm_model.data.c_displ(:,:,NMODES+m) = CSURF_MODES(:,:,m);
        dlm_model.data.dwnwash(:,NMODES+m,:) = DSURF_MODES(:,m,:);
        dlm_model.data.n_displ(:,:,NMODES+m) = NSURF_MODES(:,:,m);
        counter = beam_model.Aero.geo.nc;
      end
    end % csurf
    beam_model.Res = [];
    beam_model.Res.Aero = []; beam_model.Res.Aero.RStab_Der = [];
    NMODES = size(dlm_model.data.c_displ,3);
    beam_model.Res.NDispl = zeros(beam_model.Info.ngrid,6,NMODES);
    beam_model.Res.SOL = 'Linear Dynamic Derivatives';

%
%   assembly AIC matrix
    dlm_model.data.D = assembly_AIC_matrix(fid, beam_model.Aero.lattice, dlm_model);
%   solve system and determine Cp for each mode, reduced frequency and Mach number 
    dlm_model.data.Cp = solve_system_CP(fid, np, NK, NMODES, NMACH, dlm_model.data.D, dlm_model.data.dwnwash);  
%   build aerodynamic transfer matrix
    dlm_model.data.Qhh = set_generalized_f(fid, NMODES, dlm_model.data.c_displ, NK, NMACH, beam_model.Aero.lattice.area(1:np), ...
                                           beam_model.Aero.lattice.N(1:np,:), dlm_model.data.Cp);

    fprintf(fid, '\n\ndone.\n');
%
%   Extract dynamic derivatives for rigid body 6 DOF
%
    beam_model.Res = [];
    beam_model.Res.Aero = []; beam_model.Res.Aero.RStab_Der = [];
    beam_model.Res.NDispl = zeros(beam_model.Info.ngrid,6,NMODES);
    beam_model.Res.SOL = 'Linear Dynamic Derivatives';
%   Normalize aerodynamic transfer matrix
    dlm_model.data.Qhh = dlm_model.data.Qhh./(beam_model.Param.SUP_MAMPL^2);
    HAM = dlm_model.data.Qhh;
    CHREF = dlm_model.aero.cref; 
    CREF = CHREF*2;
    BREF = beam_model.Aero.ref.b_ref;
    SREF = beam_model.Aero.ref.S_ref;
    K = zeros(1,1,NK);
    K(:,:,1:NK) = beam_model.Aero.state.Kfreq;
%
		fprintf(fid,'\nCalculating dynamic stability derivatives...');
    beam_model.Res.Aero.RStab_Der = get_dynder(NMACH, K, CHREF, CREF, BREF, SREF, HAM, counter, MASTER);
	  fprintf(fid,'done.\n');
%
    beam_model.Aero.lattice = [];
	else
%
		error('SOL 701 must be given in input file to run dynamic derivatives analysis.');
%
	end
end
%***********************************************************************************************************************
function [displ, dwnwsh, ndispl] = set_rigid_boundary_condition(fid, geo, lattice, dlm, REF_POINT, MSCALE, SCALE, PLOT_RES)

displ  = [];
dwnwsh = [];
ndispl = [];

counter = 0;

fprintf(fid, '\n - Assemblying boundary displacements and downwash...');

% loop on patches
for n = 1 : geo.nwing 

	fprintf(fid, '\n\t Patch n. %d...', n);

	nytot = geo.ny(n,:); 
	nxtot = geo.nx(n,:) + geo.fnx(n,:);
	np = sum(nxtot .* nytot); % get patch number of panels
	[D_DISPL, DWN, N_DISPL] = set_patch_rigid_bc(dlm.aero.cref, dlm.aero.V, lattice.MID_DPOINT(counter+1:(counter + np),:), lattice.COLLOC(counter+1:(counter + np),:) ,...
	                              lattice.XYZ(counter+1:(counter + np),:,:), lattice.N(counter+1:(counter + np),:), REF_POINT, MSCALE, dlm.aero.k, SCALE, PLOT_RES);

	displ  = [displ; D_DISPL];
	dwnwsh = [dwnwsh; DWN];
    ndispl = [ndispl; N_DISPL];

	counter = counter + np;

	fprintf(fid, 'done.');

end

fprintf(fid, '\n   done.');

end
