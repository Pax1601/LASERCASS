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
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Luca Cavagna
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%*******************************************************************************
% This function determines aero load coefficients wtr modal base
function [Cy_mode, Cz_mode, Cl_mode, Cm_mode, Cn_mode] = rigid_aero_force(Cp, dyn_model, cref, bref, sref, Xsup)
%
  NMODES = size(Cp,2); 
  nk = size(Cp,3);
  np = dyn_model.beam.Aero.lattice_dlm.np;
  midPoint = dyn_model.beam.Aero.lattice_dlm.MID_DPOINT(1:np,:)*cref;
  %           Aero forces
  %             lateral
  Cy_mode = sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*(dyn_model.beam.Aero.lattice_dlm.N(:,2))),1);
  Cy2 = zeros(1, NMODES, nk*2);
  Cy2(:,:,1:2:nk*2) = Cy_mode/sref;
  [dummy, dummy, dummy, Cy_mode] = spline_coefficients_vec(dyn_model.dlm.aero.k, Cy2);
  %           vertical
  Cz_mode = sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*(dyn_model.beam.Aero.lattice_dlm.N(:,3))),1);
  Cz2 = zeros(1, NMODES, nk*2);
  Cz2(:,:,1:2:nk*2) = Cz_mode/sref;
  [dummy, dummy, dummy, Cz_mode] = spline_coefficients_vec(dyn_model.dlm.aero.k, Cz2);
  %           Aero moments
  %           pitch
  Cm_mode = sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*dyn_model.beam.Aero.lattice_dlm.N(:,3).*...
               (Xsup(1)-midPoint(:,1))),1) + sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*dyn_model.beam.Aero.lattice_dlm.N(:,1).*...
               (midPoint(:,3)-Xsup(3))),1);
  Cm2 = zeros(1, NMODES, nk*2);
  Cm2(:,:,1:2:nk*2) = Cm_mode/(sref*cref*2);
  [dummy, dummy, dummy, Cm_mode] = spline_coefficients_vec(dyn_model.dlm.aero.k, Cm2);
  %           roll
  Cl_mode = sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*dyn_model.beam.Aero.lattice_dlm.N(:,2).*...
               (Xsup(3)-midPoint(:,3))),1) + sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*dyn_model.beam.Aero.lattice_dlm.N(:,3).*...
               (midPoint(:,2)-Xsup(2))),1);
  Cl2 = zeros(1, NMODES, nk*2);
  Cl2(:,:,1:2:nk*2) = Cl_mode/(sref*bref);
  [dummy, dummy, dummy, Cl_mode] = spline_coefficients_vec(dyn_model.dlm.aero.k, Cl2);
  %           yaw
  Cn_mode = sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*dyn_model.beam.Aero.lattice_dlm.N(:,1).*...
               (Xsup(2)-midPoint(:,2))),1) + sum(bsxfun(@times,Cp, ...
                dyn_model.beam.Aero.lattice_dlm.area.*dyn_model.beam.Aero.lattice_dlm.N(:,2).*...
               (midPoint(:,1)-Xsup(1))),1);
  Cn2 = zeros(1, NMODES, nk*2);
  Cn2(:,:,1:2:nk*2) = Cn_mode/(sref*bref);
  [dummy, dummy, dummy, Cn_mode] = spline_coefficients_vec(dyn_model.dlm.aero.k, Cn2);

end