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
function Stab_Der = get_stab_der(AEROMAT, HINGEMAT)

  if ~isempty(HINGEMAT)
    nh = 1;
  else
    nh = 0;
  end
  Stab_Der = [];
  Stab_Der.Alpha = [];
  Stab_Der.Beta = [];
  Stab_Der.P_rate = [];
  Stab_Der.Q_rate = [];
  Stab_Der.R_rate = [];
  Stab_Der.Control = [];
%  Stab_Der.Alpha.dcd_dalpha = AEROMAT(1,1);  
  Stab_Der.Alpha.dcs_dalpha = AEROMAT(2,1);  
  Stab_Der.Alpha.dcl_dalpha = AEROMAT(3,1);
  %
  Stab_Der.Alpha.dcml_dalpha = AEROMAT(4,1);  
  Stab_Der.Alpha.dcmm_dalpha = AEROMAT(5,1);  
  Stab_Der.Alpha.dcmn_dalpha = AEROMAT(6,1);
  if nh
    Stab_Der.Alpha.dcmh_dalpha = HINGEMAT(:,1);
    Stab_Der.Beta.dcmh_dbeta = HINGEMAT(:,2);
    Stab_Der.P_rate.dcmh_dP = HINGEMAT(:,3);
    Stab_Der.Q_rate.dcmh_dQ = HINGEMAT(:,4);
    Stab_Der.R_rate.dcmh_dR = HINGEMAT(:,5);
  end
  %
  % DELTA BETA
%  Stab_Der.Beta.dcd_dbeta = AEROMAT(1,2);  
  Stab_Der.Beta.dcs_dbeta = AEROMAT(2,2);  
  Stab_Der.Beta.dcl_dbeta = AEROMAT(3,2);  
  %
  Stab_Der.Beta.dcml_dbeta = AEROMAT(4,2);  
  Stab_Der.Beta.dcmm_dbeta = AEROMAT(5,2);
  Stab_Der.Beta.dcmn_dbeta = AEROMAT(6,2);
  %
  % DELTA P rate
%  Stab_Der.P_rate.dcd_dP = AEROMAT(1,3);  
  Stab_Der.P_rate.dcs_dP = AEROMAT(2,3);  
  Stab_Der.P_rate.dcl_dP = AEROMAT(3,3);  
  %
  Stab_Der.P_rate.dcml_dP = AEROMAT(4,3);  
  Stab_Der.P_rate.dcmm_dP = AEROMAT(5,3);  
  Stab_Der.P_rate.dcmn_dP = AEROMAT(6,3);
  %
  % DELTA Q rate
%  Stab_Der.Q_rate.dcd_dQ = AEROMAT(1,4);  
  Stab_Der.Q_rate.dcs_dQ = AEROMAT(2,4);  
  Stab_Der.Q_rate.dcl_dQ = AEROMAT(3,4);  
  %
  Stab_Der.Q_rate.dcml_dQ = AEROMAT(4,4);  
  Stab_Der.Q_rate.dcmm_dQ = AEROMAT(5,4);  
  Stab_Der.Q_rate.dcmn_dQ = AEROMAT(6,4);  
  %
  % DELTA R rate
%  Stab_Der.R_rate.dcd_dR = AEROMAT(1,5);  
  Stab_Der.R_rate.dcs_dR = AEROMAT(2,5);  
  Stab_Der.R_rate.dcl_dR = AEROMAT(3,5);  
  %
  Stab_Der.R_rate.dcml_dR = AEROMAT(4,5);  
  Stab_Der.R_rate.dcmm_dR = AEROMAT(5,5);  
  Stab_Der.R_rate.dcmn_dR = AEROMAT(6,5);
  %
  % controls
  nc = 0;
  for n=6:size(AEROMAT,2)
    nc = nc+1;
    Stab_Der.Control.dcs_dDelta(:,nc)  =  AEROMAT(2,n);
    Stab_Der.Control.dcl_dDelta(:,nc)  =  AEROMAT(3,n);
    Stab_Der.Control.dcml_dDelta(:,nc) =  AEROMAT(4,n);
    Stab_Der.Control.dcmm_dDelta(:,nc) =  AEROMAT(5,n);
    Stab_Der.Control.dcmn_dDelta(:,nc) =  AEROMAT(6,n);
    if nh
      Stab_Der.Control.dcmh_dDelta(:,nc) =  HINGEMAT(:,n);
    end
  end

end