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
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080407      1.0     A. Da Ronch      Creation
%     091117      1.3.9   L.Travaglini     Modification
%
%
% Called by:    MainCode.m
% 
% Calls:        BULKdataAEROS.m
%
%*******************************************************************************
%
% function       writeAEROS2file(fid, stick, aircraft)
%
%   DESCRIPTION:  Write AEROS card
%
%         INPUT: NAME           TYPE       DESCRIPTION
% 
%                fid            double     file identifier
%
%                stick          struct     stores structural properties
%  
%                aircraft       struct     stores geometrical data
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%
%
%    REFERENCES:
%
%*******************************************************************************

function [] = writeAEROS2file(fid, stick, aircraft, ref)%,IDSUPORT)

% set to 0
CP = 0;
S = aircraft.wing1.area;
% Reference area [m2]
if isequal(stick.model.symmXZ, 1)
    REFS  = S;
else
    REFS  = S/2;
end
% Reference span [m]
REFB = aircraft.wing1.span;
% Reference chord [m]
% REFC = S/REFB;
if nargin == 4 && ~isempty(ref)
    REFC = ref.C_mac;
else
    REFC = MAC_wing(aircraft.wing1, aircraft.fuselage.Total_fuselage_length);
end
%
% Symmetry X-Z
SYMXZ = 0;
% Symmetry X-Y
SYMXY = 0;
BULKdataPARAM(fid)
% fprintf(fid, '$\n');
% BULKdataSUPORT(fid,IDSUPORT)
% fprintf(fid, '$\n');

% AEROS card
BULKdataAEROS(fid, CP, REFC, REFB, REFS, SYMXZ, SYMXY);
