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

function [stick] = Stick_ID_Hori(stick, aircraft)
%--------------------------------------------------------------------------------------------------
% Define ID numbers for horizontal component
%
%
% Called by:    Stick_ID.m
%
% Calls:
%
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Modified by Travaglini, 19/11/2009 now horr and fuselage don't have
% common nodes
stick.ID.horr = ( 4000:4000+(length(stick.nodes.horrC2(1,:))-1))';
stick.ID.horr_thick = ( 4000:4000+(length(stick.nodes.horrC2_thick(1,:))-1))';
stick.MAT1.hori = 400;
stick.IDSET.horr = 400;
if isequal(stick.model.horl, 1)     % Left semi-horizontal tail is defined
  % Grid Identification Number
  IndVH1 = find(stick.nodes.horrC2(2,:)==0);% nodes in the symmetry plane
  IndVH1_thick = find(stick.nodes.horrC2_thick(2,:)==0);% nodes in the symmetry plane
  stick.ID.horl = stick.ID.horr(IndVH1)';
  stick.ID.horl = [stick.ID.horl, stick.ID.horr(end)+1:stick.ID.horr(end)+(length(stick.ID.horr)-length(IndVH1))]';
  stick.ID.horl_thick = stick.ID.horr_thick(IndVH1_thick)';
  stick.ID.horl_thick = [stick.ID.horl_thick, stick.ID.horr_thick(end)+1:stick.ID.horr_thick(end)+(length(stick.ID.horr_thick)-length(IndVH1_thick))]';
  % Identification Number for SET1 card
  stick.IDSET.horl = 450;
end
%
% Property Identification Number
PIDhori_str = 4000;
stick.PID.hori = (PIDhori_str : PIDhori_str + (length(stick.ID.horr)-2))';
stick.PID.hori_thick = (PIDhori_str : PIDhori_str + (length(stick.ID.horr_thick)-2))';
% Element Identification Number
EIDhori_str = 4000;
stick.EID.horr = (EIDhori_str : EIDhori_str + (length(stick.ID.horr)-2))';
stick.EID.horr_thick = (EIDhori_str : EIDhori_str + (length(stick.ID.horr_thick)-2))';
%
% Simmetry if applicable
if isequal(stick.model.symmXZ, 1)
    stick.EID.horl = (stick.EID.horr(end)+1 : stick.EID.horr(end)+1 + (length(stick.ID.horr)-2))';
end

if isequal(stick.model.symmXZ, 1)
    stick.EID.horl_thick = (stick.EID.horr_thick(end)+1 : stick.EID.horr_thick(end)+1 + (length(stick.ID.horr_thick)-2))';
end

