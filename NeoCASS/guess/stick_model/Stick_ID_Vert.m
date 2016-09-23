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

function [stick] = Stick_ID_Vert(stick, aircraft)
%--------------------------------------------------------------------------------------------------
% Define ID numbers for vertical component
%
%
% Called by:    Stick_ID.m
%
% Calls:
%
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Modified by Travaglini, 19/11/2009 now vert and fuselage don't have
% common nodes
stick.MAT1.vert = 300;
stick.IDSET.vert = 301;
stick.ID.vert = ( 3000:3000+(length(stick.nodes.vert(1,:))-1))';
stick.ID.vert_thick = ( 3000:3000+(length(stick.nodes.vert_thick(1,:))-1))';
stick.ID.vert2 = [];
stick.ID.vert2_thick = [];
if isequal(aircraft.Vertical_tail.Twin_tail, 1)
    stick.ID.vert2 = ( 3500:3500+(length(stick.nodes.vert2(1,:))-1))';
    stick.ID.vert2_thick = ( 3500:3500+(length(stick.nodes.vert2_thick(1,:))-1))';
    stick.IDSET.vert2 = 351;
end
% Property Identification Number
PIDvert_str = 3000;
stick.PID.vert = (PIDvert_str : PIDvert_str + (length(stick.ID.vert)-2))';
stick.PID.vert_thick = (PIDvert_str : PIDvert_str + (length(stick.ID.vert_thick)-2))';
stick.PID.vert2 = [];
stick.PID.vert2_thick = [];
if isequal(aircraft.Vertical_tail.Twin_tail, 1)
    stick.PID.vert2 = stick.PID.vert;%(PIDvert_str+500 : PIDvert_str+500 + (length(stick.ID.vert2)-2))';
    stick.PID.vert2_thick = stick.PID.vert_thick;%(PIDvert_str+500 : PIDvert_str+500 + (length(stick.ID.vert2_thick)-2))';
end
% Element Identification Number
EIDvert_str = 3000;
stick.EID.vert = (EIDvert_str : EIDvert_str + (length(stick.ID.vert)-2))';
stick.EID.vert_thick = (EIDvert_str : EIDvert_str + (length(stick.ID.vert_thick)-2))';
stick.EID.vert2 = [];
stick.EID.vert2_thick = [];
if isequal(aircraft.Vertical_tail.Twin_tail, 1)
    stick.EID.vert2 = (EIDvert_str+500 : EIDvert_str+500 + (length(stick.ID.vert2)-2))';
    stick.EID.vert2_thick = (EIDvert_str+500 : EIDvert_str+500 + (length(stick.ID.vert2_thick)-2))';
end
