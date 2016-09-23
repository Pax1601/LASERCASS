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

function [stick] = Stick_ID_Fuse(stick)
%--------------------------------------------------------------------------------------------------
% Define ID numbers for fuselage component
% 
% 
% Called by:    Stick_ID.m
% 
% Calls:        
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------

% Grid Identification Number
IDfuse_str = 1000;
stick.ID.fuse  = (IDfuse_str : IDfuse_str + (length(stick.nodes.fuse(1,:))-1))';
stick.ID.fuse_thick  = (IDfuse_str : IDfuse_str + (length(stick.nodes.fuse_thick(1,:))-1))';
% Material Identification Number
stick.MAT1.fuse = 100;

% Property Identification Number
PIDfuse_str = 1000;
stick.PID.fuse = (PIDfuse_str : PIDfuse_str + (length(stick.ID.fuse)-2))';
stick.PID.fuse_thick = (PIDfuse_str : PIDfuse_str + (length(stick.ID.fuse_thick)-2))';
% stick.PID.fuse = [1:length(stick.ID.fuse)-1]';

% Element Identification Number
EIDfuse_str = 1000;
stick.EID.fuse = (EIDfuse_str : EIDfuse_str + (length(stick.ID.fuse)-2))';
stick.EID.fuse_thick = (EIDfuse_str : EIDfuse_str + (length(stick.ID.fuse_thick)-2))';
% stick.EID.fuse = [1:length(stick.ID.fuse)-1]';

% Identification Number for SET1 card
stick.IDSET.fuse = 1;
