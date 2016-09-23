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
%---------------------------------------------------------------------------------------------------------------------------
% 
% Called by:    Stick_Model.m
%
%---------------------------------------------------------------------------------------------------------------------------
function [stick] = Stick_Extra_Link(stick, aircraft)
%
stick.link.extra = {};
cont=0;
%
% Joined wing is supposed to link wing and htail
%
if (aircraft.Joined_wing.present==1)
  cont = cont+1;
  stick.link.extra{cont} = 'jwing';
end
%
% Strut braced wing is supposed to link wing and canard
%
if (aircraft.Strut_wing.present>0)
  cont = cont+1;
  stick.link.extra{cont} = 'swing';
end