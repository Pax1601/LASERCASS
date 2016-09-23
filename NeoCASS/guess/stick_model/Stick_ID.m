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
%--------------------------------------------------------------------------------------------------
% 
% Called by:    Stick_Model.m
% 
% Calls:        Stick_ID_Fuse.m, Stick_ID_Wing.m, Stick_ID_Vert.m, Stick_ID_Hori.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Modified by Travaglini 19/11/2009
function [stick] = Stick_ID(stick,aircraft)

if isequal(stick.model.fuse, 1)
    [stick] = Stick_ID_Fuse(stick);
end

if isequal(stick.model.winr, 1)
    [stick] = Stick_ID_Wing(stick);
end

if isequal(stick.model.vert, 1)
    [stick] = Stick_ID_Vert(stick,aircraft);
end

if isequal(stick.model.horr, 1)
   [stick] = Stick_ID_Hori(stick,aircraft);
end
if isequal(stick.model.canr, 1)
   [stick] = Stick_ID_Canr(stick);
end
if isequal(stick.model.win2r, 1)
   [stick] = Stick_ID_Wing2(stick);
end

if isequal(stick.model.tboomsr, 1)
    [stick] = Stick_ID_Tbooms(stick);
end
