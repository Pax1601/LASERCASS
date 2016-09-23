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
%----------------------------------------------------------------------------------------------------------------------
% Define non-structural masses.
% 
% Called by:    guess.m
% 
% Calls:        Add_NSM_Fuse.m, Add_NSM_Wing.m, Add_NSM_Vert.m, Add_NSM_Vert.m,
%               Add_NSM_Fuse_setup.m, Add_NSM_Wing_setup.m,  
% 
%   <andreadr@kth.se>
%----------------------------------------------------------------------------------------------------------------------
function [str] = Add_NSM(outf, pdcylin, aircraft, geo, stick, str)

% Inizialize variables
str.fus.NSM.dstr   = zeros(length(stick.PID.fuse), 1);      % distributed NSM on fuselage                  [kg/m], vector
str.fus.NSM.add    = [];                                    % consider each single contribution            [kg/m], struct
str.wing.NSM.dstr  = zeros(length(stick.PID.wing), 1);      % distributed NSM on wing                      [kg/m], vector
str.vtail.NSM.dstr = zeros(length(stick.PID.vert), 1);      % distributed NSM on vertical tail             [kg/m], vector
str.htail.NSM.dstr = zeros(length(stick.PID.hori), 1);      % distributed NSM on horizontal tail           [kg/m], vector
str.canard.NSM.dstr= zeros(length(stick.PID.canr), 1);      % distributed NSM on canard                    [kg/m], vector
str.tbooms.NSM.dstr= zeros(length(stick.PID.tbooms), 1);    % distributed NSM on tbooms                    [kg/m], vector

% Fuselage
if isequal(pdcylin.stick.model.fuse, 1)
    str = Add_NSM_Fuse_setup(outf, aircraft, geo, stick, str);
end

% Wing
if isequal(pdcylin.stick.model.winr, 1)
    str = Add_NSM_Wing_setup(outf, pdcylin, aircraft, geo, stick, str);
end

% Vertical tail
if isequal(pdcylin.stick.model.vert, 1)
    str = Add_NSM_Vert_setup(outf, pdcylin, aircraft, geo, stick, str);
end

% Horizontal tail
if isequal(pdcylin.stick.model.horr, 1)
   str = Add_NSM_Hori_setup(outf, pdcylin, aircraft, geo, stick, str);
end

% Canard
if isequal(pdcylin.stick.model.canr, 1)
   str = Add_NSM_Canard_setup(outf, pdcylin, aircraft, geo, stick, str);
end

% tbooms
if isequal(stick.model.tboomsr,1)
    str = Add_NSM_Tbooms_setup(outf, aircraft, geo, stick, str);
end

% % Wing2
% 
% if isequal(pdcylin.stick.model.win2r, 1)
%    [str] = Add_NSM_Wing2_setup(outf, pdcylin, aircraft, geo, stick, str);
% end

