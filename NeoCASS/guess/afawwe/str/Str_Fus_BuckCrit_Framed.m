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

function [tbar_SB, tbar_FB, d] = Str_Fus_BuckCrit_Framed(pdcylin, aircraft, geo, loads, str)
%--------------------------------------------------------------------------------------------------
% Design criteria: buckling
% 
% 
% Called by:    Str_Fus.m
% 
% Calls:        
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
%
% determine optimum buckling sizing
%

tbar = 4/(27^(1/4)).*...
       ( pi*pdcylin.wing.cf/( pdcylin.fus.ckf*geo.fus.epsilon^3*pdcylin.fus.ef*pdcylin.fus.es^3 ) ).^(1/8).*...
       ( 2.*geo.fus.r.^2.*pdcylin.fus.df/pdcylin.fus.ds.*loads.fus.Nxc.^2 ) .^(1/4);
tbar_SB = 3/4 .*tbar;
tbar_FB = 1/4 .*tbar;
d       = geo.fus.r .*sqrt( 6*pdcylin.fus.df/pdcylin.fus.ds*...
          sqrt((pi*pdcylin.wing.cf*geo.fus.epsilon*pdcylin.fus.es)/(pdcylin.fus.ckf*pdcylin.fus.ef)) );
