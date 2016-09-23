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
aircraft.Wing1 = rmfield(aircraft.Wing1, 'airfoil');
aircraft.Wing1 = rmfield(aircraft.Wing1, 'Root_Airfoil');
aircraft.Wing1 = rmfield(aircraft.Wing1, 'Kink1_Airfoil');
aircraft.Wing1 = rmfield(aircraft.Wing1, 'Kink2_Airfoil');
aircraft.Wing1 = rmfield(aircraft.Wing1, 'Tip_Airfoil');
%
aircraft.Vertical_tail = rmfield(aircraft.Vertical_tail, 'airfoil');
aircraft.Vertical_tail = rmfield(aircraft.Vertical_tail, 'Root_Airfoil');
aircraft.Vertical_tail = rmfield(aircraft.Vertical_tail, 'Kink_Airfoil');
aircraft.Vertical_tail = rmfield(aircraft.Vertical_tail, 'Tip_Airfoil');
%
aircraft.Horizontal_tail = rmfield(aircraft.Horizontal_tail, 'airfoil');
aircraft.Horizontal_tail = rmfield(aircraft.Horizontal_tail, 'Root_Airfoil');
aircraft.Horizontal_tail = rmfield(aircraft.Horizontal_tail, 'Kink_Airfoil');
aircraft.Horizontal_tail = rmfield(aircraft.Horizontal_tail, 'Tip_Airfoil');
%
%

% % % B 747-100
% % aircraft.Wing1.airfoilRoot  = 'B109.dat';
% % aircraft.Wing1.airfoilKink1 = 'B206.dat';
% % aircraft.Wing1.airfoilKink2 = 'B303.dat';
% % aircraft.Wing1.airfoilTip   = 'B500.dat';
% % %
% % aircraft.Vertical_tail.airfoilRoot = '0012';
% % aircraft.Vertical_tail.airfoilKink = '0012';
% % aircraft.Vertical_tail.airfoilTip  = '0012';
% % %
% % aircraft.Horizontal_tail.airfoilRoot = '0012';
% % aircraft.Horizontal_tail.airfoilKink = '0012';
% % aircraft.Horizontal_tail.airfoilTip  = '0012';

% TCR
aircraft.Wing1.airfoilRoot  = 'n64a206.dat';
aircraft.Wing1.airfoilKink1 = 'n64a206.dat';
aircraft.Wing1.airfoilKink2 = 'n64a206.dat';
aircraft.Wing1.airfoilTip   = 'n64a206.dat';
%
aircraft.Vertical_tail.airfoilRoot = 'n64a006.dat';
aircraft.Vertical_tail.airfoilKink = 'n64a006.dat';
aircraft.Vertical_tail.airfoilTip  = 'n64a006.dat';
%
aircraft.Horizontal_tail.airfoilRoot = 'n64a006.dat';
aircraft.Horizontal_tail.airfoilKink = 'n64a006.dat';
aircraft.Horizontal_tail.airfoilTip  = 'n64a006.dat';

%
% xml_save(filename_geo, aircraft)
