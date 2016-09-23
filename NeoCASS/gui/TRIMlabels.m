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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
%   Author: Alessandro De Gaspari, DIAPM
%
%*******************************************************************************
%
function labels = TRIMlabels()

labels = cell(26,2);

labels{1,1}  = 'ID';        labels{1,2}  = 'id';
labels{2,1}  = 'Label';     labels{2,2}  = 'label';
labels{3,1}  = 'Sym';       labels{3,2}  = 'sym';
labels{4,1}  = 'MACH';      labels{4,2}  = 'mach';
labels{5,1}  = 'ALT';       labels{5,2}  = 'alt';
labels{6,1}  = 'ANGLEA';    labels{6,2}  = 'alpha';
labels{7,1}  = 'SIDES';     labels{7,2}  = 'beta';
labels{8,1}  = 'ROLL';      labels{8,2}  = 'p';
labels{9,1}  = 'PITCH';     labels{9,2}  = 'q';
labels{10,1}  = 'YAW';      labels{10,2}  = 'r';
labels{11,1} = 'URDD1';     labels{11,2} = 'acc_x';
labels{12,1} = 'URDD2';     labels{12,2} = 'acc_y';
labels{13,1} = 'URDD3';     labels{13,2} = 'acc_z';
labels{14,1} = 'URDD4';     labels{14,2} = 'p_rate';
labels{15,1} = 'URDD5';     labels{15,2} = 'q_rate';
labels{16,1} = 'URDD6';     labels{16,2} = 'r_rate';
% labels{17,1} = 'c1wing';    labels{17,2} = 'Flap1';
% labels{18,1} = 'c2wing';    labels{18,2} = 'Flap2';
% labels{19,1} = 'c3wing';    labels{19,2} = 'Aileron';
% labels{20,1} = 'c1ht';      labels{20,2} = 'Elevator';
% labels{21,1} = 'c2vt';      labels{21,2} = 'Rudder';
% Per come è fatto guess in questo momento le variabili di trim
% definite qui sono quelle da impostare all'interno delle schede TRIM
% e non coincidono con quelle da cui parte guess per simmetrizzare.
labels{17,1} = 'flap1r';    labels{17,2} = 'Flap1';
labels{18,1} = 'flap2r';    labels{18,2} = 'Flap2';
labels{19,1} = 'aileronr';  labels{19,2} = 'Aileron';
labels{20,1} = 'elev1r';    labels{20,2} = 'Elevator';
labels{21,1} = 'rudder1';   labels{21,2} = 'Rudder';
labels{22,1} = 'elevC1r';   labels{22,2} = 'Canard';
% labels{22,1} = 'CLIMB';     labels{22,1} = 'theta';
% labels{23,1} = 'BANK';      labels{23,1} = 'phi';
% labels{24,1} = 'HEAD';      labels{24,1} = 'head';
% labels{25,1} = 'THRUST';    labels{25,1} = 'thrust';
labels{23,1} = 'VGUST';     labels{23,2} = 'vgust';
labels{24,1} = 'VSINK';     labels{24,2} = 'vsink';
labels{25,1} = 'STROKE';    labels{25,2} = 'stroke';
labels{26,1} = 'LNDGEFF';   labels{26,2} = 'LandGEff';


end