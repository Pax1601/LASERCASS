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
%***********************************************************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Alessandro Scotti <scotti@aero.polimi.it>
%
%   Rearranged from the Program MAINFL developed by DIAPM (5-5-1990)
%   Author: Paolo Mantegazza <mantegazza@aero.polimi.it>
%           Sergio Ricci <ricci@aero.polimi.it>
%***********************************************************************************************************************
%     MODIFICATIONS:
%     DATE        VERS   PROGRAMMER    DESCRIPTION
%     080126      1.0    L.Cavagna     Massive restyling
%
function fl_model = method_param(in_struct)
%
fl_model = in_struct;
fl_model.param = [];
%
% Method parameters
%
% Aerodynamic Scaling Factor (Default Value =1)
fl_model.param.AERSCA = 1;
% Maximum Iteration
fl_model.param.MAXITE = 100;
% Maximum Iteration
fl_model.param.MAXITS = 101;
% Minimum Delta V Step Increase
fl_model.param.DVMIN = 0.01;
% Maximum Delta V Step Increase
fl_model.param.DVMAX = 10;
% Standard Delta V Increase
fl_model.param.DVEL = 1;
% Minimum Flutter Velocity
fl_model.param.VMIN = 10;
% Maximum Flutter Velocity
fl_model.param.VMAX = 0.0;
% Percentage Error Allowed fo Check Criterion 
fl_model.param.ERR = 1e-3;
% Percentage Error Allowed for V Step doubling
fl_model.param.DOUBLING = 1e-3;
% Internal variables
fl_model.param.SVQU = zeros(13,1);
% Linestyle Variable
fl_model.param.linestyle = ['bo-';'gx-';'r+-';'c*-';'ms-';'yd-';'kv-';'b<-';'g>-';'rp-';'ch-';'m.-';'c+-';'m*-';'ys-';...
                            'kd-';'bv-';'g<-';'r>-';'kp-';'md-';'m.-';'c+-';'m*-';'ys-';'kd-';'bv-';'g<-';'r>-';'kp-'];
%
