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
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
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
% function init_settings(handles, gui_param)
%
function init_settings(handles, gui_param)
%
set(handles.nstates,'String',num2str(gui_param.solver.aeros.NSTATES)); % Set on screen the values of nstates

if ~isempty(gui_param.solver.suport)
    set(handles.edit_suport,'String',num2str(gui_param.solver.suport));
end

set(handles.popupmenurm,'Value',gui_param.solver.eig.NORM_MET);
if ~isempty(gui_param.solver.eig.REF_GRID)
    set(handles.popupmenuid,'String',num2str(gui_param.solver.eig.REF_GRID));
end
set(handles.popupmenudof,'Value',gui_param.solver.eig.REF_GRID_C);
% see function lmodes_CreateFcn for lmodes edit box initialization
set(handles.minf,'String',num2str(gui_param.solver.eig.MINF));
set(handles.msup,'String',num2str(gui_param.solver.eig.MAXF));

if ~isempty(gui_param.solver.flutter.NRFreq)
    set(handles.nfreq,'String',num2str(gui_param.solver.flutter.NRFreq));
end

if ~isempty(gui_param.solver.flutter.NMach)
    set(handles.nmach,'String',num2str(gui_param.solver.flutter.NMach));
end

set(handles.vmax,'String',num2str(gui_param.solver.flutter.VMAX));
set(handles.vstep,'String',num2str(gui_param.solver.flutter.NV));
set(handles.rho,'String',num2str(gui_param.solver.flutter.RHO));
set(handles.mach,'String',num2str(gui_param.solver.flutter.Mach_list));




