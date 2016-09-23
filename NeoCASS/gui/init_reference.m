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
% function init_reference(handles, gui_param)
%
function init_reference(handles, gui_param)
%
if ~isempty(gui_param.solver.aero.CREF)
    set(handles.editcref,'String',num2str(gui_param.solver.aero.CREF));
end

if ~isempty(gui_param.solver.aero.BREF)
    set(handles.editbref,'String',num2str(gui_param.solver.aero.BREF));
end

if ~isempty(gui_param.solver.aero.SREF)
    set(handles.editsref,'String',num2str(gui_param.solver.aero.SREF));
end

switch gui_param.solver.aero.SXZ
    
    case 0
        
        set(handles.popupmenuvs,'Value',1);
        
    case 1
        
        set(handles.popupmenuvs,'Value',2);
        
    case -1
   
        set(handles.popupmenuvs,'Value',3);
        
end

switch gui_param.solver.aero.SXY
    
    case 0
        
        set(handles.popupmenuhs,'Value',1);
        set(handles.editheight,'Enable','Off');
        
    case 1
        
        set(handles.popupmenuhs,'Value',2);
        set(handles.editheight,'Enable','On');
        
end

set(handles.editheight,'String',num2str(gui_param.solver.aero.HEIGHT));


switch gui_param.solver.aero.DLM_ORDER
    
    case 2
        
        set(handles.popupmenukn,'Value',1);
        
    case 1
        
        set(handles.popupmenukn,'Value',2);
        
end





