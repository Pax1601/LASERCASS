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
% function enable_solver(handles)
%
function enable_solver(handles)
%
global beam_model

if ( (beam_model.Param.MSOL(1) == 101) || (beam_model.Param.MSOL(5) == 600) )
    code_enable(1,handles.Main_File_checkbox2static,handles.Main_Run_pushbutton2static);
else
    code_disable(1,handles.Main_File_checkbox2static,handles.Main_Run_pushbutton2static);
end

if (beam_model.Param.MSOL(2) == 103)
    code_enable(2,handles.Main_File_checkbox3modal,handles.Main_Run_pushbutton3modal);
else
    code_disable(2,handles.Main_File_checkbox3modal,handles.Main_Run_pushbutton3modal);
end

% BUCKLING
%  if (beam_model.Param.MSOL(3) == 105)
%      code_enable(2,handles.Main_File_checkbox3modal,handles.Main_Run_pushbutton3modal);
%  else
%      code_disable(2,handles.Main_File_checkbox3modal,handles.Main_Run_pushbutton3modal);
%  end

if ( (beam_model.Param.MSOL(4) == 144) || (beam_model.Param.MSOL(7) == 644) )
    code_enable(3,handles.Main_File_checkbox4staer,handles.Main_Run_pushbutton4staer);
else
    code_disable(3,handles.Main_File_checkbox4staer,handles.Main_Run_pushbutton4staer);
end

if (beam_model.Param.MSOL(5) == 145)
    code_enable(4,handles.Main_File_checkbox5flutter,handles.Main_Run_pushbutton5flutter);
else
    code_disable(4,handles.Main_File_checkbox5flutter,handles.Main_Run_pushbutton5flutter);
end

if (beam_model.Param.MSOL(8) == 700)
    code_enable(7,handles.Main_File_checkbox1VLM,handles.Main_Run_pushbutton6vlm);
elseif (beam_model.Param.MSOL(9) == 701)
    code_enable(8,handles.Main_File_checkbox1VLM,handles.Main_Run_pushbutton6vlm);
else
    code_disable(7,handles.Main_File_checkbox1VLM,handles.Main_Run_pushbutton6vlm);
    code_disable(8,handles.Main_File_checkbox1VLM,handles.Main_Run_pushbutton6vlm);
end




