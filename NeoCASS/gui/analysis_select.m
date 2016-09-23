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
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     091025      1.3.7   A. De Gaspari    Modification
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function analysis_select(hObject, trim, modal, Vg_plot, Vg_env)
%
function analysis_select(hObject, trim, modal, Vg_plot, Vg_env)

h = guidata(hObject);

lmod = 'Off';
flut = 'Off';
flut_plot = 'Off';
flut_env = 'Off';
static = 'Off';
dyn = 'Off';
point_norm = 'Off';

if Vg_plot,
    lmod = 'On';
    flut = 'On';
    flut_plot = 'On';
end

if Vg_env,
    lmod = 'On';
    flut = 'On';
    flut_env = 'On';
end

if modal
    lmod = 'On';
    dyn = 'On';
    if get(h.popupmenurm,'Value') == 3
        point_norm = 'On';
    else
        point_norm = 'Off';
    end
end

if trim
    static = 'On';
end


set(h.radiobuttontrim,'Value',trim);
set(h.text9,'Enable',static);
set(h.nstates,'Enable',static);
set(h.setCS,'Enable',static);
set(h.pushbuttonselectstates,'Enable',static);
% set(h.generic_maneuver,'Enable',static);
% set(h.typical_maneuver,'Enable',static);


set(h.radiobutton4_modalanalysis,'Value',modal);
set(h.text1,'Enable',dyn);
set(h.text2,'Enable',lmod);
set(h.popupmenurm,'Enable',dyn);
set(h.lmodes,'Enable',lmod);
set(h.text5,'Enable',point_norm);
set(h.text3,'Enable',dyn);
set(h.popupmenuid,'Enable',point_norm);
set(h.minf,'Enable',dyn);
set(h.text6,'Enable',point_norm);
set(h.text4,'Enable',dyn);
set(h.popupmenudof,'Enable',point_norm);
set(h.msup,'Enable',dyn);


set(h.text7,'Enable',flut);
set(h.nfreq,'Enable',flut);
set(h.pushbuttoninsertfreq,'Enable',flut);
set(h.text15_modalbase,'Enable',flut);
set(h.pushbutton4_mselect,'Enable',flut);
set(h.text16_tracking,'Enable',flut);
set(h.pushbutton5_fmodes,'Enable',flut);

set(h.radiovgplot,'Value',Vg_plot);
set(h.text10,'Enable',flut_plot);
set(h.vmax,'Enable',flut_plot);
set(h.text11,'Enable',flut_plot);
set(h.vstep,'Enable',flut_plot);
set(h.text12,'Enable',flut_plot);
set(h.rho,'Enable',flut_plot);
set(h.text13,'Enable',flut_plot);
set(h.mach,'Enable',flut_plot);

set(h.radioenvelope,'Value',Vg_env);
set(h.text8,'Enable',flut_env);
set(h.nmach,'Enable',flut_env);
set(h.pushbuttoninsertmach,'Enable',flut_env);


guidata(hObject, h);

end

