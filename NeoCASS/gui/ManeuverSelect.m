%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
% Alessandro De Gaspari (degaspari@aero.polimi.it)
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
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090903      1.3.7   A. De Gaspari    Creation
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function I = ManeuverSelect()
%
function I = ManeuverSelect()
%
global beam_model
strim = beam_model.Aero.Trim;
lst = length(strim.ID);
list = cell(lst,1);
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
Tlabels = gui_param.solver.aeros.TRIM_LABELS;
maxlab = size(char(strim.Label_Select),2);

for i = 1:lst,
    label = strim.Label_Select{strim.Select==strim.ID(i)};
    strlabel = [label, blanks(maxlab-length(label))];
    
    param = strim.Param(i).data;
    value = strim.Value(i).data;
    Ilab = value~=0;
    subparam = ['ID', 'Sym', 'MACH', 'ALT', param(Ilab)];
    subvalue = [strim.ID(i), strim.Type(i), strim.Mach(i), strim.ALT(i), value(Ilab)];
    l = length(subvalue);
    chinput = [char(subparam), repmat('=',l,1), num2str(subvalue'), repmat(',',l,1)];
    strinp = reshape(chinput',1,numel(chinput));
    strinput = strinp(1:end-1);
    
    strvar = '';
    Tl = [Tlabels(6:end,1); {'CLIMB'; 'BANK'; 'HEAD'; 'THRUST'}];
    if all(ismember(param, Tl)),
        Ivar = ~ismember(Tl, param);
        labvar = Tlabels([false(5,1);Ivar], size(Tlabels,2));
        chvar = [char(labvar), repmat(',',length(labvar),1)];
        strv = reshape(chvar',1,numel(chvar));
        strv = strv(1:end-1);
        strvar = ['(Trim Variables: ', strv(regexpi(strv,'\S')), ')'];
    end
    
    list{find(strim.Select==strim.ID(i))} = [strlabel, ' = ', ...
                                  strinput(regexpi(strinput,'\S')), ' ', strvar];
end

I = select_list(list, 'Loaded Maneuver List:', 'TRIM Conditions Selection');



