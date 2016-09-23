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
% function filenameo = uiputext(filenamei, filterindex, formats)
%
function filename = uiputext(filename, filterindex, formats)
%
symb = {'\ ','\;','\,'};
dot = regexpi(filename,'\.');
if isempty(dot)
    ext = '';
else
    ext = filename(dot(end):end);
end
extstr = formats{filterindex,1};
if isempty(strfind(extstr, ext))
    extsymb = regexpi(extstr,symb);
    extend = min(extsymb{1});
    for i = 2:length(symb)
        ml = min(extsymb{i});
        if ml < extend
            extend = ml;
        end
    end
    extdef = formats{filterindex,1}(2:extend-1);
    if ~strcmpi(extdef,'.*')
        filename = [filename,extdef];
    end
end

