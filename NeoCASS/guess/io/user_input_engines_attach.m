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

function [aircraft] = user_input_engines_attach(fid, aircraft)

input_user_engines1 = 100;
input_user_engines2 = 100;



if isequal(aircraft.engines1.Layout_and_config, 4) && ~isequal(aircraft.engines1.Number_of_engines, 0)
    fprintf(fid, '\n+--------------------------------------------------------------------------------');
    fprintf(fid, '\n|    Engines1 layout and configuration %g: straight duct without any attachement', aircraft.engines1.Layout_and_config);
    while (~isequal(input_user_engines1, 0) && ~isequal(input_user_engines1, 1))
    input_user_engines1 = input(...
                 '\n|    Engines1 attached at wing [0] or fuselage [1]? ');
    end
    fprintf(fid,   '+--------------------------------------------------------------------------------');
    aircraft.engines1.Layout_and_config = input_user_engines1 *3;
end
if isequal(aircraft.engines1.Layout_and_config, 5) && ~isequal(aircraft.engines1.Number_of_engines, 0)
    fprintf(fid, '\n+--------------------------------------------------------------------------------');
    fprintf(fid, '\n|    Engines1 layout and configuration %g: S duct without attachement', aircraft.engines1.Layout_and_config);
    input_user_engines1 = 100;
    while (~isequal(input_user_engines1, 0) && ~isequal(input_user_engines1, 1))
    input_user_engines1 = input(...
                 '\n|    Engines1 attached at wing [0] or fuselage [1]? ');
    end
    fprintf(fid,   '+--------------------------------------------------------------------------------');
    aircraft.engines1.Layout_and_config = input_user_engines1 *3;
end
%
if isequal(aircraft.engines2.Layout_and_config, 4) && ~isequal(aircraft.engines2.Number_of_engines, 0)
    fprintf(fid, '\n+--------------------------------------------------------------------------------');
    fprintf(fid, '\n|    Engines2 layout and configuration %g: straight duct without any attachement', aircraft.engines2.Layout_and_config);
    while (~isequal(input_user_engines2, 0) && ~isequal(input_user_engines2, 1))
    input_user_engines2 = input(...
                 '\n|    Engines2 attached at wing [0] or fuselage [1]? ');
    end
    fprintf(fid,   '+--------------------------------------------------------------------------------');
    aircraft.engines2.Layout_and_config = input_user_engines2 *3;
end
if isequal(aircraft.engines2.Layout_and_config, 5) && ~isequal(aircraft.engines2.Number_of_engines, 0)
    fprintf(fid, '\n+--------------------------------------------------------------------------------');
    fprintf(fid, '\n|    Engines2 layout and configuration %g: S duct without attachement', aircraft.engines2.Layout_and_config);
    while (~isequal(input_user_engines2, 0) && ~isequal(input_user_engines2, 1))
    input_user_engines2 = input(...
                 '\n|    Engines2 attached at wing [0] or fuselage [1]? ');
    end
    fprintf(fid,   '+--------------------------------------------------------------------------------');
    aircraft.engines2.Layout_and_config = input_user_engines2 *3;
end
