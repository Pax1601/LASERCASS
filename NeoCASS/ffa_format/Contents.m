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
%  FFA MATLAB TOOLBOX - CONTENTS
%
%  Command Functions
%
%  Import/Export
%   ffa_load       -   read an FFA-format file into Matlab
%   ffa_dump       -   write a Matlab FFA-dataset out to a file 
%
%  Inspection
%   ffa_list       -   list dataset contents / get pointers
%   ffa_diff       -   compare two datasets
%
%  Data Access
%   ffa_get        -   extract items from a dataset
%   ffa_put        -   modify/place items in a dataset
%   ffa_find       -   find sub-datasets by name type or dimesion
%   
%  Datset Manipulation
%   ffa_create     -   creates a new dataset with no sub-datasets
%   ffa_getsub     -   extract a complete sub-dataset
%   ffa_putsub     -   build-in a new sub-dataset
%   ffa_delsub     -   delete a sub-datset 
%
%
%  Low-level functions
%
%   ffa_check_data -   check data size and type against descriptor 
%   ffa_forti      -   read a single Fortran record
%   ffa_forto      -   write a single Fortran record
%   ffa_refstring  -   convert pointer to cell-reference string
%   ok_ffa_dataset -   quick check for dataset
%   ok_ffa_pointer -   quick check for pointe
%-------------------------------------------------------------------------------r 
%
%  HELP    For more information enter "ffa" at the Matlab prompt.
%
%
% FFA Matlab Toolbox
% Author: Jonathan.Smith@foi.se 
