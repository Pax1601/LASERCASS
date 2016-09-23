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
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER           DESCRIPTION
%     081103      1.1     J.Opperlstrupp       Creation
%                         jespero@nada.kth.se
%*******************************************************************************
%
% function set_neocass_path
%
%   DESCRIPTION: Look for .m and .DAT files and include directories if any
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************

function set_neocass_path
ss      = pwd;
dirlist = {}; dirlist{1} = ss;
dirlist = list_subdir_m(ss,dirlist);
[m,n]   = size(dirlist);
for k = 1:n
    path(path,dirlist{k});
end;
rehash; 

  function dirlist = list_subdir_m(pathn,dirlist)
  %
  subdirs = struct2cell(dir(pathn));
  [m,n]   = size(subdirs);
  include_this = 0;
  for k = 1:n
    fn  = deblank(subdirs{1,k});                         
    if ~isequal(fn(end),'.')                             
      if subdirs{4,k}==1                                
        pathn2 = fullfile(pathn,fn);
        dirlist = list_subdir_m(pathn2,dirlist);      
      end
      index = find(lower(fn) == '.');
      if ~isempty(index)
        if ( (isequal(lower(fn(index(end):end)),'.m')) || (isequal(fn(index(end):end),'.DAT')) || (isequal(fn(index(end):end),'.p')))
          include_this = 1;
        end
      end
    end
  end

  if include_this                                          
    dirlist{end+1} = pathn;                               
  end 
  end

end
