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

    function [pc,in,ierr] = ffa_find(ds,property,value)
%
% FFA_FIND  searches an FFA cell-tree for matching items 
%
% Usage:     [pc,in,ie] = ffa_find(ds,property,value)
%
% Arguments:  ds         FFA dataset
%             property   string: 'name'  'type' or 'dims'  
%             value      value of "property" for which to search
%                        Note:- 
%                        - Values for 'name' are case-sensitive
%                          but values for 'type' are not.
%                        - For 'type' you can use "?" (or a blank) 
%                          to match any single character.
%                        - Value 'dims' is [ndim nsiz] - size last!                     
%             
% Returns:    pc         pointers to matched items      ( 1xN-cell-array ) 
%             in         index in list of input dataset ( 1xN-integers   )  
%             ie         =0 if succesful , =1 on error
%
% If no matching items are found, then "pc" and "in" are returned empty (ie=0).
%
% 2002.08.13 J.Smith 
% FFA Matlab Toolbox www.FOI.se

% defaults
   ierr=0; pc=[];
 
% checks 
   if ~ok_ffa_dataset(ds)
    ierr=1; fprintf('### ffa_find: bad dataset\n');
    return 
   end
   if ~isstring(property)
    ierr=1; fprintf('### ffa_find: bad input for "property" \n');
    return 
   end
    
% process "property" & "value"
   if     strcmp(property,'name') % NAME
    if ~isstring(value);
     ierr=1; fprintf('### ffa_find: bad value for "name" \n',property)
     return
    end
   elseif strcmp(property,'type') % TYPE
    if ~isstring(value)|value(1)==' '|isempty(value);
     ierr=1; fprintf('### ffa_find: bad value for "type" \n',property)
     return
    else
     value = [value '    ']; 
     value = value(1:4);
     value(find(value==' '))='?';           % replace " " with "?"
     mask = setxor(findstr('?',value),1:4); % index non "?" chars 
    end
   elseif strcmp(property,'dims') % DIMS
    ok = strcmp(class(value),'double');
    if ok 
     s = size(value); 
     ok = length(s)==2 & min(s)==1 & min(value>=0)==1;
    end
    if ~ok
     ierr=1; fprintf('### ffa_find: bad value for "dims" \n',property)
     return
    end
   else
    ierr=1; fprintf('### ffa_find: property "%s" not recognised\n',property);
    return 
   end

% get pointers for whole dataset 
   [pds,ierr] = ffa_list(ds,'mute');
   if ierr~=0
    ierr=1; fprintf('### ffa_find: dataset has internal errors\n');
    return 
   end
   
% loop over sub-datasets and search for matching values
   L=length(pds);
   m=0; in=[]; 
   for n=1:L
    iptr = pds{n};
    if     strcmp(property,'name')            % NAME
     name = ffa_get(ds,iptr,'name');
     name = shj_trim(name);
     valu = shj_trim(value);
     match = strcmp(name,valu);
    elseif strcmp(property,'type')            % TYPE
     type = ffa_get(ds,iptr,'type');
     match = strcmp(value(mask),type(mask));
    elseif strcmp(property,'dims')            % DIMS
     dims = ffa_get(ds,iptr,'dims');
     match = dims(1)==value(1) & dims(2)==value(2);
    end
    if match              % store pointer
     m=m+1;
     pc{m} = iptr;
     in(m) = n;
    end
   end
   
%   if m==0
%    fprintf('\n  ffa_find: nothing found\n\n');
%    return 
%   end
   
%-------------------------------------------------------------------------------
   function b = isstring(str)
% check if str is a string
    b = ischar(str);
    if b
     s = size(str);
     b = s(1)==1 & length(s)==2;
    end
%-------------------------------------------------------------------------------
    
    
   
   
   
   
   
   
   
   
   
   
   
   
   

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

