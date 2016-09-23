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

   function  ok = ffa_check_data(ctype,nsiz,ndim,data)
%
%  FFA_CHECK_DATA   check data size and type against FFA descriptor
%
%  Usage
%               ok = ffa_check_data(ctype,nsiz,ndim,data)
%
%  Arguments
%               ctype  char*4 FFA type diescriptor 
%               nsiz   data size
%               ndim   data dimension
%               data   data matrix (2D numeric or cell array or single string)
%
%  Returns
%               ok     1 x 2 logical [ size_ok type_ok ]
%
%
% 2007-03-28 J.Smith 
% FFA Matlab Toolbox www.FOI.se

   if ~(ischar(ctype)&length(ctype)>=1)
    fprintf('### ffa_check_data: invalid cname\n'); return
   end

   c = upper(ctype(1));
   
   s = size(data);                                        % SIZE check
   
   if  isempty(data)                                      %    - no data 
    size_ok = nsiz*ndim==0; 
    
   elseif ischar(data)                                    %    - single string 
    if s(1)~=1
     fprintf('### ffa_check_data: bad string \n')
     disp(inpt{n})
     fprintf('###\n\n')
     ierr=1;return
    end
    size_ok = ndim==1 & nsiz==1; 
 
   else                                                   %    - other
    size_ok = nsiz==s(1) & ndim==s(2);
   
   end  
    
                                                          % TYPE check   
   if ischar(data)                                        %      - single string
    type_ok = (s(2)==1&c=='A')|(s(2)==16&c=='S')|(s(2)==72&c=='L');
    
   elseif iscell(data)                                    %      - cell array of strings   
    type_ok = c=='A'|c=='S'|c=='L'; 
    if iscell(data)&type_ok&size_ok                       
     for i=1:nsiz
      for j=1:ndim
       str = data{i,j}; 
       ss = size(str);
       if ~ischar(str)|ss(1)~=1
        fprintf('### ffa_check_data: bad string in cell-aray \n')
        ierr=1;return
       end
       type_ok = type_ok & (ss(2)==1&c=='A')|(ss(2)==16&c=='S')|(ss(2)==72&c=='L');
      end
     end
    end
   else                                                   %      - numeric data                               
    type_ok = ~isempty(findstr(c,'BIRDCZ'));
   end                                         

   if (nsiz*ndim)==0     % ANY type is ok for an EMPTY dataset.
    type_ok = ~isempty(findstr(c,'NSLABIRDCZ'));
   end 
   
   ok = [ size_ok type_ok ];

   
% Note: should add checks for data contents matching types 
%       ( e.g. if "x" is integer data fix(x)==x )

