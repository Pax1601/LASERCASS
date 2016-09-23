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

   function x = ffa_get(dset,iptr,cid)
%
% FFA_GET     copies data-items from an FFA dataset
%
% Usage:      x = ffa_get(ds,ip[,option])
%
% Arguments:  ds       FFA dataset  
%             ip       pointer (e.g. ip=ps{3} see "ffa_list")     
%             option   string: 'data'  'name'  'type' 'dims' or 'all'
%  
% Returns:    x         data/value specified by "option"
%
%                    --option-----x-output-----------------------
%             DEFAULT  'data' -   data (matrix/string/cell-array)  
%             option=  'name' -   name-string
%                      'type' -   type-string 
%                      'dims' -   [ndim nsiz nsub]   
%                      'all'  -   x{1} = cname              char-16
%                                 x{2} = ctype              char-4
%                                 x{3} = [ndim nsiz nsub]   integer-vector 
%                                 x{4} = data               numeric/string
%
% 2002-04-18 J.Smith 
% FFA Matlab Toolbox www.FOI.se
  
% defaults
    x=[];
    if ~exist('cid')
     cid='data';
    end     
    
% checks 
    if     ~ok_ffa_dataset(dset)
     ierr=1; fprintf('### ffa_get: bad dataset\n');
     return 
    elseif ~ok_ffa_pointer(iptr)
     ierr=1; fprintf('### ffa_get: bad pointer\n');
     return
    end
   
% assemble reference strings from pointer "iptr"
   [str,sa,as] = ffa_refstring(iptr); 
    if isempty(str)
     ierr=4; return
    end
   
% retrieve descriptors and data   
    try
     eval(sprintf(' cname = %s%d%s; ',sa,1,as)); 
     eval(sprintf(' ctype = %s%d%s; ',sa,2,as)); 
     eval(sprintf(' nnn   = %s%d%s; ',sa,3,as)); 
     eval(sprintf(' data  = %s%d%s; ',sa,4,as)); 
    catch
     x=[]; fprintf('### ffa_get: error using "%s"\n',str); return
    end
    
% assign output
    switch cid
     case 'data'
      x = data; 
     case 'name'
      x = cname;
     case 'type'
      x = ctype;
     case 'dims'
      x = nnn;
     case 'all'
      x{1} = cname;
      x{2} = ctype;
      x{3} = nnn;
      x{4} = data;
     otherwise
      x=[]; fprintf('### ffa_get: "%s" not recognised\n',cid)
     end 
