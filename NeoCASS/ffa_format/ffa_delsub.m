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

   function  [dseto,ierr] = ffa_delsub(dset,iptr)
%
% FFA_DELSUB   delete a sub-dataset
%  
% Usage:     [dso,ie] = ffa_delsub(ds,ip)     
%  
% Arguments:  ds      FFA-dataset 
%             ip      pointer to the sub-dataset which is to be deleted 
%                     ( e.g. >> ps = ffa_list(ds);   )    
%                     (      >> ip = ps{4};          )   
%  
% Returns:    dso     new FFA-dataset (reduced by the specified deletion)      
%             ie      =0 on successful completion, >0 otherwise
%
%
% 2002.06.14 J.Smith 
% FFA Matlab Toolbox www.FOI.se
  
% defaults
    ierr=0; dseto=dset;  
    
% checks
    if     ~ok_ffa_dataset(dset)
     ierr=1; fprintf('### ffa_delsub: bad dataset\n');
     return 
    elseif ~ok_ffa_pointer(iptr)
     ierr=1; fprintf('### ffa_delsub: bad pointer\n');
     return
    end
    if isempty(iptr)
     ierr=1; fprintf('### ffa_delsub: cannot delete the root dataset \n');
     return
    end
     
% get pointer to "parent" dataset
    L=length(iptr); 
    if L==1
     pptr = [];         
    elseif L>1
     pptr = iptr(1:L-1); 
    end    
% get index of "child" dataset to be deleted
    junk = iptr(L);
     
% get reference string of parent 
   [str,sa] = ffa_refstring(pptr);
    if isempty(str)
     ierr=2; fprintf('### ffa_delsub: refstring failed \n')
     return
    end
    
% get "parent" dimension descriptors  
    nnn = ffa_get(dset,pptr,'dims');

% decrement number of "child" datasets
    nnn(3)=nnn(3)-1;
    eval(sprintf(' %s3} = nnn;  ',sa));            %  dset{...3} = nnn
    
% get "subz" - cell-array of "sibling" datasets
    eval(sprintf(' subz = %s5}; ',sa));            %  subz = dset{...5}
    
% remove cells for selected "child" from "subz"
    s = size(subz);  
    k = find([1:s(1)]~=junk); 
    if isempty(k)
     subt={};
    else
     for i=1:length(k)
      for j=1:s(2)
       subt{i,j}=subz{k(i),j};
      end
     end
    end
    
% assign the modified "subz" to "dset"
    eval(sprintf(' %s5} = subt; ',sa));            %  dset{...5} = subz
        
% assign output
    dseto = dset;

%-------------------------------------------------------------------------------
    
    
