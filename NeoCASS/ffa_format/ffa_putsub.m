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

   function  [dseto,ierr] = ffa_putsub(dset,dskid,iptr,isib)
%
% FFA_PUTSUB  build-in a new sub-dataset to an existing FFA dataset
%  
% Usage:     [dso,ie] = ffa_putsub(ds,dsb,[ip[,is]])     
%                                                         DEFAULTS
% Arguments:  ds      FFA-dataset                         
%             dsb     FFA-dataset to be built-in          
%             ip      pointer to "adopting" sub-dataset      [] 
%                     ( e.g. >> ps = ffa_list(ds);   )     "first" 
%                     (      >> ip = ps{4};          )    
%             is      position in among "sibling"            [] 
%                     sub-datasets of "ds"                 "last" 
%		     (integer >0 or empty []        )
%  
% Returns:    dso     new FFA dataset with "dsb" built-in                    
%             ie      =0 on successful completion, >0 on error
% 
% 2002-06-04 J.Smith 
% FFA Matlab Toolbox www.FOI.se
 
% defaults
    ierr=0; dseto=dset;
    if ~exist('iptr')
     iptr = [];
    end
    if ~exist('isib')
     isib=[];
    end 
    
% checks
    if     ~ok_ffa_dataset(dset)
     ierr=1; fprintf('### ffa_putsub: bad dataset\n');
     return 
    elseif ~ok_ffa_pointer(iptr)
     ierr=1; fprintf('### ffa_putsub: bad pointer\n');
     return
    end
    
     
% get reference string of parent 
   [str,sa] = ffa_refstring(iptr);
    if isempty(str)
     ierr=2; dset=[]; fprintf('### ffa_putsub: refstring failed \n')
     return
    end
    
% get "parent" dimension descriptors  
    nnn = ffa_get(dset,iptr,'dims'); 
    nsibs = nnn(3);

% increment number of "child" datasets
    nnn(3)=nnn(3)+1;
    eval(sprintf(' %s3} = nnn;  ',sa));            %  dset{...3} = nnn
    
% get "subz" - cell-array of "sibling" datasets
    eval(sprintf(' subz = %s5}; ',sa));            %  subz = dset{...5}
    
% add new "child" dataset to "subz"
    s = size(subz); 
    if s(1)==nsibs      % check parent & set default "isib"
     if isempty(isib)|isib>nsibs+1
      isib = nsibs+1;
     end
    else
     ierr=3; dset=[]; fprintf('### ffa_putsub: error in parent dataset (nsub) \n')
     return
    end
    
    ns=nsibs+1;         % re-arrange "sub" to fit in the new data
    for j=1:5
     subz{ns,j}=dskid{1,j};         
     for k=ns:-1:isib+1
           tmp{j} = subz{k-1,j};
      subz{k-1,j} = subz{k  ,j}; 
      subz{k  ,j} =      tmp{j};    
     end      
    end
    
% assign the modified "subz" to "dset"
    eval(sprintf(' %s5} = subz; ',sa));            %  dset{...5} = subz

% assign output
    dseto = dset;


%-------------------------------------------------------------------------------
    
    
