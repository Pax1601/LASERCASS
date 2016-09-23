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

   function [ierr]=ffa_dump(dset,filename,machinetype)
%
% FFA_DUMP   writes an FFA-dataset out to an FFA-format file
%
% Usage:     ie = ffa_dump(ds[,filename,[machinetype]])
%
% Arguments: ds             FFA dataset
%            filename       string     DEFAULT 'ffa_dump.bout'  
%
%            machinetype(*) 'ieee-be'  DEFAULT IEEE floating point big-endian 
%                           'ieee-le'          IEEE floating point little-endian
%                           'native'           local machine format 
%                            ...
% 
% Returns:   ie             0 if successful
%  
% (*)  For a complete list of values for "machinetype" see Matlab's
%      online help information for the standard "fopen" function.
%
% 2002.04.18 J.Smith 
% FFA Matlab Toolbox www.FOI.se

% checks
    if ~ok_ffa_dataset(dset)
     ierr=1; fprintf('### ffa_dump: bad dataset\n');
     return 
    end
    
% defaults
   if ~exist('filename')          % default filename
    filename = 'ffa_dump.bset';
   end
   if ~exist('machinetype')       % default machinetye
    machinetype='ieee-be';
   end

   ifi = fopen(filename,'w',machinetype); 
   if ifi<0
    ierr=1; fprintf('### ffa_dump: error opening file "%s"\n',filename);
    return
   end
   
% recursively upack and write out the data structure  
   [dset,ierr,pds,ilevel] = ffa_recursout(ifi,dset);
   
% close file
   iers = fclose(ifi);
   if iers~=0
    fprintf('### ffa_dump: error closing file "%s"\n',filename);
    ierr = [ierr iers];
   end
    
    
% ----------------------------------------------------------------------------
   function [dset,ierr,pds,ilevel] = ffa_recursout(ifi,dset,ierr,pds,ilevel)
% FFA_RECURSOUT:   FFA-format recursive output  
%
   if ~exist('ierr')          % initialise 
    ierr=0;  ierrloc=0; pds=[]; ilevel=0; ilocal=0; 
   else
    if ierr==0
     ilocal=ilevel;  ierrloc=ierr;
    else     
%     fprintf('### ffa_dump: error on entry\n');              
     return
    end
   end
   
   if length(pds)<ilevel
    ierr=1; fprintf('### ffa_dump: length(pds)<ilevel \n'); 
    return
   end
   
% get data items 
   if isempty(pds)
    for j=1:4
     A{j}=dset{1,j};
    end
   else 
    A = ffa_get(dset,pds(1:ilevel),'all');
   end
   cname=A{1}; ctype=A{2}; nnn=A{3}; data=A{4}; nsub=nnn(3);
   
% write descriptor and data 
   [ierr] = ffa_forto(ifi,cname,ctype,nnn,data);
    if ierr~=0
     fprintf('### ffa_dump: write error at level %d for data "%s"\n',ilevel,cname); 
     return
    end
  
% loop over sub-datasets
   ilocal=ilocal+1;
   for n=1:nsub
    if isempty(pds)
     pds=n;
    else
     pds(ilocal)=n;
    end
% recursive call for sub-dataset
   [dset,ierrloc,pds,ilevel] = ffa_recursout(ifi,dset,ierr,pds,ilocal);
    if ierrloc>0
     ierr=ierrloc; return
    end
   end   
   ilocal=ilocal-1;   
    
   if exist('ierr')
    lerr = ierrloc;
   end
   if exist('ilevel')
    ilevel = ilocal;
   end

   return
   
   
