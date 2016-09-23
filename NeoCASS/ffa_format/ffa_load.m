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

   function [dset,ierr] = ffa_load(filename,machinetype)
%
% FFA_LOAD   loads data from an FFA-format BIANRY file.   
% 
% Usage:     [ds,ie] = ffa_load(filename[,machinetype]) 
%
% Arguments: filename       string e.g. 'massive.bmsh'
%
%            machinetype(*) 'ieee-be'  DEFAULT  IEEE floating point big-endian 
%                           'ieee-le'           IEEE floating point little-endian
%                           'native'            local machine format 
%                            ...
%
% Returns:   ds             FFA dataset  
%            ie             =0 if successful
%  
% (*)  For a complete list of values for "machinetype" see Matlab's
%      online help information for the standard "fopen" function.
%
% 2002.04.18 J.Smith 
% FFA Matlab Toolbox www.FOI.se

   dset=[]; pds=[]; ierr=0;

% default the machine type
   if ~exist('machinetype')
    machinetype='ieee-be';
   end
   
% open file and do some routine checks
   ifi = fopen(filename,'r',machinetype); 
   if ifi<0
    ierr=1;fprintf('### ffa_load: error opening file "%s"\n',filename);
    return
   end
   
% recursively read and assmeble the data structure  
  [dset,ierr,pds,ilevel] = ffa_recursin(ifi); 
   
% close file
   iers = fclose(ifi);
   if iers~=0
    fprintf('### ffa_load: error closing file "%s"\n',filename);
    ierr = [ierr iers];
   end
    
% ----------------------------------------------------------------------------
   function [dset,ierr,pds,ilevel] = ffa_recursin(ifi,dset,ierr,pds,ilevel)
% FFA_RECURSIN:   recursive input of data from an FFA-format file.
%                 - coding follows subroutine "ffreds.f" in the FFA suite
%
 
   if ~exist('ierr')          % initialise 
    ierr=0;  ierrloc=0; pds=[]; ilevel=0; ilocal=0; dset=[];
   else
    if ierr==0
     ilocal=ilevel;  ierrloc=ierr;
    else     
%     fprintf('### ffa_load: error on entry\n'); % dev              
     return
    end
   end
   
   if length(pds)<ilevel
    ierr=1; fprintf('### ffa_load: length(pds)<ilevel \n'); return
   end
   
% read dataset DESCRIPTOR
  [rec,ierr] = ffa_forti(ifi);  
  if ierr~=0                      
   fprintf('### ffa_load: error reading descriptor \n')
   return
  else
   cname=rec{1}; 
   ctype=rec{2}; 
    ndim=rec{3}(1); 
    nsiz=rec{3}(2); 
    nsub=rec{3}(3); nnn=[ndim nsiz nsub];
  end
  
% read dataset DATA 
  [data,ierr] = ffa_forti(ifi,ctype,ndim,nsiz); 
   if ierr~=0                      
    ierr=1; fprintf('### ffa_load: error reading data: %s %d %d %d <<%s>> \n',ctype,nnn,cname)
    return
   end

% create dataset entry 
  [dset,ierr] = ffcreate(dset,pds,cname,ctype,nnn,data,ilevel);
  
% loop over sub-datasets
   ilocal=ilocal+1;
   for n=1:nsub
    if isempty(pds)
     pds=n;
    else
     pds(ilocal)=n;
    end
% recursive call for sub-dataset
   [dset,ierr,pds,ilevel] = ffa_recursin(ifi,dset,ierr,pds,ilocal);
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
   
%-------------------------------------------------------------------------------   
  function [dset,ierr] = ffcreate(dset,pds,cname,ctype,nnn,data,level)
% FFCREATE   creates an entry in a "cell-tree" image of an FFA dataset 
%
% Arguments:   dset       "cell tree" FFA datset 
%              pds        "pointer" to dataset
%              cname      data name string :  16 characters 
%              ctype      data type string :   4 characters 
%              nnn        [ndim nsiz nsub] : integer 3-vector  
%              data       data matrix      : numeric array or []
%              level      level in dataset : integer >= length(pds)   DEFAULTED
%    
%  Returns:    dset       updated dataset
%              ierr       error code
%    
%  NOTE
%  There are no "traps" to stop this function creating inconsistent entries. 
%
%  2002.01.11 J.Smith.  FFCREATE 
% 

% defaults
  ierr=0;
  if ~exist('level')
   level=length(pds);
  end

% checks
  if level>length(pds) 
   ierr=20; fprintf('### ffa_load: level=%d > length(pds)=%d \n',level,length(pds))
   return
  end 
  
% assemble reference strings from pointer "pds"
  [str,sa,as] = ffa_refstring(pds,level); 
   if isempty(str)
    ierr=4; return
   end
  
% create database entries 
  estr{1}=sprintf(' %s1%s=''%s'';     ',sa,as, cname ); 
  estr{2}=sprintf(' %s2%s=''%s'';     ',sa,as, ctype ); 
  estr{3}=sprintf(' %s3%s=[%d %d %d]; ',sa,as, nnn   ); 
  estr{4}=sprintf(' %s4%s=%s;         ',sa,as,'data' ); 
  estr{5}=sprintf(' %s5%s=cell(%d,1); ',sa,as, nnn(3)); 
  for n=1:5
   try
    eval(estr{n})
   catch
    ierr=10+n; fprintf('### ffa_load: error using "%s" \n',estr{n});
    return
   end
  end
  
    
