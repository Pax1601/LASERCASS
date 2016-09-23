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

    function [dsub,ierr] = ffa_getsub(dset,iptr)
%
% FFA_GETSUB:  copies a "child" sub-dataset from an FFA dataset
%
% USAGE:     [dsub,ie] = ffa_getsub(ds,ip)
%
% Arguments:  ds     FFA dataset
%             ip     pointer to sub-dataset  (e.g. ip=ps{3} see "ffa_list")                                    
%             
% Returns:    dsub   FFA dataset - selected "child" sub-dataset of "ds"               
%             ie     =0 if successful
%
% 2002-04-18 J.Smith 
% FFA Matlab Toolbox www.FOI.se
 
   ierr = 0;
 
% checks
    if     ~ok_ffa_dataset(dset)
     ierr=1; fprintf('### ffa_getsub: bad dataset\n');
     return 
    elseif ~ok_ffa_pointer(iptr)
     ierr=1; fprintf('### ffa_getsub: bad pointer\n');
     return
    end
   
% translate pointer into reference string
   [str,sa,as] = ffa_refstring(iptr);
   
% get data
   for n=1:5 
    estr = sprintf('dsub{%d} = %s%d%s;',n,sa,n,as);
    try
     eval(estr);
    catch
     ierr=3; fprintf('ffa_getsub: error using %s \n',estr)
     dsub=[]; return
    end
   end
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

