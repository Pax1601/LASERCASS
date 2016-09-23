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

   function [str,sa,as] = ffa_refstring(iptr,level)
%
% FFA_REFSTRING  translate an "ip" pointer into a "cell-tree" referemce string
%
% Usage:    [str,sa,as] = ffa_refstring(ip[,level])
%
% Examples:  
%            input:  ip  = [3 4 2]               
%            gives:  str = 'dset{1,5}{4,5}{2,:}        
%                    sa  = 'dset{1,5}{4,5}{2,'      
%                    as  = '}'                      
%
%            input:  ip = []
%            gives:  str= 'dset{1,:}'
%                    sa = 'dset{1,'
%                    as = '}'
%
% 2002-04-18 J.Smith 
% FFA Matlab Toolbox www.FOI.se

% checks
    if ~ok_ffa_pointer(iptr)
     fprintf('### ffa_refstring: invalid pointer\n')
     str=[];as=[];sa=[]; return
    end

% defaults
    if ~exist('level')
     level=length(iptr);
    end
    
% construct reference string
    if isempty(iptr) | level==0
     str='dset{1,:}';
    else
     str='dset{1,5}';
     for n=1:level-1
      str=sprintf('%s{%d,5}',str,iptr(n));
     end  
     str=sprintf('%s{%d,:}',str,iptr(level)); 
    end
    k=findstr(str,':');       % split the reference string at the ":" to get
    sa=str(1:k-1);            % "sa" < opening string
    as=str(k+1:length(str));  % "as" > closing string
    
    return
