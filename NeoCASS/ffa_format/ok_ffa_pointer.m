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

  function ok = ok_ffa_pointer(iptr)
%
%  OK_FFA_POINTER   quick validity check on a pointer for a Matlab FFA dataset
%  
%  Usage:        ok = ok_ffa_pointer(ip)
%  
%  Arguments:    ds    FFA cell-tree dataset
%  
%  Returns:      ok    1 if OK, 0 otherwise
% 
% 2002-04-18J.Smith 
% FFA Matlab Toolbox www.FOI.se
 
  ok = strcmp(class(iptr),'double');
  if ok & ~isempty(iptr)
   ok = ok & ( min(size(iptr))==1 & length(size(iptr))==2 );
   ok = ok & min( iptr>0 );
  end
