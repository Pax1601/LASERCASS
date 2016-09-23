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

   function [ds,ps,ie] = ffabload(filen,opt,machinetype)
%   
%  FFABLOAD loads an FFA dataset, gets the locator array and lists it
%   
%  Usage:   [ds,ps,ie] = ffabload(filename[,opt[,machinetype]])
%   
%  Arguments:   filename       filename string
%               opt            'mute' or ''
%
%               machinetype(*) DEFAULT is 'ieee-be'  ( for Experts ! )
%   
%  Returns:     ds             cell array FFA dataset 
%               ps             cell array index to dataset
%               ierr           >1 on error, =0 otherwise
% 
%  Note:         
%  
%  The command   
%
%  >> [ds,ps] = ffabload('big.bmsh'); 
% 
%  is equivalent to 
%          
%  >> ds = ffa_load('big.bmsh');  ps = ffa_list(ds);
% 
% 
%  See also :  ffa_load, ffa_list, ffaaload
% 
% (*)  For a complete list of values for "machinetype" see Matlab's
%      online help information for the standard "fopen" function.
%
% 2004-02-06 J.Smith 
% FFA Matlab Toolbox www.FOI.se

   ds={}; ps={}; ie=0;
   
   if ~exist('opt')
     opt = '';
   end 
   
   if ~exist('machinetype')
    machinetype='ieee-be';
   end
   
   [ds,ie] = ffa_load(filen,machinetype);
  
   if ie~=0
     fprintf('### ffabload: failed to load: "%s"\n\n',filen);
     return
   end 
   
   if nargout>1
     if strcmp(opt,'mute')
       [ps,ie] = ffa_list(ds,'mute');
     else
       [ps,ie] = ffa_list(ds);
     end
   end
   
   if ie~=0
     fprintf('### error in ffabload\n\n');
   end 
   

   
