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

   function [ds,ps] = ffaaload(fname,opt)
%
%  FFAALOAD
% 
%  Load a dataset in FFA ASCII format, using 'ffaa2b' and 'ffa_load' 
%
%  The program 'ffaa2b' is part of the Edge package and 'ffa-load'
%  is the basic binary load function in the FFA Matlab Toolbox.
%
%  Usage:
%
%        [ds,ps] = ffaaload(filename,[option])
%
%         ds     :   dataset
%         ps     :   locator array
%
%  Option:
%
%       'mute' supresses detailed screen messages
%
%  Example:
%
%       [ds,ps] = ffaaload('massive.ainp','mute')
%
%  Error:
%       ds & ps are returned empty on error
%
%
% 2004-02-06 J.Smith 
% FFA Matlab Toolbox www.FOI.se

   ds=[]; ps=[]; ie=0;
   if ~exist('opt')
     opt = '' ;
   end  
   if strcmp(opt,'')
     mute = 0;
   elseif strcmp(opt,'mute')
     mute = 1;
   else
     fprintf('### ffaaload: bad input "%s" \n\n',opt)
     return
   end

   fid = fopen(fname);   
   if fid<0
     fprintf('### ffaaload:  file "%s" is not readable\n\n',fname)
     return
   else
     fclose(fid);
   end
  
   if mute 
   
     str1 = sprintf('! ffaa2b %s ffaaload.buzz > ffaaload.tmp',fname);
     str2 = sprintf(' [ds,ie] =  ffa_load(''ffaaload.buzz''); '     );
     str3 = sprintf('!  rm -f  ffaaload.buzz ffaaload.tmp'          );   
     eval(str1); 
     eval(str2); 
     eval(str3);
   
   else
   
   str1 = sprintf('!  ffaa2b  %s           ffaaload.buzz' ,fname);
   str2 = sprintf('  [ds,ie] =  ffa_load(''ffaaload.buzz'');   ');
   str3 = sprintf('!  rm -f                ffaaload.buzz'       );   
   fprintf([ '>>  ' str1 '\n']), eval(str1);
   fprintf([ '>>  ' str2 '\n']), eval(str2);
   if ~isempty(ds)
   fprintf([ '>>  ' str3 '\n']), eval(str3);
   end
   end
     
   if ie~=0
     fprintf(  '### ffaaload: error loading file "%s" \n\n',fname)
     return
   end
      
   if ~ok_ffa_dataset(ds);return; end 
   
   if mute
     ps = ffa_list(ds,'mute');
   else
     ps = ffa_list(ds);
   end
   
