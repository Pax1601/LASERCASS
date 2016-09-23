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

   function ffa_helpdesk(opt)
%
% FFA_HELPDESK   invokes the HTML online help for FFA-Matlab-Tools
%
% Usage:
%
%     >> ffa_helpdesk                 :  locates the HTML manual
%
%     >> ffa_helpdesk('dat')          :  copies "example.bfil"
%                                               to the current directory
%
% 2005-06-07 J.Smith 
% FFA Matlab Toolbox www.FOI.se
             
   if ~exist('opt')
    opt = 'none';
   end

   pathfinder = 'ffa_pathfinder_for_install_location_19590411';
   filename = which(pathfinder);
   k = strfind(filename,pathfinder);
   ffa_install_directory = filename(1:k-2);   
   html_index = [ ffa_install_directory '/manual/html/index.html' ];
   
   if      strcmp(opt,'dat')      % copy "example.bfil"
   
    estr = sprintf('! cp %s/manual/ffa-data/example.bfil .\n',ffa_install_directory);
    eval(estr);
    fprintf('>> ! pwd; ls -l example.bfil\n')
    ! pwd; ls -l example.bfil
    
   elseif  strcmp(opt,'none')     % open the HTML manual
     
     fprintf('\n   The FFA Matlab Toolbox manual can be found at www.foi.se/edge \n') 
     fid = fopen(html_index);
     if fid>0
     fprintf('   A local copy is installed here:\n   %s \n\n',html_index)
     else
      fprintf('   There should also be a local copy of the HTML manual but it''s not here!\n   %s\n\n',html_index)
     end
   
   else                          % exit on error
    fprintf('### ffa_helpdesk: unrecognised argument\n')
   end
   
   
   
