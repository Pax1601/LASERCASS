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

   function str = shj_trim(str,N)
%
% SHJ_TRIM   strip leading and trailing blanks from a string
%            and (optionally) right-pad/trim it to a fixed length
%
% Usage 
%           strout = shj_trim(strin[,length])
%
% Arguments 
%           strin   : input string
%           length  : fixed length for output string   OPTIONAL
%
% Returns
%           strout  : output string  (empty on error)
%
% 2002-08-13 J.Smith 
% FFA Matlab Toolbox www.FOI.se

   if ~exist('N')
    fixed_length=0; N=0;
   else
    fixed_length=1;
   end


   invalid = ~ischar(str);
     empty = isempty(str);
     blank = length(findstr(str,' '))==length(str);

% checks
   if invalid
    str = ''; return
   end
   
%    
   if empty|blank
    str='';
    for n=1:N
     str = sprintf('%s ',str);
    end
    return
   end


% strip blanks left and right   
   for k=1:2     
    while strcmp(str(1),' ')
     L=length(str);
     if L>=2
      str = str(2:L);
     end
    end
   str=fliplr(str);
   end

% right pad/trim if required   
   if fixed_length
    L=length(str);
    if L<N 
     while length(str)<N
      str=[str ' '];
     end
    elseif L>N
     str = str(1:N);
    end
   end
   
