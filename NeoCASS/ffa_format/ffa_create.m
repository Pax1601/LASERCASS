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

   function [ds] = ffa_create(cname,ctype,data)
%  FFA_CREATE   create a single FFA dataset with no sub-datasets
% 
%                                                       DEFAULTS
%  Usage:      ds = ffa_create(cname,ctype,data) %         - 
%        or    ds = ffa_create(cname)            %  ctype='N';  data=[];
%        or    ds = ffa_create                   %  cname='Nameless'; 
%	                                           
% 
%  Arguments:  cname   data name string (internally trimmed to 16 chars)
%              ctype   data type string (FFA-format descriptor)
%              data    data matrix
% 
%  Returns:    ds      FFA-dataset  (on error an empty cell-array is returned)
%
%
% 2003.10.23 J.Smith 
% FFA Matlab Toolbox www.FOI.se

   if ~exist('ctype')
    ctype = 'N';
    data = [];
   end
   if ~exist('cname')
    cname = 'Nameless';
   end
   
   if iscell(data) % (added 2003-10-23) 
    fprintf('### ffa_create: bad data (must be string or numeric)\n');
    ds={}; return
   end
     
%   try
   
    cname = trim(cname,16);
    ctype = trim(upper(ctype),4); 
    
    if ~ischar(data) % numeric data
     nnn = [fliplr(size(data)) 0];
     nsiz = nnn(2);
     ndim = nnn(1);
     
    else             % character data            (added 2003-10-23)              
     nsiz=1; ndim=1; nnn = [ ndim nsiz 0 ];
     switch ctype(1) 
      case 'A'
       data = trim(data,1);
      case 'S'
       data = trim(data,16);
      case 'L'
       data = trim(data,72);
      otherwise
       fprintf('### ffa_create: bad type "%s" for string data\n',ctype(1));
       ds={}; return
     end
    end
    
    ok = ffa_check_data(ctype,nsiz,ndim,data);
    
    ds = {cname,ctype,nnn,data,{}};
    
    if ok_ffa_dataset(ds)
      ierr=0;
    else
     fprintf('### ffa_create: bad dataset!\n');
     ds={}; return
    end
    if ~ok(1)
     fprintf('### ffa_create: size mismatch\n');
     ds={}; return 
    end
    if ~ok(2)
     fprintf('### ffa_create: bad type: %s\n',ctype);
     ds={}; return  
    end
    
%   catch 
   
%    fprintf('### ffa_create: weird internal error\n');
%    ds={};

%   end
   
   

%------------------------------------------------------------------------------
   function str = trim(str,N)
% TRIM   strip leading and trailing blanks from a string
%        and right pad/trim it to length N

% 2002.04.10 updated to return blank strings of length N if str is empty or blank

   invalid = ~ischar(str);
     empty = isempty(str);
     blank = length(findstr(str,' '))==length(str);

   if invalid
    str = ''; return
   end
   
   if empty|blank
    str='';
    for n=1:N
     str = sprintf('%s ',str);
    end
    return
   end
   
   for k=1:2
    while strcmp(str(1),' ')
     L=length(str);
     if L>=2
      str = str(2:L);
     end
    end
   str=fliplr(str);
   end
   
   L=length(str);
   
   if L<N
    while length(str)<N
     str=[str ' '];
    end
   elseif L>N
    str = str(1:N);
   end
   
