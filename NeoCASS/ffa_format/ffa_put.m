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

 function [dseto,ierr] = ffa_put(dset,iptr,c1,i1,c2,i2,c3,i3,c4,i4,c5,i5)
%
% FFA_PUT    modify data-items in a given sub-dataset 
%
%  Usage    [dso,ie] = ffa_put(ds,ip,[opt1,input1[opt2,input2...]...])
%
%  Arguments    ds         FFA dataset
%               ip         pointer       (e.g. ip=ps{3} see "ffa_list") 
%               optN       option string -
%  recommended --------->  - 'auto' replace data and adjust automatically
%                          - 'data' replace data 
%                          - 'dims' replace [ndim nsize]
%                          - 'name' replace data-name 
%                          - 'type' replace data-type descriptor 
%                         
%               inputN     replacement data/string/dimensions/cell-array
%
%  Returns     dso        FFA dataset updated with new inputs
%              ie         =0 if successful, >0 on error
%
%
%  NOTE  When using 'auto' the following adjustments are made.
%
%        Input: string 
%         - data - string trimmed/padded to 1, 16 or 72 characters length
%         - dims - set to [1 1]
%         - type - first character set to 'A' 'S' or 'L' to match string length
%
%        Input: empty matrix 
%         - dims - set to [0 0]
%         - type - set to 'N   '
%
%        Input: full matrix 
%         - dims - set to match array size
%         - type - unchanged - must be set using the 'type' option -
%
%
% 2002-06-28 J.Smith 
% FFA Matlab Toolbox www.FOI.se

% checks
    if     ~ok_ffa_dataset(dset)
     ierr=1; fprintf('### ffa_put: bad dataset\n');
     return 
    elseif ~ok_ffa_pointer(iptr)
     ierr=1; fprintf('### ffa_put: bad pointer\n');
     return
    end
    
% defaults
   dseto=dset;
   ierr=0; 
   if ~exist('c2');c2='';end; if ~exist('i2');i2='';end 
   if ~exist('c3');c3='';end; if ~exist('i3');i3='';end 
   if ~exist('c4');c4='';end; if ~exist('i4');i4='';end 
   if ~exist('c5');c5='';end; if ~exist('i5');i5='';end 
   
% constants : cell-array of allowed input-codes
   code{1}='data'; code{3}='type'; code{5}='adat';
   code{2}='name'; code{4}='dims'; code{6}='auto';
   
% check number of arguments
   if ~ismember(nargin,[4 6 8 10 12])   
    fprintf('### ffa_put: incorrect number of arguments\n')
    ierr=1;dset=[];return
   end
   
% get reference string from pointer
   [str,sa,as] = ffa_refstring(iptr);
   if isempty(str)
    fprintf('### ffa_put: error getting reference string\n')
    ierr=1;dset=[];return
   end
   
% get nnn=[nsiz ndim nsub];
   eval(sprintf(' nnn = %s3};',sa));
   
% get type string;
   eval(sprintf(' ctype = %s2};',sa)); ctype=upper(ctype);
   
% store the cN,iN input pairs in cell-arrays "codi" and "inpt"
   ncip = (nargin-2)/2;          % number of code/input pairs
   for n=1:ncip
    eval(sprintf(' codi{%d}=c%d;  inpt{%d}=i%d;\n',[n n n n]));
   end

% check the code/input pairs 
   codinstr='';
   for n=1:ncip
    if ~ismember(codi{n},code) % check input code is 
     fprintf('### ffa_put: code "%s" not recognised\n',codi{n})
     err=1;dset=[];return
    end
    codinstr = sprintf('%s %s',codi{n},codinstr);
   end
   if      ismember('adat',codi)  % check for conflicting codes
    if     ismember('data',codi) 
     fprintf('### ffa_put: code conflict: "adat" & "data" \n'); 
     err=1;dset=[];return
    elseif ismember('dims',codi) 
     fprintf('### ffa_put: code conflict: "adat" & "dims" \n');
     err=1;dset=[];return
    end 
   end
   for n=1:ncip           
    if length(findstr(codinstr,codi{n}))>1 % check for duplicate codes
     fprintf('### ffa_put: duplicated code "%s" \n',codi{n});
     err=1;dset=[];return
    end
    if     strcmp('type',codi{n}) % check for valid input
     if isempty(findstr('BIRDASLN',inpt{n}(1)))
      fprintf('### ffa_put: type "%s" not supported\n',inpt{n});
      err=1;dset=[];return
     end
    elseif strcmp('dims',codi{n})
     nn=inpt{n}; 
     if ~( min(size(nn))==1 & length(nn)==2 & min( fix(nn)==nn )==1 )
      fprintf('### ffa_put: input for "dims" is invalid \n');
      err=1;dset=[];return
     end
    end
   end 
   
% assign inputs as specified
   for n=1:ncip
   
    try 
    
     if     strcmp('name',codi{n})           % name
      name=trim(inpt{n},16);
      eval(sprintf(' %s1}=name;',sa));
	   
     elseif strcmp('type',codi{n})           % type
      type=trim(upper(inpt{n}),4);
      eval(sprintf(' %s2}=type;',sa));
	   
     elseif strcmp('dims',codi{n})           % nsiz,ndim 
      nnn(1)=inpt{n}(1); 
      nnn(2)=inpt{n}(2); 
      eval(sprintf(' %s3}=nnn;',sa));    
	    
     elseif strcmp('data',codi{n})           % data  
      eval(sprintf(' %s4}=inpt{n};',sa));
	   
     elseif strcmp('auto',codi{n})|strcmp('adat',codi{n})   % data & auto-set 
      data=inpt{n};                                         % (was 'adat' in  
      s=size(data);                                         %  older version)
      
      if ischar(data)                 % single-string data
       if s(1)~=1
        fprintf('### ffa_put: bad character data \n');
        disp(inpt{n});fprintf('###\n\n');ierr=1;return
       end
       nnn(1)=1; nnn(2)=1;            % - size - 1 x 1
       if     s(2)==1                 % - type - A/S/L
        ctype(1)='A';
       elseif s(2)<=16;
        ctype(1)='S'; data=trim(data,16);
       else
        ctype(1)='L'; data=trim(data,72);
       end
       
      elseif isempty(data)            % empty matrix / no data
       nnn(1)=0; nnn(2)=0;            % - size - 0 x 0
       ctype = 'N   ';                % - type - N
       
      else                            % full matrix           
       nnn(1)=s(2); nnn(2)=s(1); 
       
      end
       
      eval(sprintf(' %s2}=ctype;',sa));
      eval(sprintf(' %s3}=nnn;',sa));
      eval(sprintf(' %s4}=data;',sa));
	
     end
      
    catch
    
     fprintf('### ffa_put: error handling "%s" \n',codi{n});
     err=1;dset=[];return
     
    end % try
    
   end
   
% assign output
   dseto = dset;
   
   
%------------------------------------------------------------------------------
  function str = trim(str,N)
% TRIM   strip leading and trailing blanks from a string
%        and right pad/trim it to length N

   invalid = ~ischar(str);
   empty = isempty(str);
   blank = length(findstr(str,' '))==length(str);

   if empty|blank|invalid
    str = ''; return
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
   










