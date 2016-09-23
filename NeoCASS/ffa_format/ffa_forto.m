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

    function [ierr] = ffa_forto(ifi,cname,ctype,nnn,data)
%
% FFA_FORTO:  writes FFA-format(*) descriptor and data records to a file
%
% Usage:       ie = ffa_forto(ifi,cname,ctype,nnn,data)
%                          
% Arguments:   ifi      file identifier from Matlab "fopen" function  
%              cname    data name            16-char string 
%              ctype    FFA type descriptor   4-char string 
%              nnn     [ndim nsize nsub]  
%              data     numeric matrix / string / cell-array of strings    
%    
% Returns:     ie       error code (=0 on successful completion)  
%
% RESTRICTIONS:
%
%  Complex data inputs (types C & Z) are not supported in this release.
%  
% REFERENCES:  
% 
% (*) "Standardized Data Format version 1" FFAP-A-950, 1992, S.Wallin.
%
% 2002-04-18 J.Smith 
% FFA Matlab Toolbox www.FOI.se

   ierr=0; serr=''; ndim=nnn(1); nsiz=nnn(2); nsub=nnn(3);

% checks        
   cname = [cname '                ']; 
   cname=cname(1:16);
   ctype = [ctype '    ']; 
   ctype=upper(ctype(1:4));
   c1 = ctype(1);
   
   if isempty(findstr('BIRDCZASLN',c1));              % check "ctype"
    fprintf('### ffa_forto: type "%s" in "%s" not recognised\n',c,ctype)
    ierr=1;return
   elseif c1=='B'| c1=='C'|c1=='Z' 
    fprintf('### ffa_forto: type "%s" in "%s" not supported\n',c,ctype)
    ierr=2;return
   elseif c1=='A'|c1=='S'|c1=='L'                       
    if (nsiz*ndim>1) & ~iscell(data)              % check character data 
     fprintf('### ffa_forto: data is multi-string but not stored as a cell-array\n') 
     ierr=3;return
    end
   end
   
   
% write DESCRIPTOR record        ( number of bytes in record is 32 = 16 + 4 + 3*4 ) 

    cname = trim(cname,16);
    ctype = trim(ctype,4);
   
    cb  = fwrite(ifi,32,'int32');         % record begin 
    cd1 = fwrite(ifi,cname,'uchar'); 
    cd2 = fwrite(ifi,ctype,'uchar'); 
    cd3 = fwrite(ifi,nnn,'int32'); 
    ce  = fwrite(ifi,32,'int32');         % record -end- 
    
    if [ cb cd1 cd2 cd3 ce ] ~= [ 1 16 4 3 1 ]
     fprintf('### ffa_forto: error writing descriptor\n') 
     ierr=4;return
    end
    
                    
% write DATA record              ( number of bytes in record is variable )
   
    switch ctype(1)                         % set data precision / string-length
     case 'I'
      prec='int32';
     case 'R'
      prec='float32';
     case 'D'
      prec='float64';
     case 'A'
      prec='uchar'; clength=1;
     case 'S'
      prec='uchar'; clength=16;
     case 'L'
      prec='uchar'; clength=72;
     case 'N'
      prec='none';
     otherwise
      fprintf('### ffa_forto: unrecognised type descriptor "%s" in "%s"\n',c1,ctype)
      ierr=5;return
    end     
    
    if     strcmp(prec,'none')|(ndim*nsiz)==0     % write a "NO DATA" data record (!)
      return
     
    elseif strcmp(prec,'uchar')           % write a CHARACTER data record
    
     nbytes = clength*ndim*nsiz;  % Bytes count
     
     cb = fwrite(ifi,nbytes,'int32');   % record begin
      for isiz=1:nsiz
       for idim=1:ndim
        if     ischar(data)     % single string
	 str = data;
        elseif iscell(data)     % multiple strings
         str = data{isiz,idim};
	end
	 str = trim(str,clength);
     cd = fwrite(ifi,str,prec); % write string to file
       end
      end
     ce = fwrite(ifi,nbytes,'int32');   % record -end- 
     
     if [ cb cd ce ] ~= [ 1 nbytes 1 ]  
      ierr=10;fprintf('### ffa_forto: error writing data (character)\n')
      return
     end 
     
    else                                          % write a REAL/INTEGER data record
    
     if strcmp(prec,'float64')  % type "D"    
      nbytes = ndim*nsiz*8;
     else                      % type "I" & type "R" (complex) are not supported
      nbytes = ndim*nsiz*4;
     end
      data=reshape(data,nsiz*ndim,1);
     cb = fwrite(ifi,nbytes,'int32'); % record begin
     cd = fwrite(ifi,data,prec);
     ce = fwrite(ifi,nbytes,'int32'); % record -end- 
     if [ cb cd ce ] ~= [ 1 ndim*nsiz 1 ]
      ierr=11;fprintf('### ffa_forto: error writing data (numeric)\n')
      return
     end 
     
    end                                                          

   return
   
   
   
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
   
