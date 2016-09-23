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

    function [rec,ierr] = ffa_forti(ifi,ctype,ndim,nsiz)
%
% FFA_FORTI:  reads a single Fortran record from a file in FFA-format(*)
%
% Usage: "DESCRIPTOR"  [rec,ie] = ffa_forti(ifi)                   
%        "DATA"        [rec,ie] = ffa_forti(ifi,ctype,ndim,nsiz)   
%                          
% Arguments:   ifi      file identifier from Matlab "fopen" function   
%              ctype    FFA type descriptor  ( 4-char string )
%              ndim     dimension of data    ( integer > 0   )
%              nsiz     size of data         ( integer > 0   )
%    
% Returns:     rec      data/descriptor
%              ie       error code (=0 on successful completion)      
%
% "DESCRIPTOR" rec  is a 5-cell array containing the FFA-format
%                   descriptor-variables: CNAME, CTYPE, NDIM, NSIZ, NSUB. 
% 
% "DATA"       rec  depends on ctype            ctype(1)
%              ---                              --------    
%              numeric 2D-array or scalar       B I R D C Z       
%              string or cell-array of strings  A S L         
%              empty matrix                     N                                   
%
%
% RESTRICTIONS:
%
%  The file must be FFA-format(*) Fortran BIRNARY output created under a 
%  UNIX or LINUX operating system. (Use "ffa_a2b" to convert ascii data.) 
%  
%  Complex data inputs (types C & Z) are not supported in this release.
%  
% REFERENCES:  
% 
% (*) "Standardized Data Format version 1" FFAP-A-950, 1992, S.Wallin.
%
% 2001-04-18 J.Smith 
% FFA Matlab Toolbox www.FOI.se

%-------------------------------------------------------------------------------
   rec=[]; ierr=0; 
% default arguments 
   if ~exist('ctype')     % DESCRIPTOR input - doesn't need "ctype,ndim,nsiz)"
    descriptor_record=1;      
   else                   % DATA input
    descriptor_record=0; 
    ctype_ok = 0;         % check "ctype"
    if isempty(ctype)
     ctype_ok = 1;
    else
     ctype_ok = ~isempty(findstr('BIRDCZASLN',ctype(1)));
    end
    if ~ctype_ok
     ierr=1;fprintf('### ffa_forti: data type "%s" not recognised \n',ctype(1:3))
     return
    end
    if ctype(1)=='B'| ctype(1)=='C'|ctype(1)=='Z' 
     ierr=2;fprintf('### ffa_forti: type "%s" in "%s" not supported \n',ctype(1),ctype)
     return
    end
    ctype = upper(ctype);       % uppercase and pad-out "ctype"
    if length(ctype)<3
     ctype = [ctype '   ']; 
     ctype = ctype(1:4);
    end
   end   
   
   if descriptor_record   % read a DESCRIPTOR record
   
    [ibeg,c] = fread(ifi,1,'int32');      % record begin
    
     if c==0     % end of file
      ierr=-1; 
      return    
     end  
     
    [name,c] = fread(ifi,16,'uchar'); 
       rec{1}=char(name');
    [type,c] = fread(ifi,4,'uchar'); 
       rec{2}=char(type');      
    [innn,c] = fread(ifi,3,'int32');  
       rec{3}=innn; 
    [iend,c] = fread(ifi,1,'int32');      % record -end-    
     
    if c==0     % end of file
     ierr=-1; 
     return    
    end  
    
    if ibeg~=iend
     ierr=4;fprintf('### ffa_forti: error reading descriptor record \n')
     return
    end
                       
   else                   % read a DATA record
   
    switch ctype(1)                   % set data precision / string-length
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
      prec='undefined'; 
      fprintf('### ffa_forti: unrecognised descriptor "%s" \n',ctype(1))
      ierr=2; return
    end 
    
    if     strcmp(prec,'none')|(ndim*nsiz==0)     % read "NO DATA"
                                 
     rec = []; 
     return
     
    elseif strcmp(prec,'uchar')           % read CHARACTER data
    
     [ibeg,c] = fread(ifi,1,'int32'); % record begin
     for isiz=1:nsiz
      for idim=1:ndim
       [str,c]=fread(ifi,clength,prec);
       str=char(str');
       rec{isiz,idim}=str;  
      end
     end
     if nsiz==1 & ndim==1
      rec=rec{1,1};   
     end
     [iend,c] = fread(ifi,1,'int32'); % record -end- 
     
     if c==0     % end of file
      ierr=-1; 
      return    
     end  
     
     if ibeg~=iend
      ierr=4;fprintf(['### ffa_forti: error reading record (character) \n'])
      return
     end
     
    else                                          % read REAL/INTEGER data
    
    [ibeg,c] = fread(ifi,1,'int32'); % record begin
    [data,c] = fread(ifi,ndim*nsiz,prec);
    [iend,c] = fread(ifi,1,'int32'); % record -end- 
     
     rec = reshape(data,nsiz,ndim);  
     
     if c==0     % end of file
      ierr=-1; 
      return    
     end  
     
     if ibeg~=iend
      ierr=5;fprintf(['### ffa_forti: error reading record (numeric) \n'])
      return
     end
     
    end
    
   end                    

   return
   
