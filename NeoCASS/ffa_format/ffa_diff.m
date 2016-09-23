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

    function [idiff,itab,names,delta] = ffa_diff(dsetA,dsetB,opt)
%
% FFA_DIFF  compare data items in two FFA datasets 
%
% Usage:     [idiff,itab,names,delta] = ffa_diff(ds1,ds2[,option])   
%                                    
%
% Arguments:      ds1, ds2    FFA datasets
%                 option      options: 'mute' turns of screen output
%                             DEFAULT is opt='none' or ''       
%                
% Returns:        idiff    0  datasets are identical (tolerance fixed by type)
%                          1  " have same structure but not all items match
%                          2  " contian internal errors 
%                          3  " are of different size
%                          4  " have same size but different structures
%
%                 itab     either: logic-matrix as in Screen Output ( idiff <= 1 )
%                          or:     empty                            ( idiff  > 1 )  
%
%                 names    names of sub-datasets
%
%                 delta    delta values for sub-datasets
% 
%
% Screen Output:  If the two datasets have compatible structures then a
%                 "logic table" is displayed showing which of the names 
%                  types and data-items in the two datasets match up. 
%
%                 Any compatible(*) non-matching numeric data is flagged with 
%                 the value:
%                            delta = 0                         if a=b=0
%                                  = |a-b|/(((a^2+b^2)/2)^.5)  otherwise 
%
%                 where a & b are the pair of data values with maximum |a-b|
%
%             (*) Data is "compatible" if  the array sizes agree and "matching" 
%                 if "delta" is less than the resolution:  0     'I'  integer
%                                                          1e-9  'R'  real*4
%                                                          1e-16 'D'  real*8  
% 2004-02-03 J.Smith 
% FFA Matlab Toolbox www.FOI.se

% defaults
    idiff=[];itab=[];
    if ~exist('opt')
     opt='';
    elseif strcmp(opt,'none')
     opt='';
    end
    if ~(strcmp('',opt)|strcmp('mute',opt))
     fprintf('### ffa_diff: invalid option \n')
    end
    
% checks
    if ~ok_ffa_dataset(dsetA)
     ierr=1; fprintf('### ffa_diff: bad dataset (first)\n\n');
     return 
    elseif ~ok_ffa_dataset(dsetB)  
     ierr=1; fprintf('### ffa_diff: bad dataset (second)\n\n');
     return 
    end

% get pointers for the two datasets

    [pdsA,ierrA] = ffa_list(dsetA,'mute'); LA=length(pdsA);
    [pdsB,ierrB] = ffa_list(dsetB,'mute'); LB=length(pdsB);
    
    if ~(ierrA==0&ierrB==0)
     idiff=2;
     if ~strcmp(opt,'mute')
      fprintf('### ffa_diff: %d. datsets have internal errors \n',idiff);
     end
     return
    end
    
    if LA~=LB
     idiff=3;
     if ~strcmp(opt,'mute')
      fprintf('### ffa_diff: %d. datasets are of different size \n',idiff);
     end
     return
    end
    
    for j=2:LA
     if length(pdsA{j})==length(pdsB{j})
      pp(j)=max(pdsA{j}~=pdsB{j});
     else
      pp(j)=0;
     end
    end  

    if max(pp)~=0
     idiff=4;
     itab=find(pp);
     if ~strcmp(opt,'mute')
      fprintf('### ffa_diff: %d. datasets have non-matching structures \n',idiff);
     end
     return    
    end
    
% print/compile check table & set arguments 
    if ~strcmp(opt,'mute')
     fprintf('\n     n  name1            all    name type size ndim nsub data         \n\n')
    end
    itab=[];
    for n=1:LA % for(n) loop over sub-datasets
     A = ffa_get(dsetA,pdsA{n},'all');
     B = ffa_get(dsetB,pdsB{n},'all');
     nameA=A{1}; typeA=A{2}; nnnA=A{3}; dataA=A{4}; 
     nameB=B{1}; typeB=B{2}; nnnB=B{3}; dataB=B{4}; 
     
     s_name = strcmp(nameA,nameB);         % logical: same name 
     s_type = strcmp(typeA,typeB);         % logical: same type
     s_dims = nnnA==nnnB;                  % logical: same dimensions
     names{n} = nameA;
                                
     if  compatible(dataA,dataB)           % check data matches (begin) 
      numeric=0;                              
      if ischar(dataA)&ischar(dataB)       % - single string   
       s_data = strcmp(dataA,dataB);
      elseif iscell(dataA)&iscell(dataB)   % - multiple strings
       s_data = 0;
       sA = size(dataA);
       for p=1:sA(1)
        for q=1:sA(2)
	 s_data = s_data & strcmp(dataA{p,q},dataA{p,q});
	end
       end     
      elseif isempty(dataA)&isempty(dataB) % - empty
       s_data =1;
      else                                 % - numeric
       numeric=1;
       if typeA(1)==typeB(1)
       [idiff,del]=mynus(typeA,dataA,dataB);
       else
       [idiff,del]=mynus( 'I' ,dataA,dataB);
       end 
       s_data = idiff==0;
      end
     else                  
      s_data = 0;                          %
     end%compatible                        % check data matches (-end-) 
     
     s_all = s_name&s_type&s_dims(1)&s_dims(1)&s_dims(1)&s_data;  % comparison parameters
     nn = [s_all s_name s_type s_dims s_data];                    %
     itab = [itab;nn];                                            %
     if ~strcmp(opt,'mute')                                       % print  ""   ""   ""
      if ~s_data & numeric
       if isempty(del)
        fprintf('%6d  %16s   %d    %d    %d    %d    %d    %d    %d   --  \n',n,nameA,nn) % not the same size
       else
        fprintf('%6d  %16s   %d    %d    %d    %d    %d    %d    %d   %5.0e \n',n,nameA,nn,del)
       end
      else
       fprintf('%6d  %16s   %d    %d    %d    %d    %d    %d    %d       \n',n,nameA,nn)
      end
     end

     if ~exist('del')|isempty(del)
       delta(n) = -1;
     else
       delta(n) = del;
     end     

    end% for(n) loop over sub-datasets
    
    if min(min(itab))==0         
     idiff=1;                 % datasets differ
    else
     idiff=0;                 % datasets identical (within tolerance)
    end
    
    if ~strcmp(opt,'mute')    % finish print
     fprintf('\n                           ')
     fprintf('%d    %d    %d    %d    %d    %d    %d',min(itab))
     if min(itab(:,7))==0
      fprintf('   DELTA \n')
     else
      fprintf('\n')
     end
    end
    fprintf('\n\n')
    
%------------------------------------------------------------------------------- 
   function ok = compatible(a,b)
% check if a & b can be compared
     sa=size(a); sb=size(b);    % check size 
     if length(sa)==length(sb); 
      ok=1;
     else
      ok=0;
     end
     if ok
      if    iscell(a)           % check class
       ok = iscell(b);
      elseif ischar(a)
       ok =  ischar(b);
      elseif isempty(a);
       ok =  isempty(b);
      else   
       ok= ~(ischar(b)|iscell(b)|isempty(b));
      end
     end
%-------------------------------------------------------------------------------
     function  [idiff,delr,dela] = mynus(type,X1,X2,tols)
% MYNUS    type-dependent comparison of two 2D "double" arays
%
% Usage:      [idiff,dela,delr] = mynus(type,X1,X2[,tols])
%
% Arguments:    type     string starting with 'I' 'R' or 'D' (ignores case)
%               X1,X2    2D "double" arrays of the same size
%               tol      vector of tolerance values for types I R D 
%
% Returns:      idiff    0  if X1 & X2 are : identical 
%                        1                 : same size, data differs
%                        2                 : sizes differ
%                       <0 on error
%               dela     max positive difference - absolute
%               delr     max positive difference - relative
%
% 2002.01.24 J.Smith.
%-------------------------------------------------------------------------------

% set defaults
     idiff=0; dela=[]; delr=[];
     if ~exist('tols')
      tols = [ 0 1e-9 1e-16 ];
     end

% check type and set tolerance
     type_ok = ischar(type) & ~isempty(type);      
     if type_ok               
      type = upper(type(1));                    
      switch type             % set tolerance
       case 'I'               %  - integer
        tol=tols(1);          %
       case 'R'               %  - real*4
        tol=tols(2);          %
       case 'D'               %  - real*8
        tol=tols(3);
       otherwise              % unrecognised type - treat as integer (zero tolerance)
        tol=0;
      end
     else                    
      idiff=-2;return 
     end  
     
     S1=size(X1); S2=size(X2);        % get array sizes
     
     if length(S1)==2&length(S2)==2   % check arrays are 2D
      if ~isequal(S1,S2)              % check sizes match
       idiff=2;return 
      end
     else
      idiff=-3;return 
     end
     
      dX = abs(X1-X2);                 % locate max diference
     maX = max(max(dX));
    [i,j] = find(dX==maX); 
     if ~(isempty(i)|isempty(j))
      i=i(1); j=j(1);      % ignore multiple maxima               
     else
      idiff=-4;return      % exit if no maxima (wierd)
     end     

     x1=X1(i,j);           % values at max difference
     x2=X2(i,j); 
     
     dela = abs(x1-x2);    % return variables "dela" & "delr"
     D=((x1^2+x2^2)/2)^(.5);      
     if D>0                %
      delr = dela/D;       %
     else                  %
      delr = 0;            %
     end                   %

     if type=='I'  % return variable "idiff"
      idiff = ~(dela==tol);
     else
      idiff = ~(delr<tol);
     end
     
     
     
%-------------------------------------------------------------------------------     
 
       
     
    
