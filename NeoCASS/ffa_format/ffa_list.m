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

   function  [pds,ierr,errt] = ffa_list(dset,option)
%
% FFA_LIST   constructs a "pointer array" for an FFA dataset 
%            and prints a listing of it's contents (optional)
%
% Usage:     [ps,ie,et] = ffa_list(ds[,option])
%
% Arguments: ds      FFA dataset 
%            option  'mute' - suppress screen output       
%
% Returns:   ps      N by 1 cell-array of "pointer" vectors -
%                    - pointer for sub-dataset "n" is "ps{n}"
%            et      error table as per "---" entries in listing 
%            ie      =0 if successful and "errt" shows no errors
%
%
% Screen Output:  ( similar to output for Fortran program "ffalist" )
%===================================================================
%   1 N   ---         0 x 0   N/  level_0_name         
%   2 LI  ---         1 x 1         level_1_set_1_name 
%   3 RCF ---   9876543 x 1         level_1_set_2_name 
% ...etc.                                              
%===================================================================
%XXXX                                : index to data item
%     XXXX                           : type descriptor   
%         XXX                        : check code: (data vs descriptor)
%         ---                        :     ---   no error  
%         ---                        :     ?--   data "dims" mismatch
%         ---                        :     -?-   data "type" mismatch
%         ---                        :     --?   error in subset count
%             XXXXXXXXX x X          : "dims"      (nsize x ndimension) 
%                             XX     : no. sub-datasets (blank if none)
%                                 XXX: name of data item 
%===================================================================
%
% 2002-06-28 J.Smith 
% FFA Matlab Toolbox www.FOI.se

% defaults    
    pds=[]; errt=[]; 
    if ~exist('option')
     option = '';
    end    
% checks 
    if ~ok_ffa_dataset(dset)
     ierr=1; fprintf('### ffa_list: bad dataset\n');
     return 
    end
    if (~strcmp(option,'mute'))&(~strcmp(option,''))
     ierr=5; fprintf('### ffa_list: unrecognised option "%s"\n',option); 
     return
    end  

    ruler = '=====================================\n';
    if ~strcmp(option,'mute'); fprintf(ruler); end
   [ierr,pds,ilevel,plist] = ffrecurlist(dset,option);
    if ~strcmp(option,'mute'); fprintf(ruler); end
    clear pds
    L=size(plist);
    L=L(1);
    for n=1:L
     pds{n,1}   = plist{n,1};
     errt(n,:) = plist{n,2};
    end
    ierr = max(max(errt))~=0;
    
    return
    
% ----------------------------------------------------------------------------
   function [ierr,pds,ilevel,plist] = ffrecurlist(dset,opt,ierr,pds,ilevel,plist)
% recursive procedure to scan through cell-tree
  
    if ~exist('ierr')          % initialise 
     ierr=0;  pds=[]; ilevel=0; ilocal=0; ierrloc=ierr; plist=[]; elist=[]; errt=[];
    else                                  %20020516SHJ% 
     if ierr==0
      ilocal=ilevel;  ierrloc=ierr; 
     else                     
      return
     end
    end
    
    if length(pds)<ilevel
     ierr=1; fprintf('### ffa_list: length(pds)<ilevel \n'); return
    end
   

% assemble reference strings from pointer "pds"
   [str,sa,as] = ffa_refstring(pds,ilevel); 
    if isempty(str)
     ierr=4; return
    end
   
% retrieve descriptors and data at current position in tree   
    try
     eval(sprintf(' cname = %s%d%s; ',sa,1,as)); 
     eval(sprintf(' ctype = %s%d%s; ',sa,2,as)); 
     eval(sprintf(' nnn   = %s%d%s; ',sa,3,as)); 
     eval(sprintf(' data  = %s%d%s; ',sa,4,as));  
     eval(sprintf(' subz  = %s%d%s; ',sa,5,as));
    catch
     ierr=2; fprintf('### ffa_list: error at level %d using "%s"\n',ilevel,str); return
    end
    ndim=nnn(1); nsiz=nnn(2); nsub=nnn(3); 

% check data against descriptor 
    ok_d = ffa_check_data(ctype,nsiz,ndim,data); % check size and type
    ok_s = check_subs(nsub,subz);                % check no. sub-datasets


% print results & update "plist"
%    try  
     cflag= '---';           % error indicators:
     if     ~ok_d(1)         %    
      cflag(1)='?';          %   ?--  size
     elseif ~ok_d(2)         %          
      cflag(2)='?';          %   -?-  type
     elseif ~ok_s            %         
      cflag(3)='?';          %   --?  subset count 
     end                     %
     
     s = size(plist);        % "plist" update
     N = s(1)+1;             %
     if isempty(pds)|ilevel==0
      ptr=[];                %
     else                    %
      ptr=pds(1:ilevel);     %
     end                     %
     plist{N,1} = ptr;       % 
     plist{N,2} = [~ok_d(1) ~ok_d(2) ~ok_s];
     
     if ~strcmp(opt,'mute')  % print (begin) one row of the list 
      cindent='';            % - set indent for "name"              
      for n=1:ilevel
       cindent=sprintf('%s  ',cindent);
      end
      cdim = sprintf('x %d    ',ndim); cdim = cdim(1:6);
      % print descriptors
      if nsub==0
       format='%6.0f %s %s %9.0f %s        %s%s ';
       fprintf(format,N,ctype,cflag,nsiz,cdim,     cindent,cname)
      else
       format='%6.0f %s %s %9.0f %s %4.0f/  %s%s ';
       fprintf(format,N,ctype,cflag,nsiz,cdim,nsub,cindent,cname)
      end
      % print data sample                      (040327shj)
      nelems = nsiz*ndim;  
      if nelems==0
        data = [];
        datastring = '';
      else
        if iscell(data) 
          if ischar(data{1})
            datastring = data{1}; 
          else
            datastring = '';
          end
        else 
          if nelems>1
            data = reshape(data,1,nelems);
          end
          if ischar(data)
            datastring = sprintf('"%s"',trim(data));
          else
            datastring = '';
            for nda=1:min(3,nelems)
              datastring = sprintf('%s  %d',datastring,data(nda));
            end
          end
        end
      end
      if ~isempty(datastring)      
        fprintf(' = %s',datastring)
      end
      
      if 0
      
      if (nsiz==1|ndim==1)&(nsiz<7&ndim<7) 
       if      ischar(data)
        fprintf('"%s"',trim(data))
       elseif ~iscell(data) 
        for k=1:length(data) 
        fprintf('%d ',data(k))
	end
       end
      end
      
      end % 0
      
      fprintf('\n')
     end                    % print (-end-)
%    catch
%     ierr=3; fprintf('\n### ffa_list: error printing list \n'); return
%    end
      
% loop over sub-datasets
    ilocal=ilocal+1;
    for n=1:nsub
     if isempty(pds)
      pds=n;
     else
      pds(ilocal)=n;
     end
% recursive call for sub-dataset
     [ierrloc,pds,ilevel,plist] = ffrecurlist(dset,opt,ierr,pds,ilocal,plist); 
     if ierrloc>0
      ierr=ierrloc; return
     end
    end   
    ilocal=ilocal-1;   
   
    if exist('ierr')
     ierr = ierrloc;   % 20020516SHJ "lerr" changed to ierr
    end
    if exist('ilevel')
     ilevel = ilocal;
    end
  
%-------------------------------------------------------------------------------  

   function ok = check_subs(nsub,subz);   
% check no. sub-datasets  
    if isempty(subz)                                     
     ok = nsub==0;
    else
     s = size(subz);
     ok = s(1)==nsub & s(2)==5;
    end
   
%------------------------------------------------------------------------------
   function str = trim(str)
% TRIM   strip leading and trailing blanks from a string
%
% Usage:    str = trim(str)

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
   

%-------------------------------------------------------------------------------
% Further developments:
%
% "options" to extended so that values can be entered as a string 
% (use "findstr" instead of "strcmp") 
%
% some additional options:  nocheck  -suspend checks just get pointerrt-
%                                     (if not "mute" then don't print checks)
%                           ?control over list output format
%
%
%
%
%
   
