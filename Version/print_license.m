%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER           DESCRIPTION
%     081103      1.1     J.Opperlstrupp       Creation
%                         jespero@nada.kth.se
%*******************************************************************************
%
% function set_neocass_path
%
%   DESCRIPTION: Look for .m and .DAT files and include directories if any
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************

function print_license(license)
ss      = pwd;
dirlist = {};% dirlist{1} = ss;
dirlist = list_subdir_m(ss,dirlist);
[m,n]   = size(dirlist);
for k = 1:n
    if isempty(strfind(dirlist{k},'xml_toolbox')) 
        cd(dirlist{k})
        funMat = dir('*.m');
        NumFun = length(funMat);
        for jj = 1 :NumFun
            if isempty(strfind(funMat(jj).name,'print_license'))
                printONtop(license,funMat(jj).name);
            end
        end
    end
%     path(path,dirlist{k});
end;
% rehash; 

  function dirlist = list_subdir_m(pathn,dirlist)
  %
  subdirs = struct2cell(dir(pathn));
  [m,n]   = size(subdirs);
  include_this = 0;
  for k = 1:n
    fn  = deblank(subdirs{1,k});                         
    if ~isequal(fn(end),'.')                             
      if subdirs{4,k}==1                                
        pathn2 = fullfile(pathn,fn);
        dirlist = list_subdir_m(pathn2,dirlist);      
      end
      index = find(lower(fn) == '.');
      if ~isempty(index)
        if  (isequal(lower(fn(index(end):end)),'.m')) %|| (isequal(fn(index(end):end),'.DAT')) )
          include_this = 1;
        end
      end
    end
  end

  if include_this                                          
    dirlist{end+1} = pathn;                               
  end 
  end

end