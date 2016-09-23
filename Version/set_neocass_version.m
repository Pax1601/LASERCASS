function set_neocass_version(varargin)
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER           DESCRIPTION
%     111013       1.0    L. Riccobene         Creation
%
%**************************************************************************
%
% function        set_neocass_version('Package_name', 'Package_version')
%
%   DESCRIPTION: Set NeoCASS packages version, input should be given in
%                pairs. Invoking the function with no input, print to video
%                current packages' versions (if a version file exists).
%
%         INPUT: NAME            TYPE       DESCRIPTION
%
%                Package_name    string     one of the available NeoCASS package
%
%                Package_version string     package version, strictly in
%                                           the format: 'x.x.xxx', where
%                                           the first two digits identify
%                                           the version while the latest
%                                           the revision.
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
% !Remark: a file called 'NeoCASS_version.mat' will be generated - if the
% function is launched for the first time - or read and updated.
%**************************************************************************

% Version string is composed by 3 numbers separated by dots
vernum = 2;
versep = '.';

% Set default file name
default_filename = 'NeoCASS_version.mat';

% Available packages
packagesC = {'NeoCASS', 'NeoResp'};

% Version path
versiondir = neoversion_path;

nin = nargin;

if nin

    % Recover date
    date = datestr(now);
       
    packages = lower(packagesC);
    
    % If a version file exists load struct, either way initialize data struct
    if exist(default_filename, 'file') && exist(versiondir, 'dir')
        
        % Load version data
        ld   = load(fullfile(versiondir, default_filename), 'data');
        data = ld.data;
        
    else
        
        % Initialize data
        for k = 1:length(packages),
            data.(packages{k}).package_name   = packagesC{k};
            data.(packages{k}).last_version   = [];
            data.(packages{k}).actual_version = [];
            data.(packages{k}).version_date   = [];
        end
        
    end
    
    % Input should be given in pairs label/value
    if ~rem(nin, 2)
        
        for k = 1:2:nin,
            
            % Match package name with available packages (capital letters
            % don't affect comparison)
            findname = ismember(packages, lower(varargin{k}));
            
            if any(findname) && nnz(findname)==1
                % Build version name: developer inserts a string which will
                % be split and padded with date
                if ~ischar(varargin{k+1})
                    error('set_neocass_version:WrongInputType',...
                        'Version number should be a string');
                else
                    
                    % Check dots number (should be equal to prescribed
                    % vernum)
                    if nnz(varargin{k+1} == versep) == vernum
                        % Version and revision
                        version_str = varargin{k+1};
                    else
                        error('set_neocass_version:WrongInputElemNumber',...
                            'Version number is composed by three elements separated by dots');
                    end
                    
                end
                
                % Make a copy of the last modified version and update
                % actual field
                data.(packages{findname}).last_version   = data.(packages{findname}).actual_version;
                data.(packages{findname}).actual_version = version_str;
                
                % Save date
                data.(packages{findname}).version_date   = date;
                
            else
                
                fprintf(' - Package %s not available, skipping...\n', varargin{k});
                
            end
            
        end
        
        % Save on disk version data
        save(fullfile(versiondir, default_filename), 'data');
        
    else
        
        error('set_neocass_version:WrongInputPair',...
            'Input should be given in pairs label/value');
        
    end
    
else
    
    % With no input show current version(s) stored in the version file if it exists
    if exist(default_filename, 'file') && exist(versiondir, 'dir')

        for k = 1:length(packagesC),
           disp(get_neocass_version(packagesC{k})); 
        end
        
    else
       fprintf(' - Couldn''t find any version file in the path.\n'); 
    end
    
end

