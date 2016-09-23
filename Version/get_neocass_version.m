function version = get_neocass_version(package_name)
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
% function      version = get_neocass_version(package_name)
%
%   DESCRIPTION: Set NeoCASS packages version
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                package_name   string     package name whose version is
%                                          required
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                version        string     version number with appended the
%                                          modification date
%**************************************************************************

nin = nargin;

if nin && ischar(package_name)
    
    % Set default file name
    default_filename = 'NeoCASS_version.mat';
    
    % Version path
    versiondir = neoversion_path;

    % Available packages
    packages = lower({'NeoCASS', 'NeoResp'});
    
    % Preliminary check on input
    chk_name = ismember(packages, lower(package_name));
    if ~any(chk_name)
        error('get_neocass_version:VersionNotFound', 'Package (%s) not available', package_name);
    else
        
        % If a version file exists load struct
        if exist(default_filename, 'file') && exist(versiondir, 'dir')
            
            % Load version data
            ld   = load(fullfile(versiondir, default_filename), 'data');
            data = ld.data;
            
            % Form version string
            package_ver = data.(packages{chk_name});
            
            version = [package_ver.package_name, ' version ', package_ver.actual_version, '.',...
                blanks(1), 'Release date:', blanks(1), package_ver.version_date];
            
        else
            % Error: missing version file
            error('get_neocass_version:FileNotFound', 'Version file (%s) not found!', default_filename);
        end
    end
    
else
    error('get_neocass_version:IncorrectInput', 'Only one character input is accepted');
end