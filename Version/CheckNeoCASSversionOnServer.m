function status = CheckNeoCASSversionOnServer()
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER           DESCRIPTION
%     121004      2.1.450  L. Riccobene         Creation
%
%**************************************************************************
%
% function        status = CheckNeoCASSversionOnServer()
%
%   DESCRIPTION:  Compare version numbers between the current NeoCASS
%                 installation and the Server package.
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%**************************************************************************

% Check if NeoCASS path exists
status   = 0;
verFname = 'NeoCASS_version.mat';
neo_dir  = which(verFname);

% Check Java
if ~usejava('jvm')
    error('CheckNeoCASSversionOnServer:NoJvm', 'CheckNeoCASSversionOnServer requires Java.');
end

if ~isempty(neo_dir)
    
    % Inquiry server to find latest updloaded version
    ServerURL     = 'https://www.neocass.org/downloads/';
    ServerVerFile = 'Current_version.txt';
    fullURL       = [ServerURL, ServerVerFile];
    
    try
        
        % Use urlread
        ServerVer = urlread(fullURL);
        
        % Generate version number
        verDir   = neoversion_path;
        loadData = load(fullfile(verDir, verFname));
        verData  = loadData.data;
        
        % Compare version numbers
        package_name = lower('NeoCASS');
        actVerN      = verData.(package_name).actual_version;
        
        % Convert char to number
        actVerN_num   = str2double(actVerN(actVerN ~= '.'));
        ServerVer_num = str2double(ServerVer(ServerVer ~= '.'));
        
        % Pop-up message box only if server version is greater than
        % computer version
        if ~strcmp(actVerN, ServerVer) && (ServerVer_num > actVerN_num)
            filename = 'NeoCASS_Latest_Version.rar';
            status   = CheckNeoCASSversionOnServerMsgBox(actVerN, ServerVer,...
                [ServerURL, filename]);
        elseif isnan(ServerVer_num)
            % Something went wrong while reading from the repository (maybe
            % a redirection issue)
            fprintf(' ** Couldn''t recover NeoCASS version from server **\n');
            status = -1;
        else
            % Skip
            fprintf(' - NeoCASS is updated to the latest version (%s)\n', actVerN);
        end
        
    catch excep
        % Show extra info on error
%         fprintf('** Couldn''t establish an internet connection to %s url. **\n\n', ServerURL);
%         fprintf(' %s \n', excep.message);
         fprintf('.\n');
    end
    
else
    fprintf(' - NeoCASS installation not found in Matlab path, can''t check version...\n');
end

