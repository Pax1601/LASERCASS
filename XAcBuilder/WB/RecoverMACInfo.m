function [MAC, YMAC, XLEMAC, XACMAC] = RecoverMACInfo(aircraft)
%**************************************************************************
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
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS      PROGRAMMER       DESCRIPTION
%     120919      2.1.416   L.Riccobene      Creation
%
%**************************************************************************
%
% function     [MAC, YMAC, XLEMAC, XACMAC] = RecoverMACInfo(aircraft)
%
%
%   DESCRIPTION: Given the struct or the XML file storing all aircraft
%                infos (from AcBuilder), returns mean aerodynamic chord,
%                its spanwise position with respect symmetry plane x-z, its
%                leading edge x-coordinate with respect aircraft nose and
%                subsonic aerodynamic centre x-coordinate w.r.t. aircraft nose. 

%         INPUT: NAME           TYPE       DESCRIPTION
%
%                aircraft    struct/char   from AcBuilder or XML file name
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                MAC            double     mean aerodynamic chord [m]
%                YMAC           double     MAC spanwise position w.r.t.
%                                          aircraft centerline [m] 
%                XLEMAC         double     MAC leading edge x-coordinate
%                                          w.r.t. aircraft nose [m] 
%                XACMAC         double     MAC aerodynamic centre
%                                          x-coordinate w.r.t aircraft nose [m]
%
%    REFERENCES:
%
%**************************************************************************

% Check input
which_input = class(aircraft);

% Recover sub-struct related to Wing1
switch which_input
    case 'char'
        try
            % Load xml file
            aircraft = neocass_xmlwrapper(aircraft);
        catch excp
            fprintf('## Error (%s): input should be either an XML file or a struct ##\n', excp.message);
        end
    case 'struct'
        % Do nothing
    otherwise
        fprintf('Error: class %s not allowed as input ##\n', which_input);
        MAC    = 0;
        YMAC   = 0;
        XLEMAC = 0;
        XACMAC = 0;
        return
end
% call for wing MAC calculation
[MAC, YMAC, XLEMAC, XACMAC] = MAC_wing(aircraft.Wing1, aircraft.Fuselage.Total_fuselage_length);
%