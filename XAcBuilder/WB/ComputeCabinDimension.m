function [cabnwid, cabnhei, cabnfwd] = ComputeCabinDimension(aircraft)
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
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     120508      2.237   L.Riccobene      Creation
%
%**************************************************************************
%
% function       Script called by rcogs
%
%
%   DESCRIPTION:   Compute cabin dimensions cabin width (max internal
%                  width), cabin height (accounting for high/low wing
%                  passing through the fuselage) and cabin floor width.
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%
%
%    REFERENCES:
%
%**************************************************************************
%

% Fuselage Section characteristics
d_v_a   = aircraft.Fuselage.Aftfuse_X_sect_vertical_diameter;
d_v_f   = aircraft.Fuselage.Forefuse_X_sect_vertical_diameter;
xi_x_a  = aircraft.Fuselage.Forefuse_Xs_distortion_coefficient;
xi_x_f  = aircraft.Fuselage.Aftfuse_Xs_distortion_coefficient;

% Wing placement and dimensions at root
wingplc = aircraft.Wing1.placement;
wingthk = aircraft.Wing1.thickness_root;
WCHDROT = aircraft.Reference_wing.Orig_root_chrd_at_ac_CL;

% Aft fuselage section Fourier coefficients
a0a     = aircraft.Fuselage.a0_aft;
a1a     = aircraft.Fuselage.a1_aft;
b1a     = aircraft.Fuselage.b1_aft;

% Fore fuselage section Fourier coefficients
a0f     = aircraft.Fuselage.a0_fore;
a1f     = aircraft.Fuselage.a1_fore;
b1f     = aircraft.Fuselage.b1_fore;

% Angle resolution 1 degree
N = 36;

% Fuselage fore section radius
[~, ~, Ref] = ComputeFuselageXSecRadius(a0f, a1f, b1f, N, xi_x_f, d_v_f);

% Fuselage aft section radius
[~, ~, Rea] = ComputeFuselageXSecRadius(a0a, a1a, b1a, N, xi_x_a, d_v_a);

% Radius and coordinates of average section
Rm  = mean([Ref; Rea]);
phi = 0:2*pi/N:2*pi;
xm  = Rm.*cos(phi);
ym  = Rm.*sin(phi);
dvm = mean([d_v_f, d_v_a]);

% Wall thickness for average section
wt  = .93; % percentage
xim = xm.*wt;
yim = ym.*wt;

if aircraft.cabin.Cabin_max_internal_height <= 0
    
    % Max vertical diameter (max cabin height)
    dvi_max = max(yim) - min(yim);
    
    % Accounting for wing passing through fuselage - either low or high - and
    % wall thickness
    if wingplc <= 0. || wingplc >= 1.
        
        % Wing-box does not go through x-section
        cabnhei = dvi_max;
        
    else
        
        % If wingplc = 0.5 it means that wing leading edge is on fuselage center,
        % i.e. y_LE = 0, if 1. > wingplc > 0.5 high wing while 0. < wingplc < 0.5 low wing
        if wingplc > .5
            cabnhei = dvi_max - dvm*(wingplc-.5) - wingthk*WCHDROT/2;
        else
            cabnhei = dvi_max - dvm*wingplc - wingthk*WCHDROT/2;
        end
        
    end
    
else
    % User defined
    cabnhei = aircraft.cabin.Cabin_max_internal_height;
end

if aircraft.cabin.Cabin_max_internal_width <= 0.
    
    % Max horizontal diameter (max cabin width)
    [~, Ih] = max(xim);
    
    % Compute cabin width
    cabnwid = 2*xim(Ih);
    
else
    % User defined
    cabnwid = aircraft.cabin.Cabin_max_internal_width;
end

if aircraft.cabin.Cabin_floor_width <= 0.
    
    if wingplc <= 0. || wingplc >= 1.
        
        % Seat width (percentage of maximum cabin width)
        if aircraft.cabin.Seats_abreast_in_fuselage > 0.
            seat_width_p = 0.015;
            % Account for aisle width too
            seat_width   = seat_width_p*(aircraft.cabin.Seats_abreast_in_fuselage+1)*cabnwid;
        else
            % Suppose at least four seats abreast (plus aisle width)
            seat_width   = 0.075*cabnwid;
        end
        
        % Find maximum horizontal diameter to accommodate seats
        [~, Is_max] = max(abs(xim - seat_width));
        
        % Compute cabin floor width
        cabnfwd = 2*abs(xim(Is_max));
        
    else
        
        % Find cabin height upper apex
        [~, Iv_max] = max(yim);
        
        % Find cabin floor y-coordinate with respect maximum internal diameter
        % upper apex
        if wingplc > 0.5
            [~, Ifl] = min( abs(yim - (yim(Iv_max)-dvm*wingplc)) );
        else
            [~, Ifl] = min( abs(yim - (yim(Iv_max)-cabnhei)) );
        end
        
        % Compute cabin floor width
        cabnfwd = 2*abs(xim(Ifl));
        
    end
    
else
    % User defined
    cabnfwd = aircraft.cabin.Cabin_floor_width;
end
