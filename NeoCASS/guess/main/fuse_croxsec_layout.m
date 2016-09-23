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

function [cross_section] = fuse_croxsec_layout(concept, tbar_S, d, Nxc, E)

% Initialize the output structure
cross_section = [];

switch concept
       
%----------------------------------------------------------------------
% Simply stiffened shell

    case 1
        
        [ts, tw, bs, bw] = kcon2(tbar_S, d, Nxc, E);
        % PBARSM3 card input
        cross_section = [ts, tw, bs, bw];
        
%----------------------------------------------------------------------
% Z-stiffened shell

    case 2
        
        [ts, tf, tw, bs, bf, bw] = kcon3(tbar_S, d, Nxc, E);
        % PBARSM4 card input
        cross_section = [ts, tw, tf, bs, bw, bf];
        
    case 3
        
        [ts, tf, tw, bs, bf, bw] = kcon4(tbar_S, d, Nxc, E);
        % PBARSM4 card input
        cross_section = [ts, tw, tf, bs, bw, bf];
        
    case 4
        
        [ts, tf, tw, bs, bf, bw] = kcon5(tbar_S, d, Nxc, E);
        % PBARSM4 card input
        cross_section = [ts, tw, tf, bs, bw, bf];
        
%----------------------------------------------------------------------
% Truss-core sandwich

    case 5  
        
        [tf, tc, bf, bc, h, theta] = kcon6(tbar_S, Nxc, E);
        % PBARSM5 card input
        THETA = theta *180/pi *ones(length(tf));
        cross_section = [tf, tc, bf, h, THETA];
        
    case 6
        
        [tf, tc, bf, bc, h, theta] = kcon8(tbar_S, Nxc, E);
        % PBARSM5 card input
        THETA = theta *180/pi *ones(length(tf));
        cross_section = [tf, tc, bf, h, THETA];
        
    case 7
        
        [tf, tc, bf, bc, h, theta] = kcon9(tbar_S, Nxc, E);
        % PBARSM5 card input
        THETA = theta *180/pi *ones(length(tf));
        cross_section = [tf, tc, bf, h, THETA];
        
    otherwise
        
        error('Conceptual cross-sectional fuselage layout not available');
        
end

end
%------------------------------------------------------------------------------------------------------------


%------------------------------------------------------------------------------------------------------------
function [ts, tw, bs, bw] = kcon2(tbar_S, d, Nxc, E)
%--------------------------------------------------------------------------
% Applicable to fuselage structural concept: 1
%   Simply stiffened shell, frames, sized for minimum weight in buckling
% 
% Output: 4 parameters
% 
% Source: ASD TR 61-692, pag. 4-6
%--------------------------------------------------------------------------

% optimum ratios: tw/ts = 2.25
opt1 = 2.25;
% optimum ratios: bw/bs = 0.65
opt2 = 0.65;

% skin thickness
ts = tbar_S ./ (1 + opt2 *opt1);
% web thickness of Zee
tw = opt1 *ts;
% distance between Zees
bs = 1.1 .*(1 + opt1 *opt2) .*d .*sqrt(Nxc ./(E.*tbar_S .*(opt1 *opt2^3).*(4 + opt1 *opt2)));
% web higth of Zee
bw = opt2 .*bs;

end
%------------------------------------------------------------------------------------------------------------
function [ts, tf, tw, bs, bf, bw] = kcon3(tbar_S, d, Nxc, E)
%--------------------------------------------------------------------------
% Applicable to fuselage structural concept: 2
%   Z-stiffened shell, frames, best buckling
% 
% Output: 6 parameters
% 
% Source: ASD TR 61-692, pag. 4-9
%--------------------------------------------------------------------------

% optimum ratios: tw/ts = 1.06
opt1 = 1.06;
% optimum ratios: bw/bs = 0.87
opt2 = 0.87;

% skin thickness
ts = tbar_S ./ (1 + 1.6 *opt2 *opt1);
% web thickness of Zee
tw = opt1 *ts;
% flange thickness of Zee
tf = tw;
% web higth of Zee
bw = ((0.4 *(1 + 1.6 *opt2 *opt1)) ./sqrt(opt2 *opt1 *(1 + 0.59 *opt2 *opt1))) .*d .*sqrt(Nxc ./(E.*tbar_S));
% flange length of Zee
bf = 0.3 *bw;
% distance between Zees
bs = bw ./opt2;

end
%------------------------------------------------------------------------------------------------------------
function [ts, tf, tw, bs, bf, bw] = kcon4(tbar_S, d, Nxc, E)
%--------------------------------------------------------------------------
% Applicable to fuselage structural concept: 3
%   Z-stiffened shell, frames, buckling-minimum gage compromise
% 
% Output: 6 parameters
% 
% Source: ASD TR 61-692, pag. 4-9
%--------------------------------------------------------------------------

% optimum ratios: tw/ts = 0.90
opt1 = 0.90;
% optimum ratios: bw/bs = 0.58
opt2 = 0.58;

% skin thickness
ts = tbar_S ./ (1 + 1.6 *opt2 *opt1);
% web thickness of Zee
tw = opt1 *ts;
% flange thickness of Zee
tf = tw;
% web higth of Zee
bw = ((0.4 *(1 + 1.6 *opt2 *opt1)) ./sqrt(opt2 *opt1 *(1 + 0.59 *opt2 *opt1))) .*d .*sqrt(Nxc ./(E.*tbar_S));
% flange length of Zee
bf = 0.3 *bw;
% distance between Zees
bs = bw ./opt2;

end
%------------------------------------------------------------------------------------------------------------
function [ts, tf, tw, bs, bf, bw] = kcon5(tbar_S, d, Nxc, E)
%--------------------------------------------------------------------------
% Applicable to fuselage structural concept: 4
%   Z-stiffened shell, frames, buckling-pressure compromise
% 
% Output: 6 parameters
% 
% Source: ASD TR 61-692, pag. 4-9
%--------------------------------------------------------------------------

% optimum ratios: tw/ts = 0.60
opt1 = 0.60;
% optimum ratios: bw/bs = 0.60
opt2 = 0.60;

% skin thickness
ts = tbar_S ./ (1 + 1.6 *opt2 *opt1);
% web thickness of Zee
tw = opt1 *ts;
% flange thickness of Zee
tf = tw;
% web higth of Zee
bw = ((0.4 *(1 + 1.6 *opt2 *opt1)) ./sqrt(opt2 *opt1 *(1 + 0.59 *opt2 *opt1))) .*d .*sqrt(Nxc ./(E.*tbar_S));
% flange length of Zee
bf = 0.3 *bw;
% distance between Zees
bs = bw ./opt2;

end
%------------------------------------------------------------------------------------------------------------
function [tf, tc, bf, bc, h, theta] = kcon6(tbar_S, Nxc, E)
%--------------------------------------------------------------------------
% Applicable to fuselage structural concept: 5
%   Truss-core sandwich, best buckling
% 
% Output: 6 parameters
% 
% Source: ASD TR 61-692, pag. 4-12
%--------------------------------------------------------------------------

% optimum angle: theta = 62°
theta = 62 *pi/180;
% optimum ratio: tc/tf = 0.92
opt = 0.92;

% face sheet thickness
tf = tbar_S ./ (2 + opt /cos(theta));
% core thichkness
tc = opt .*tf;
% interpolated from Fig.4-6, pag. 4-14, given theta = 62° and tc/tf = 0.92
Kx = 4.1;
%
bf = 0.95 *tf .*sqrt(Kx .*tbar_S .*E ./Nxc);
% 
bc = bf ./(2*cos(theta));
% sandwich width
h = tan(theta) .*bf/2;

end
%------------------------------------------------------------------------------------------------------------
function [tf, tc, bf, bc, h, theta] = kcon8(tbar_S, Nxc, E)
%--------------------------------------------------------------------------
% Applicable to fuselage structural concept: 6
%   ...
% 
% Output: 6 parameters
% 
% Source: ASD TR 61-692, pag. 4-12
%--------------------------------------------------------------------------

% optimum angle: theta = 55°
theta = 55 *pi/180;
% optimum ratio: tc/tf = 0.65
opt = 0.65;

% face sheet thickness
tf = tbar_S ./ (2 + opt /cos(theta));
% core thichkness
tc = opt .*tf;
% interpolated from Fig.4-6, pag. 4-14, given theta = 55° and tc/tf = 0.65
Kx = 3.5;
%
bf = 0.95 *tf .*sqrt(Kx .*tbar_S .*E ./Nxc);
% 
bc = bf ./(2*cos(theta));
% sandwich width
h = tan(theta) .*bf/2;

end
%------------------------------------------------------------------------------------------------------------
function [tf, tc, bf, bc, h, theta] = kcon9(tbar_S, Nxc, E)
%--------------------------------------------------------------------------
% Applicable to fuselage structural concept: 7
%   Truss-core sandwich, no frames, buckling minimum gage-pressure
%   compromise
% 
% Output: 6 parameters
% 
% Source: ASD TR 61-692, pag. 4-12
%--------------------------------------------------------------------------

% optimum angle: theta = 45°
theta = 45 *pi/180;
% optimum ratio: tc/tf = 1.0
opt = 1.0;

% face sheet thickness
tf = tbar_S ./ (2 + opt /cos(theta));
% core thichkness
tc = opt .*tf;
% interpolated from Fig.4-6, pag. 4-14, given theta = 55° and tc/tf = 0.65
Kx = 5.3;
%
bf = 0.95 *tf .*sqrt(Kx .*tbar_S .*E ./Nxc);
% 
bc = bf ./(2*cos(theta));
% sandwich width
h = tan(theta) .*bf/2;

end
%------------------------------------------------------------------------------------------------------------
