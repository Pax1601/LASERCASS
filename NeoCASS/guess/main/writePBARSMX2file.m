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
%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% PBARSMX2 card generation in ASCII file
%
%   Author: <andreadr@kth.se>
%
% Called by:    setup_PROPERTY_BAR.m
%
% Calls:        BULKdataPBAR.m
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     080806      1.0      A. Da Ronch      Creation
%     091119      1.3.9    Travaglini       Modification
%
% Modified by Travaglini changing wing2 with canard
%*******************************************************************************
function [geo, str, stick] = writePBARSMX2file(outf, fid, pdcylin, geo, loads, str, stick, aircraft)

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Beam properties');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

fprintf(outf, '\n     Exporting beam structural properties...');

%----------------------------------------------------------------------------------------------------------------------
% Fuselage
if isequal(stick.model.fuse, 1)
    
    % Beam element mass
    str.fus.Mstick = interp1(geo.fus.x_nodes_1_2_thick, str.fus.m, geo.fus.x_nodes_1_2);
    
    % Beam element area
    str.fus.Astick = interp1(geo.fus.x_nodes_1_2_thick,...
        str.fus.m./(pdcylin.fus.ds*diff(geo.fus.x_nodes_thick)), geo.fus.x_nodes_1_2, 'spline');
    
    % Total thickness
    str.fus.tbar_Bstick = interp1(geo.fus.x, str.fus.tbar_B, geo.fus.x_nodes_1_2);
    if any(str.fus.tbar_Bstick < 0)
        error('Negative value in interpolation data 1.');
    end
    
    % Shell thickness
    str.fus.tbar_Sstick = interp1(geo.fus.x, str.fus.tbar_S, geo.fus.x_nodes_1_2);
    if any(str.fus.tbar_Sstick < 0)
        error('Negative value in interpolation data 2.');
    end
    
    % Frame thickness
    str.fus.tbar_Fstick = interp1(geo.fus.x, str.fus.tbar_F, geo.fus.x_nodes_1_2);
    if any(str.fus.tbar_Fstick < 0)
        error('Negative value in interpolation data.');
    end
    
    % Frame area
    str.fus.tbar_Afstick = interp1(geo.fus.x, str.fus.tbar_Af, geo.fus.x_nodes_1_2);
    if any(str.fus.tbar_Afstick < 0)
        error('Negative value in interpolation data 3.');
    end
    
    % Frame spacing
%     str.fus.tbar_dstick = interp1(geo.fus.x(2:end-1), str.fus.d(2:end-1), geo.fus.x_nodes_1_2);
    str.fus.tbar_dstick = interp1(geo.fus.x, str.fus.d, geo.fus.x_nodes_1_2);
    if any(str.fus.tbar_dstick < 0)
        error('Negative value in interpolation data 4.');
    end
    
    % Inertia Ixx
    % str.fus.I1stick = interp1(geo.fus.x, str.fus.I1, geo.fus.x_nodes_1_2);
    % Equivalent radius
    geo.fus.rstick = interp1(geo.fus.x, geo.fus.r, geo.fus.x_nodes_1_2);
    if any(geo.fus.rstick < 0)
        error('Negative value in interpolation data 5.');
    end
    
    % A: ellipse semiaxis along z
    geo.fus.thick_horstick = interp1(geo.fus.thick_dom, geo.fus.thick_hor, geo.fus.x_nodes_1_2);
    
    % B: ellipse semiaxis along y
    geo.fus.thick_verstick = interp1(geo.fus.thick_dom, geo.fus.thick_ver, geo.fus.x_nodes_1_2);
    
    %     % initialize
    %     C1 = zeros(length(stick.PID.fuse), 1);
    %     C2 = C1;
    %     D1 = C1;
    %     D2 = C1;
    %     E1 = C1;
    %     E2 = C1;
    %     F1 = C1;
    %     F2 = C1;
    
    %     for i = 1:length(stick.PID.fuse)
    %
    %         % give the average since constant properties are assumed over the
    %         % beam length; construct the cross changing sign
    %         stick.PBAR.fuse.str_rec_coef.C1(i) = (stick.PBAR.fuse.str_rec_coef.C(1,i*2-1)+stick.PBAR.fuse.str_rec_coef.C(1,i*2))/2;
    %         stick.PBAR.fuse.str_rec_coef.C2(i) = (stick.PBAR.fuse.str_rec_coef.C(2,i*2-1)+stick.PBAR.fuse.str_rec_coef.C(2,i*2))/2;
    %         stick.PBAR.fuse.str_rec_coef.D1(i) = (stick.PBAR.fuse.str_rec_coef.D(1,i*2-1)+stick.PBAR.fuse.str_rec_coef.D(1,i*2))/2;
    %         stick.PBAR.fuse.str_rec_coef.D2(i) = (stick.PBAR.fuse.str_rec_coef.D(2,i*2-1)+stick.PBAR.fuse.str_rec_coef.D(2,i*2))/2;
    %         stick.PBAR.fuse.str_rec_coef.E1(i) = -C1;
    %         stick.PBAR.fuse.str_rec_coef.E2(i) = -C2;
    %         stick.PBAR.fuse.str_rec_coef.F1(i) = -D1;
    %         stick.PBAR.fuse.str_rec_coef.F2(i) = -D2;
    %
    %     end
    
    % Select PBARSM2 or PBARSMX (X through 3 to 5)
    stype = pdcylin.optimization_smonoq;
    [geo, str, stick] = writePBARSMX2file_layout(outf, fid, pdcylin, geo, loads, str, stick, stype);
    
end
%--------------------------------------------------------------------------------------------------------------------------

%----------------------------------------------------------------------------------------------------------------------
% Tail booms
if isequal(stick.model.tboomsr, 1)
    
    % Beam element mass
    str.tbooms.Mstick = interp1(geo.tbooms.x_nodes_1_2_thick, str.tbooms.m, geo.tbooms.x_nodes_1_2);
    
    % Beam element area
    str.tbooms.Astick = interp1(geo.tbooms.x_nodes_1_2_thick,...
        str.tbooms.m./(pdcylin.tbooms.ds*diff(geo.tbooms.x_nodes_thick)), geo.tbooms.x_nodes_1_2, 'spline');
    
    % Total thickness
    str.tbooms.tbar_Bstick = interp1(geo.tbooms.x, str.tbooms.tbar_B, geo.tbooms.x_nodes_1_2);
    if any(str.tbooms.tbar_Bstick < 0)
        error('Negative value in interpolation data 1.');
    end
    
    % Shell thickness
    str.tbooms.tbar_Sstick = interp1(geo.tbooms.x, str.tbooms.tbar_S, geo.tbooms.x_nodes_1_2);
    if any(str.tbooms.tbar_Sstick < 0)
        error('Negative value in interpolation data 2.');
    end
    
    % Frame thickness
    str.tbooms.tbar_Fstick = interp1(geo.tbooms.x, str.tbooms.tbar_F, geo.tbooms.x_nodes_1_2);
    if any(str.tbooms.tbar_Fstick < 0)
        error('Negative value in interpolation data.');
    end
    
    % Frame area
    str.tbooms.tbar_Afstick = interp1(geo.tbooms.x, str.tbooms.tbar_Af, geo.tbooms.x_nodes_1_2);
    if any(str.tbooms.tbar_Afstick < 0)
        error('Negative value in interpolation data 3.');
    end
    
    % Frame spacing
%     str.fus.tbar_dstick = interp1(geo.fus.x(2:end-1), str.fus.d(2:end-1), geo.fus.x_nodes_1_2);
    str.tbooms.tbar_dstick = interp1(geo.tbooms.x, str.tbooms.d, geo.tbooms.x_nodes_1_2);    
    if any(str.tbooms.tbar_dstick < 0)
        error('Negative value in interpolation data 4.');
    end
    
    % Inertia Ixx
    % str.fus.I1stick = interp1(geo.fus.x, str.fus.I1, geo.fus.x_nodes_1_2);
    % Equivalent radius
    geo.tbooms.rstick = geo.tbooms.R*ones(length(geo.tbooms.x_nodes_1_2),1);
    if any(geo.tbooms.rstick < 0)
        error('Negative value in interpolation data 5.');
    end
        
    % Select PBARSM2 or PBARSMX (X through 3 to 5)
    stype = pdcylin.optimization_smonoq;
    [geo, str, stick] = writePBARSMX2file_tb_layout(outf, fid, pdcylin, geo, loads, str, stick, stype);
    
end
%--------------------------------------------------------------------------------------------------------------------------

%----------------------------------------------------------------------------------------------------------------------
% Wing
if isequal(stick.model.winr, 1)
    
    % Interpolate thickness of cover, chord and height of wing box
    geo.wing.tbsstick = interp1(geo.wing.y_nodes_thick, geo.wing.tbs, geo.wing.y_nodes_1_2);
    geo.wing.Zsstick  = interp1(geo.wing.y_nodes_thick, geo.wing.Zs,  geo.wing.y_nodes_1_2);
    str.wing.tCstick  = interp1(geo.wing.y_nodes_thick, str.wing.tC , geo.wing.y_nodes_1_2);
    str.wing.tWstick  = interp1(geo.wing.y_nodes_thick, str.wing.tW , geo.wing.y_nodes_1_2);
    str.wing.dWstick  = interp1(geo.wing.y_nodes_thick, str.wing.dW , geo.wing.y_nodes_1_2);
    
    % Get column vector
    geo.wing.tbsstick = go2col(geo.wing.tbsstick);
    geo.wing.Zsstick  = go2col(geo.wing.Zsstick);
    str.wing.tCstick  = go2col(str.wing.tCstick);
    str.wing.tWstick  = go2col(str.wing.tWstick);
    str.wing.dWstick  = go2col(str.wing.dWstick);

    % Beam element mass
    str.wing.Mstick = interp1(geo.wing.y_nodes_1_2_thick, str.wing.m, geo.wing.y_nodes_1_2);

    % Beam element area
    % Separate carry-through and cantilever part
    nrcth   = pdcylin.stick.nwing_carryth;
    dy_carr = norm(stick.ptos.winrC2(:, 2)-stick.ptos.winrC2(:, 1))/nrcth;    
    str.wing.Astick              = zeros(numel(geo.wing.y_nodes_1_2), 1);
    str.wing.Astick(1:nrcth)     = str.wing.m(1:nrcth)./(pdcylin.wing.dsw*dy_carr);
    str.wing.Astick(nrcth+1:end) = interp1(geo.wing.y_nodes_1_2_thick(nrcth+1:end),...
        str.wing.mbox./(pdcylin.wing.dsw*diff(geo.wing.y_nodes_thick(nrcth+1:end))), geo.wing.y_nodes_1_2(nrcth+1:end), 'linear');
    
    % Determine lumped area to set in 4 corners for simplest wing box type (PBARSM1)
    % str.wing.Abstick = (str.wing.Astick -2.*(geo.wing.tbsstick+geo.wing.Zsstick).*str.wing.tCstick) / 4;
    A4 = 2*geo.wing.tbsstick.*str.wing.tWstick + geo.wing.Zsstick.*str.wing.tCstick;
    str.wing.Abstick = (str.wing.Astick - A4)/4;
    
    if any(str.wing.Abstick < 0)
        error('Negative area for PBARSMX2 card');
    end
    
    for i = 1:length(stick.PID.wing)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.wing.str_rec_coef.C(1,i);
        C2 = stick.PBAR.wing.str_rec_coef.C(2,i);
        D1 = stick.PBAR.wing.str_rec_coef.D(1,i);
        D2 = stick.PBAR.wing.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        % PBARSM1 card
        %        BULKdataPBARSM1(fid, stick.PID.wing(i), stick.MAT1.wing, str.wing.Abstick(i),...
        %                                     str.wing.tCstick(i), geo.wing.Zsstick(i),...
        %                                     geo.wing.tbsstick(i), str.wing.NSM.dstr(i),...
        %                                     C1, C2, D1, D2, E1, E2, F1, F2);
        
        %        BULKdataPBARGMW(fid, stick.PID.wing(i), stick.MAT1.wing, str.wing.tWstick(i)*geo.wing.Kgw,...
        %                                     str.wing.tCstick(i)*geo.wing.Kgc, str.wing.dWstick(i), ...
        %                                     geo.wing.Zsstick(i), geo.wing.tbsstick(i), ...
        %                                     geo.wing.Kgw, geo.wing.Kgc, 2, geo.wing.epw, geo.wing.ec, geo.wing.epc, ...
        %                                     str.wing.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        BULKdataPBARGMW(fid, stick.PID.wing(i), stick.MAT1.wing, str.wing.tWstick(i),...
            str.wing.tCstick(i), str.wing.dWstick(i), ...
            geo.wing.Zsstick(i), geo.wing.tbsstick(i), ...
            geo.wing.Kgw, geo.wing.Kgc, 2, geo.wing.epw, geo.wing.ec, geo.wing.epc, ...
            str.wing.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        
    end
    
end
%----------------------------------------------------------------------------------------------------------------------

%----------------------------------------------------------------------------------------------------------------------
% Vertical tail
if isequal(stick.model.vert, 1)
         
    % Interpolate thickness of cover, chord and height of wing box
    geo.vtail.tbsstick = interp1(geo.vtail.y_nodes_thick, geo.vtail.tbs, geo.vtail.y_nodes_1_2);
    geo.vtail.Zsstick  = interp1(geo.vtail.y_nodes_thick, geo.vtail.Zs , geo.vtail.y_nodes_1_2);
    str.vtail.tCstick  = interp1(geo.vtail.y_nodes_thick, str.vtail.tC , geo.vtail.y_nodes_1_2);
    str.vtail.tWstick  = interp1(geo.vtail.y_nodes_thick, str.vtail.tW , geo.vtail.y_nodes_1_2);
    str.vtail.dWstick  = interp1(geo.vtail.y_nodes_thick, str.vtail.dW , geo.vtail.y_nodes_1_2);
    
    % Get column vector
    geo.vtail.tbsstick = go2col(geo.vtail.tbsstick);
    geo.vtail.Zsstick  = go2col(geo.vtail.Zsstick);
    str.vtail.tCstick  = go2col(str.vtail.tCstick);
    str.vtail.tWstick  = go2col(str.vtail.tWstick);
    str.vtail.dWstick  = go2col(str.vtail.dWstick);
    
    % Beam element mass
    str.vtail.Mstick = interp1(geo.vtail.y_nodes_1_2_thick, str.vtail.mbox, geo.vtail.y_nodes_1_2);
    
    % Beam element area
    str.vtail.Astick = interp1(geo.vtail.y_nodes_1_2_thick,...
        str.vtail.mbox./(pdcylin.vtail.dsw*diff(geo.vtail.y_nodes_thick)), geo.vtail.y_nodes_1_2, 'linear');

    % Determine lumped area to set in 4 corners for simplest wing box type (PBARSM1)
    A4 = 2.*(geo.vtail.tbsstick.*str.vtail.tWstick + geo.vtail.Zsstick.*str.vtail.tCstick);
    str.vtail.Abstick = (str.vtail.Astick -A4)/ 4;
    if any(str.vtail.Abstick < 0)
        error('Negative area for PBARSMX2 card');
    end
    
    for i = 1:length(stick.PID.vert)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.vert.str_rec_coef.C(1,i);
        C2 = stick.PBAR.vert.str_rec_coef.C(2,i);
        D1 = stick.PBAR.vert.str_rec_coef.D(1,i);
        D2 = stick.PBAR.vert.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        
        % PBARSM1 card
        %        BULKdataPBARSM1(fid, stick.PID.vert(i), stick.MAT1.vert, str.vtail.Abstick(i),...
        %                                     str.vtail.tCstick(i), geo.vtail.Zstick(i),...
        %                                     geo.vtail.tbsstick(i), str.vtail.NSM.dstr(i),...
        %                                     C1, C2, D1, D2, E1, E2, F1, F2)
        
        %        BULKdataPBARGMW(fid, stick.PID.vert(i), stick.MAT1.vert, str.vtail.tWstick(i)*geo.vtail.Kgw,...
        %                                     str.vtail.tCstick(i)*geo.vtail.Kgc, str.vtail.dWstick(i), ...
        %                                     geo.vtail.Zsstick(i), geo.vtail.tbsstick(i), ...
        %                                     geo.vtail.Kgw, geo.vtail.Kgc, 2, geo.vtail.epw, geo.vtail.ec, geo.vtail.epc, ...
        %                                     str.vtail.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        BULKdataPBARGMW(fid, stick.PID.vert(i), stick.MAT1.vert, str.vtail.tWstick(i),... 
            str.vtail.tCstick(i), str.vtail.dWstick(i), ...
            geo.vtail.Zsstick(i), geo.vtail.tbsstick(i), ...
            geo.vtail.Kgw, geo.vtail.Kgc, 2, geo.vtail.epw, geo.vtail.ec, geo.vtail.epc, ...
            str.vtail.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        
        
    end
    
end
%----------------------------------------------------------------------------------------------------------------------
% Horizontal tail

if isequal(stick.model.horr, 1)
    
    % Interpolate thickness of cover, chord and height of wing box
    geo.htail.tbsstick = interp1(geo.htail.y_nodes_thick, geo.htail.tbs, geo.htail.y_nodes_1_2);
    geo.htail.Zsstick  = interp1(geo.htail.y_nodes_thick, geo.htail.Zs , geo.htail.y_nodes_1_2);
    str.htail.tCstick  = interp1(geo.htail.y_nodes_thick, str.htail.tC , geo.htail.y_nodes_1_2);
    str.htail.tWstick  = interp1(geo.htail.y_nodes_thick, str.htail.tW , geo.htail.y_nodes_1_2);
    str.htail.dWstick  = interp1(geo.htail.y_nodes_thick, str.htail.dW , geo.htail.y_nodes_1_2);
    
    % Get column vector
    geo.htail.tbsstick = go2col(geo.htail.tbsstick);
    geo.htail.Zsstick  = go2col(geo.htail.Zsstick);
    str.htail.tCstick  = go2col(str.htail.tCstick);
    str.htail.tWstick  = go2col(str.htail.tWstick);
    str.htail.dWstick  = go2col(str.htail.dWstick);
    
    % Check carry-through existence
    if geo.htail.twc > 0.0
        nrcth   = pdcylin.stick.nhtail_carryth;
        dy_carr = norm(stick.ptos.horrC2(:, 2)-stick.ptos.horrC2(:, 1))/nrcth;
        
        % Compute beam element area
        str.htail.Astick              = zeros(numel(geo.htail.y_nodes_1_2), 1);
        str.htail.Astick(1:nrcth)     = str.htail.m(1:nrcth)./(pdcylin.htail.dsw*dy_carr);
        str.htail.Astick(nrcth+1:end) = interp1(geo.htail.y_nodes_1_2_thick(nrcth+1:end),...
            str.htail.mbox./(pdcylin.htail.dsw*diff(geo.htail.y_nodes_thick(nrcth+1:end))), geo.htail.y_nodes_1_2(nrcth+1:end), 'linear');
    else
        % Otherwise compute only cantilever part        
        str.htail.Astick =  interp1(geo.htail.y_nodes_1_2_thick,...
            str.htail.mbox./(pdcylin.htail.dsw*diff(geo.htail.y_nodes_thick)), geo.htail.y_nodes_1_2, 'linear');
    end
    
    % Determine lumped area to set in 4 corners for simplest wing box type (PBARSM1)
    A4 = 2.*(geo.htail.tbsstick.*str.htail.tWstick + geo.htail.Zsstick.*str.htail.tCstick);
    str.htail.Abstick = (str.htail.Astick - A4)/4;
    
    if any(str.htail.Abstick < 0)
        error('Negative area for PBARSMX2 card');
    end
    
    for i = 1:length(stick.PID.hori)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.hori.str_rec_coef.C(1,i);
        C2 = stick.PBAR.hori.str_rec_coef.C(2,i);
        D1 = stick.PBAR.hori.str_rec_coef.D(1,i);
        D2 = stick.PBAR.hori.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        
        % PBARSM1 card
        %        BULKdataPBARSM1(fid, stick.PID.hori(i), stick.MAT1.hori, str.htail.Abstick(i),...
        %                                     str.htail.tCstick(i), geo.htail.Zstick(i),...
        %                                     geo.htail.tbsstick(i), str.htail.NSM.dstr(i),...
        %                                     C1, C2, D1, D2, E1, E2, F1, F2)
        %        BULKdataPBARGMW(fid, stick.PID.hori(i), stick.MAT1.hori, str.htail.tWstick(i)*geo.htail.Kgw,...
        %                                     str.htail.tCstick(i)*geo.htail.Kgc, str.htail.dWstick(i), ...
        %                                     geo.htail.Zsstick(i), geo.htail.tbsstick(i), ...
        %                                     geo.htail.Kgw, geo.htail.Kgc, 2, geo.htail.epw, geo.htail.ec, geo.htail.epc, ...
        %                                     str.htail.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        BULKdataPBARGMW(fid, stick.PID.hori(i), stick.MAT1.hori, str.htail.tWstick(i),...
            str.htail.tCstick(i), str.htail.dWstick(i), ...
            geo.htail.Zsstick(i), geo.htail.tbsstick(i), ...
            geo.htail.Kgw, geo.htail.Kgc, 2, geo.htail.epw, geo.htail.ec, geo.htail.epc, ...
            str.htail.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        
    end
    
end
%----------------------------------------------------------------------------------------------------------------------

%----------------------------------------------------------------------------------------------------------------------
% Canard
if isequal(stick.model.canr, 1)
    
    % Interpolate thickness of cover, chord and height of wing box
    geo.canard.tbsstick = interp1(geo.canard.y_nodes_thick, geo.canard.tbs, geo.canard.y_nodes_1_2);
    geo.canard.Zstick   = interp1(geo.canard.y_nodes_thick, geo.canard.Zs , geo.canard.y_nodes_1_2);
    str.canard.tCstick  = interp1(geo.canard.y_nodes_thick, str.canard.tC , geo.canard.y_nodes_1_2);
    str.canard.tWstick  = interp1(geo.canard.y_nodes_thick, str.canard.tW , geo.canard.y_nodes_1_2);
    str.canard.I1stick  = interp1(geo.canard.y_nodes_thick, str.canard.I1 , geo.canard.y_nodes_1_2);
    str.canard.dWstick  = interp1(geo.canard.y_nodes_thick, str.canard.dW , geo.canard.y_nodes_1_2);
    
    % Get column vector
    geo.canard.tbsstick = go2col(geo.canard.tbsstick);
    geo.canard.Zstick   = go2col(geo.canard.Zstick);
    str.canard.tCstick  = go2col(str.canard.tCstick);
    str.canard.tWstick  = go2col(str.canard.tWstick);
    str.canard.I1stick  = go2col(str.canard.I1stick);
    
    % Beam element mass
    str.canard.Mstick = interp1(geo.canard.y_nodes_1_2_thick, str.canard.m, geo.canard.y_nodes_1_2);
    
    % Check carry-through existence
    if geo.canard.twc > 0.0
        nrcth   = pdcylin.stick.ncanard_carryth;
        dy_carr = norm(stick.ptos.canrC2(:, 2)-stick.ptos.canrC2(:, 1))/nrcth;
        
        % Compute beam element area
        str.canard.Astick              = zeros(numel(geo.canard.y_nodes_1_2), 1);
        str.canard.Astick(1:nrcth)     = str.canard.m(1:nrcth)./(pdcylin.canard.dsw*dy_carr);
        str.canard.Astick(nrcth+1:end) = interp1(geo.canard.y_nodes_1_2_thick(nrcth+1:end),...
            str.canard.mbox./(pdcylin.canard.dsw*diff(geo.canard.y_nodes_thick(nrcth+1:end))), geo.canard.y_nodes_1_2(nrcth+1:end), 'linear');
    else
        % Otherwise compute only cantilever part        
        str.htail.Astick =  interp1(geo.canard.y_nodes_1_2_thick,...
            str.canard.mbox./(pdcylin.canard.dsw*diff(geo.canard.y_nodes_thick)), geo.canard.y_nodes_1_2, 'linear');
    end
    
    % Determine lumped area to set in 4 corners for simplest wing box type (PBARSM1)
    A4 = 2.*(geo.canard.tbsstick.*str.canard.tWstick + geo.canard.Zstick.*str.canard.tCstick);
    str.canard.Abstick = (str.canard.Astick - A4)/4;
    
    if any(str.canard.Abstick < 0)
        error('Negative area for PBARSMX2 card');
    end
    
    for i = 1:length(stick.PID.canr)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.canr.str_rec_coef.C(1,i);
        C2 = stick.PBAR.canr.str_rec_coef.C(2,i);
        D1 = stick.PBAR.canr.str_rec_coef.D(1,i);
        D2 = stick.PBAR.canr.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        
        % PBARSM1 card
        %            BULKdataPBARSM1(fid, stick.PID.wing2(i), stick.MAT1.wing2, str.wing2.Astick(i), str.wing2.I1stick(i),...
        %                str.wing2.tCstick(i), geo.wing2.Zstick(i),...
        %                geo.wing2.tbsstick(i), str.wing2.NSM.dstr(i),...
        %                C1, C2, D1, D2, E1, E2, F1, F2);
        
        %        BULKdataPBARGMW(fid, stick.PID.wing2(i), stick.MAT1.wing2, str.wing2.tWstick(i)*geo.wing2.Kgw,...
        %                                     str.wing2.tCstick(i)*geo.wing2.Kgc, str.wing2.dWstick(i), ...
        %                                     geo.wing2.Zstick(i), geo.wing2.tbsstick(i), ...
        %                                     geo.wing2.Kgw, geo.wing2.Kgc, 2, geo.wing2.epw, geo.wing2.ec, geo.wing2.epc, ...
        %                                     str.wing2.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        BULKdataPBARGMW(fid, stick.PID.canr(i), stick.MAT1.canr, str.canard.tWstick(i),...
            str.canard.tCstick(i), str.canard.dWstick(i), ...
            geo.canard.Zstick(i), geo.canard.tbsstick(i), ...
            geo.canard.Kgw, geo.canard.Kgc, 2, geo.canard.epw, geo.canard.ec, geo.canard.epc, ...
            str.canard.NSM.dstr(i), C1, C2, D1, D2, E1, E2, F1, F2);
        
    end
    
end

fprintf(outf, 'done.');

end % end of writePBAR2file.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------------------------------------


