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

function writeSPLINE22file(outf, fid, stick, aircraft, RMAX, TCOND)
%--------------------------------------------------------------------------------------------------
% SPLINE2 card
%
% Inputs:       fid, indicating file to write in
%
%
% Called by:    MainCode.m
%
% Calls:        BULKdataSPLINE2.m
%
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% modified by Travaglini 19/11/2009, adding new spline on new CAERO1
% defined on wing and replacing wing2 with canard

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Interpolation definition');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

fprintf(outf, '\n\tExporting spatial coupling interpolation parameters...');

%--------------------------------------------------------------------------
% Wing
%--------------------------------------------------------------------------

if isequal(stick.model.winr, 1)
    
    for i = 1:length(stick.IDCAERO1.winr)
        % Define inputs to function
        ID       = stick.IDCAERO1.winr(i);
        IDCAERO1 = stick.IDCAERO1.winr(i);
        LPAN     = stick.nTOT.wing(i);
        IDSET    = stick.IDSET.winr;
        % Recall function
        BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
        
    end
    if ~isempty(stick.IDCAERO1.winrP)
        for j = 1:length(stick.IDCAERO1.winrP)
            ID       = stick.IDCAERO1.winrP(j);
            IDCAERO1 = stick.IDCAERO1.winrP(j);
            LPAN     = stick.nTOT.wingP(j);
            IDSET    = stick.IDSET.winr;
            % Recall function
            BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
        end
    end
        
    if isequal(stick.model.winl, 1)
        
        for i = 1:length(stick.IDCAERO1.winr)
            % Define inputs to function
            ID       = stick.IDCAERO1.winl(i);
            IDCAERO1 = stick.IDCAERO1.winl(i);
            LPAN     = stick.nTOT.wing(i);
            IDSET    = stick.IDSET.winl;
            % Recall function
            BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
        end
        if ~isempty(stick.IDCAERO1.winlP)
            for j = 1:length(stick.IDCAERO1.winlP)
                ID       = stick.IDCAERO1.winlP(j);
                IDCAERO1 = stick.IDCAERO1.winlP(j);
                LPAN     = stick.nTOT.wingP(j);
                IDSET    = stick.IDSET.winl;
                % Recall function
                BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
            end
        end
    end
    
end

%--------------------------------------------------------------------------
% Vertical tail
%--------------------------------------------------------------------------

if isequal(stick.model.vert, 1)
    
    for i = 1:length(stick.IDCAERO1.vert)
        % Define inputs to function
        ID       = stick.IDCAERO1.vert(i);
        IDCAERO1 = stick.IDCAERO1.vert(i);
        LPAN     = stick.nTOT.vert(i);
        IDSET    = stick.IDSET.vert;
        % Recall function
        BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
    end
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        for i = 1:length(stick.IDCAERO1.vert2)
            % Define inputs to function
            ID       = stick.IDCAERO1.vert2(i);
            IDCAERO1 = stick.IDCAERO1.vert2(i);
            LPAN     = stick.nTOT.vert(i);
            IDSET    = stick.IDSET.vert2;
            % Recall function
            BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
        end
        
    end
    
    
    
end



%--------------------------------------------------------------------------
% Horizontal tail
%--------------------------------------------------------------------------

if isequal(stick.model.horr, 1)
    
    for i = 1:length(stick.IDCAERO1.horr)
        % Define inputs to function
        ID       = stick.IDCAERO1.horr(i);
        IDCAERO1 = stick.IDCAERO1.horr(i);
        LPAN     = stick.nTOT.hori(i);
        IDSET    = stick.IDSET.horr;
        % Recall function
        BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
    end
    
    if isequal(stick.model.horl, 1)
        
        for i = 1:length(stick.IDCAERO1.horr)
            % Define inputs to function
            ID       = stick.IDCAERO1.horl(i);
            IDCAERO1 = stick.IDCAERO1.horl(i);
            LPAN     = stick.nTOT.hori(i);
            IDSET    = stick.IDSET.horl;
            % Recall function
            BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
        end
        
    end
    
end



%--------------------------------------------------------------------------
% Canard
%--------------------------------------------------------------------------

if isequal(stick.model.canr, 1) && aircraft.Strut_wing.present~=1
    
    for i = 1:length(stick.IDCAERO1.canr)
        % Define inputs to function
        ID       = stick.IDCAERO1.canr(i);
        IDCAERO1 = stick.IDCAERO1.canr(i);
        LPAN     = stick.nTOT.canr(i);
        IDSET    = stick.IDSET.canr;
        % Recall function
        BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
    end
    
    if isequal(stick.model.canl, 1)
        
        for i = 1:length(stick.IDCAERO1.canr)
            % Define inputs to function
            ID       = stick.IDCAERO1.canl(i);
            IDCAERO1 = stick.IDCAERO1.canl(i);
            LPAN     = stick.nTOT.canr(i);
            IDSET    = stick.IDSET.canl;
            % Recall function
            BULKdataSPLINE2(fid, ID, IDCAERO1, 1, LPAN, IDSET, 2, RMAX, TCOND);
        end
        
    end
    
end


fprintf(outf, 'done.');
