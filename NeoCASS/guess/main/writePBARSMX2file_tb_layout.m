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

function [geo, str, stick] = writePBARSMX2file_tb_layout(outf, fid, pdcylin, geo, loads, str, stick, stype)

switch stype
    
    %--------------------------------------------------------------------------
    case 1
        
        switch(pdcylin.tbooms.kcon)
            
            case {1,2,3,4,5}
                
                for i = 1:length(stick.PID.tbooms)
                    % PBARGFF card
                    BULKdataPBARGFF(fid, stick.PID.tbooms(i), stick.MAT1.tbooms, str.tbooms.tbar_Sstick(i),...
                        str.tbooms.tbar_Afstick(i), str.tbooms.tbar_dstick(i), geo.tbooms.rstick(i), str.tbooms.NSM.dstr(i),...
                        geo.tbooms.m, geo.tbooms.epsilon, geo.tbooms.Kmg, geo.tbooms.Kp, pdcylin.wing.cf, ...
                        pdcylin.tbooms.ckf, pdcylin.tbooms.df, pdcylin.tbooms.ef,...
                        geo.tbooms.R, 0,...
                        0, geo.tbooms.R,...
                        -geo.tbooms.R, 0,...
                        0, -geo.tbooms.R);
                end
                
            case {6,7}
                
                for i = 1:length(stick.PID.tbooms)
                    % PBARGFU card
                    BULKdataPBARGFU(fid, stick.PID.tbooms(i), stick.MAT1.tbooms, str.tbooms.tbar_Sstick(i), geo.tbooms.rstick(i),...
                        geo.tbooms.m, geo.tbooms.epsilon, str.tbooms.NSM.dstr(i), geo.tbooms.Kmg, geo.tbooms.Kp,...
                        geo.tbooms.R, 0,...
                        0, geo.tbooms.R,...
                        -geo.tbooms.R, 0,...
                        0, -geo.tbooms.R);
                end
                
            otherwise
                
                error('Unknown value for tailbooms kcon.');
                
        end
        
        %--------------------------------------------------------------------------
    case 2
        
        % distance between frames within each beam
        stick.tbooms.d_frame = zeros(length(stick.PID.tbooms), 1);
        % number of beams ending with frames (within the beam length)
        stick.tbooms.nrd = zeros(length(stick.PID.tbooms), 1);
        % compressive load correspondent to min frame spacing
        stick.tbooms.Nxc = zeros(length(stick.PID.tbooms), 1);
        
        for i = 1:length(stick.PID.tbooms)
            
            idl = find(geo.tbooms.x >= geo.tbooms.x_nodes(i));
            idu = find(geo.tbooms.x < geo.tbooms.x_nodes(i+1));
            
            % Select the min number of stringers within each single beam, not considering the zeros
            d_frame = str.tbooms.d( idl(1):idu(end) );
            Nxc = loads.tbooms.Nxc( idl(1):idu(end) );
            zero_finder = find( d_frame == 0 );
            if ~isempty(zero_finder)
                d_frame(zero_finder) = [];
                Nxc(zero_finder) = [];
            end
            
            % Distance between frames within each beam
            [stick.tbooms.d_frame(i), indx] = min( d_frame );
            stick.tbooms.Nxc(i) = Nxc(indx);
            
        end
        
        % Define the number of frames for each single beam
        for i = 1:length(stick.PID.tbooms)
            
            if stick.tbooms.d_frame(i) <= stick.tbooms.Lbeam(i)
                % frames spacing smaller than beam length
                stick.tbooms.nrd(i) = ceil(stick.tbooms.Lbeam(i) /stick.tbooms.d_frame(i));
            else
                % frames spacing greater than beam length
                stick.tbooms.nrd(i) = 0;
            end
            
        end
        
        %------------------------------------------------------------------
        % Evaluate struct concept parameters
        concept = pdcylin.tbooms.kcon;
        tbar_S = str.tbooms.tbar_Sstick;
        d = stick.tbooms.d_frame;
        Nxc = stick.tbooms.Nxc;
        E = pdcylin.tbooms.es;
        [cross_section] = fuse_croxsec_layout(concept, tbar_S, d, Nxc, E);
        
        % Select the PBARSMX card
        switch pdcylin.tbooms.kcon
            
            case 1 % Unflanged
                
                for i = 1:length(stick.PID.tbooms)
                    % PBARSM3 card
                    BULKdataPBARSM3(fid, stick.PID.tbooms(i), stick.MAT1.tbooms, geo.tbooms.rstick(i), str.tbooms.NSM.dstr(i),...
                        cross_section(i,1), cross_section(i,2), cross_section(i,3), cross_section(i,4),...
                        stick.tbooms.nrd(i),...
                        geo.tbooms.R, 0,...
                        0, geo.tbooms.R,...
                        -geo.tbooms.R, 0,...
                        0, -geo.tbooms.R);
                end
                
            case {2, 3, 4} % Z-stiffened
                
                for i = 1:length(stick.PID.tbooms)
                    % PBARSM4 card
                    BULKdataPBARSM4(fid, stick.PID.tbooms(i), stick.MAT1.tbooms, geo.tbooms.rstick(i), str.tbooms.NSM.dstr(i),...
                        cross_section(i,1), cross_section(i,2), cross_section(i,3),...
                        cross_section(i,4), cross_section(i,5), cross_section(i,6),...
                        stick.tbooms.nrd(i),...
                        geo.tbooms.R, 0,...
                        0, geo.tbooms.R,...
                        -geo.tbooms.R, 0,...
                        0, -geo.tbooms.R);
                end
                
            case {5, 6, 7} % Sandwich
                
                for i = 1:length(stick.PID.tbooms)
                    % PBARSM5 card
                    BULKdataPBARSM5(fid, stick.PID.tbooms(i), stick.MAT1.tbooms, geo.tbooms.rstick(i), str.tbooms.NSM.dstr(i),...
                        cross_section(i,1), cross_section(i,2), cross_section(i,3), cross_section(i,4),...
                        cross_section(i,5),...
                        stick.tbooms.nrd(i),...
                        geo.tbooms.R, 0,...
                        0, geo.tbooms.R,...
                        -geo.tbooms.R, 0,...
                        0, -geo.tbooms.R);
                end
                
        end
        
end
