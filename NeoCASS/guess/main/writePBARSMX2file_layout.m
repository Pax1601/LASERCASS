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

function [geo, str, stick] = writePBARSMX2file_layout(outf, fid, pdcylin, geo, loads, str, stick, stype)

switch stype
    
    %--------------------------------------------------------------------------
    case 1
        
        switch(pdcylin.fus.kcon)
            
            case {1,2,3,4,5}
                
                for i = 1:length(stick.PID.fuse)
                    % PBARGFF card
                    BULKdataPBARGFF(fid, stick.PID.fuse(i), stick.MAT1.fuse, str.fus.tbar_Sstick(i),...
                        str.fus.tbar_Afstick(i), str.fus.tbar_dstick(i), geo.fus.rstick(i), str.fus.NSM.dstr(i),...
                        geo.fus.m, geo.fus.epsilon, geo.fus.Kmg, geo.fus.Kp, pdcylin.wing.cf, ...
                        pdcylin.fus.ckf, pdcylin.fus.df, pdcylin.fus.ef,...
                        stick.PBAR.fuse.str_rec_coef.C1(i), stick.PBAR.fuse.str_rec_coef.C2(i),...
                        stick.PBAR.fuse.str_rec_coef.D1(i), stick.PBAR.fuse.str_rec_coef.D2(i),...
                        stick.PBAR.fuse.str_rec_coef.E1(i), stick.PBAR.fuse.str_rec_coef.E2(i),...
                        stick.PBAR.fuse.str_rec_coef.F1(i), stick.PBAR.fuse.str_rec_coef.F2(i));
                end
                
            case {6,7}
                
                for i = 1:length(stick.PID.fuse)
                    % PBARGFU card
                    BULKdataPBARGFU(fid, stick.PID.fuse(i), stick.MAT1.fuse, str.fus.tbar_Sstick(i), geo.fus.rstick(i),...
                        geo.fus.m, geo.fus.epsilon, str.fus.NSM.dstr(i), geo.fus.Kmg, geo.fus.Kp,...
                        stick.PBAR.fuse.str_rec_coef.C1(i), stick.PBAR.fuse.str_rec_coef.C2(i),...
                        stick.PBAR.fuse.str_rec_coef.D1(i), stick.PBAR.fuse.str_rec_coef.D2(i),...
                        stick.PBAR.fuse.str_rec_coef.E1(i), stick.PBAR.fuse.str_rec_coef.E2(i),...
                        stick.PBAR.fuse.str_rec_coef.F1(i), stick.PBAR.fuse.str_rec_coef.F2(i));
                end
                
            otherwise
                
                error('Unknown value for fuselage kcon.');
                
        end
        
        %--------------------------------------------------------------------------
    case 2
        
        % distance between frames within each beam
        stick.fus.d_frame = zeros(length(stick.PID.fuse), 1);
        % number of beams ending with frames (within the beam length)
        stick.fus.nrd = zeros(length(stick.PID.fuse), 1);
        % compressive load correspondent to min frame spacing
        stick.fus.Nxc = zeros(length(stick.PID.fuse), 1);
        
        for i = 1:length(stick.PID.fuse)
            
            idl = find(geo.fus.x >= geo.fus.x_nodes(i));
            idu = find(geo.fus.x < geo.fus.x_nodes(i+1));
            
            % Select the min number of stringers within each single beam, not considering the zeros
            d_frame = str.fus.d( idl(1):idu(end) );
            Nxc = loads.fus.Nxc( idl(1):idu(end) );
            zero_finder = find( d_frame == 0 );
            if ~isempty(zero_finder)
                d_frame(zero_finder) = [];
                Nxc(zero_finder) = [];
            end
            
            % Distance between frames within each beam
            [stick.fus.d_frame(i), indx] = min( d_frame );
            stick.fus.Nxc(i) = Nxc(indx);
            
        end
        
        % Define the number of frames for each single beam
        for i = 1:length(stick.PID.fuse)
            
            if stick.fus.d_frame(i) <= stick.fus.Lbeam(i)
                % frames spacing smaller than beam length
                stick.fus.nrd(i) = ceil(stick.fus.Lbeam(i) /stick.fus.d_frame(i));
            else
                % frames spacing greater than beam length
                stick.fus.nrd(i) = 0;
            end
            
        end
        
        %------------------------------------------------------------------
        % Evaluate struct concept parameters
        concept = pdcylin.fus.kcon;
        tbar_S = str.fus.tbar_Sstick;
        d = stick.fus.d_frame;
        Nxc = stick.fus.Nxc;
        E = pdcylin.fus.es;
        [cross_section] = fuse_croxsec_layout(concept, tbar_S, d, Nxc, E);
        
        % Select the PBARSMX card
        switch pdcylin.fus.kcon
            
            case 1 % Unflanged
                
                for i = 1:length(stick.PID.fuse)
                    % PBARSM3 card
                    BULKdataPBARSM3(fid, stick.PID.fuse(i), stick.MAT1.fuse, geo.fus.rstick(i), str.fus.NSM.dstr(i),...
                        cross_section(i,1), cross_section(i,2), cross_section(i,3), cross_section(i,4),...
                        stick.fus.nrd(i),...
                        stick.PBAR.fuse.str_rec_coef.C1(i), stick.PBAR.fuse.str_rec_coef.C2(i),...
                        stick.PBAR.fuse.str_rec_coef.D1(i), stick.PBAR.fuse.str_rec_coef.D2(i),...
                        stick.PBAR.fuse.str_rec_coef.E1(i), stick.PBAR.fuse.str_rec_coef.E2(i),...
                        stick.PBAR.fuse.str_rec_coef.F1(i), stick.PBAR.fuse.str_rec_coef.F2(i));
                end
                
            case {2, 3, 4} % Z-stiffened
                
                for i = 1:length(stick.PID.fuse)
                    % PBARSM4 card
                    BULKdataPBARSM4(fid, stick.PID.fuse(i), stick.MAT1.fuse, geo.fus.rstick(i), str.fus.NSM.dstr(i),...
                        cross_section(i,1), cross_section(i,2), cross_section(i,3),...
                        cross_section(i,4), cross_section(i,5), cross_section(i,6),...
                        stick.fus.nrd(i),...
                        stick.PBAR.fuse.str_rec_coef.C1(i), stick.PBAR.fuse.str_rec_coef.C2(i),...
                        stick.PBAR.fuse.str_rec_coef.D1(i), stick.PBAR.fuse.str_rec_coef.D2(i),...
                        stick.PBAR.fuse.str_rec_coef.E1(i), stick.PBAR.fuse.str_rec_coef.E2(i),...
                        stick.PBAR.fuse.str_rec_coef.F1(i), stick.PBAR.fuse.str_rec_coef.F2(i));
                end
                
            case {5, 6, 7} % Sandwich
                
                for i = 1:length(stick.PID.fuse)
                    % PBARSM5 card
                    BULKdataPBARSM5(fid, stick.PID.fuse(i), stick.MAT1.fuse, geo.fus.rstick(i), str.fus.NSM.dstr(i),...
                        cross_section(i,1), cross_section(i,2), cross_section(i,3), cross_section(i,4),...
                        cross_section(i,5),...
                        stick.fus.nrd(i),...
                        stick.PBAR.fuse.str_rec_coef.C1(i), stick.PBAR.fuse.str_rec_coef.C2(i),...
                        stick.PBAR.fuse.str_rec_coef.D1(i), stick.PBAR.fuse.str_rec_coef.D2(i),...
                        stick.PBAR.fuse.str_rec_coef.E1(i), stick.PBAR.fuse.str_rec_coef.E2(i),...
                        stick.PBAR.fuse.str_rec_coef.F1(i), stick.PBAR.fuse.str_rec_coef.F2(i));
                end
                
        end
        
end
