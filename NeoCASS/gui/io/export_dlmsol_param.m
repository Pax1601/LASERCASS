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
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080224      1.0     L.Cavagna        Creation
%     090312      1.3     A. De Gaspari    Modification
%     0904--      1.3     A. Scotti        Modification
%     090624      1.3.7   A. De Gaspari    Debugging last modification (MKAERO didn't work with less than 8 frequency values)
%
%*******************************************************************************
%
% function export_dlmsol_param(fp, mach, freq, cref, vmax, vpoints, rho, 
%                              simmxz, simmxy, dlm_order)
%
%   DESCRIPTION: Appends AERO and MKAERO1 cards to SMARTCAD file
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                fp             pointer    SMARTCAD file pointer
%                mach           real       Mach list
%                freq           real       Reduced frequency list
%                track          real       Modes tracking selection
%                modes          real       Qhh selected modes
%                cref           real       Reference chord
%                vmax           real       Maximum velocity for flutter
%                vpoints        integer    Number of velocity steps for flutter
%                rho            real       Density for flutter
%                simmxz         integer    Symm./Antis. along vertical plane
%                simmxy         integer    Symm./Antis. along horizontal plane
%                dlm_order      integer    Kernel order (1 or 2)
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************

function export_dlmsol_param(fp, mach, freq, track, modes, cref, vmax, vpoints, rho, simmxz, simmxy, dlm_order)
%
FIELD = 8;
%
str = num2str(cref, '%7g'); cref_str = [str, blanks(FIELD-length(str))];
str = num2str(vmax, '%7g'); vmax_str = [str, blanks(FIELD-length(str))];
str = num2str(vpoints, '%7g'); vpoints_str = [str, blanks(FIELD-length(str))];
str = num2str(rho, '%7g'); rho_str = [str, blanks(FIELD-length(str))];

str = num2str(simmxz, '%d'); simmxz_str = [str, blanks(FIELD-length(str))];
str = num2str(simmxy, '%d'); simmxy_str = [str, blanks(FIELD-length(str))];
str = num2str(dlm_order, '%d'); dlm_order_str = [str, blanks(FIELD-length(str))];
%
line = ['\nAERO    ', blanks(FIELD), vmax_str, cref_str, rho_str, simmxz_str, simmxy_str, ...
                      dlm_order_str, vpoints_str]; 
%
fprintf(fp, '\n$ Doublet lattice solver parameters');
fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fp, line);
%
fprintf(fp, '\n$ Doublet lattice aerodynamic transfer matrix points');
fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
%
NMACH  = length(mach);
NFREQ  = length(freq);
%
count = 0;
%
fnc = (NFREQ/8);


if fnc > 1
    count = 0;
    mstr = num2str(mach, '%8g');
    %mstr = [str, blanks(FIELD-length(str))];
    
    %    for j = 1:fnc
    for j = 1:(ceil(fnc))
        
        line = ['\nMKAERO1 ', mstr, '\n', blanks(FIELD)];
        fprintf(fp, line);
        count = count +8;
        for k=1:8
            
            if (NFREQ-((j-1)*8+k-1))>0
                
                str = num2str(freq((j-1)*8 + k), '%7g');
                fstr = [str, blanks(FIELD-length(str))];
                fprintf(fp, fstr);
                
            else
                
                
            end
        end
    end
    
else
    
    % write remainders
    rfnc = NFREQ - count;
    
    mstr = num2str(mach, '%8g');
    %mstr = [str, blanks(FIELD-length(str))];
    %
    line = ['\nMKAERO1 ', mstr, '\n', blanks(FIELD)];
    fprintf(fp, line);
    %
    for k=1:rfnc
        str = num2str(freq(count + k), '%7g');
        fstr = [str, blanks(FIELD-length(str))];
        fprintf(fp, fstr);
    end
end
%
%
%
%
%
%
% Field format
fmt = ['%', num2str(FIELD),'d']; 


NMODES = length(modes);

if NMODES,
    
    CNMODES = ceil(NMODES/8);
    modes_str = num2str(modes, fmt);
    modes_str = [blanks(FIELD-length(num2str(modes(1)))), modes_str];
    ts = rem(NMODES,8);
    if ts,
        modes_str = [modes_str, blanks(8*(8-ts))];
    end
    modes_mat = reshape(modes_str, FIELD*8, CNMODES)';
    front  = repmat(blanks(FIELD), CNMODES, 1);
    front(1,:) = 'MSELECT ';
    back = repmat('\n', CNMODES, 1);
    
    out = [front, modes_mat, back];
    
    fprintf(fp, '\n$ Selected modes defining the modal basis ');
    fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
    fprintf(fp, out');
    
end
%
%
%
NTRACK = length(track);
%
if NTRACK,
    
    CNTRACK = ceil(NTRACK/8);
    track_str = num2str(track, fmt);
    track_str = [blanks(FIELD-length(num2str(track(1)))), track_str];
    ts = rem(NTRACK,8);
    if ts,
        track_str = [track_str, blanks(8*(8-ts))];
    end
    track_mat = reshape(track_str, FIELD*8, CNTRACK)';
    front  = repmat(blanks(FIELD), CNTRACK, 1);
    front(1,:) = 'FMODES  ';
    back = repmat('\n', CNTRACK, 1);
    
    out = [front, track_mat, back];
    
    fprintf(fp, '\n$ Selected modes for tracking in V-g plot');
    fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
    fprintf(fp, out');
    
end



