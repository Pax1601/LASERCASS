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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci           <ricci@aero.polimi.it>
%                      Luca Cavagna           <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari  <degaspari@aero.polimi.it>
%                      Luca Riccobene         <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************

function print_attitude(fid, FM)
%
  fprintf(fid,'\n\nATTITUDE\n');
  if FM.Fixed(7)==1
    fprintf(fid,'\n X acc:      %g [m/s^2].', FM.Value(7));
  else
    fprintf(fid,'\n X acc:      UNKNOWN.');
  end
  if FM.Fixed(8)==1
    fprintf(fid,'\n Y acc:      %g [m/s^2].', FM.Value(8));
  else
    fprintf(fid,'\n Y acc:      UNKNOWN.');
  end
  if FM.Fixed(9)==1
    fprintf(fid,'\n Z acc:      %g [m/s^2].', FM.Value(9));
  else
    fprintf(fid,'\n Z acc:      UNKNOWN.');
  end
  if FM.Fixed(10)==1
    fprintf(fid,'\n P-DOT:      %g [rad/s^2].', FM.Value(10));
  else
    fprintf(fid,'\n P-DOT:      UNKNOWN.');
  end
  if FM.Fixed(11)==1
    fprintf(fid,'\n Q-DOT:      %g [rad/s^2].', FM.Value(11));
  else
    fprintf(fid,'\n Q-DOT:      UNKNOWN.');
  end
  if FM.Fixed(12)==1
    fprintf(fid,'\n R-DOT:      %g [rad/s^2].', FM.Value(12));
  else
    fprintf(fid,'\n R-DOT:      UNKNOWN.');
  end
  if FM.Fixed(2)==1
    fprintf(fid,'\n Alpha:      %g [deg].', FM.Value(2)*180/pi);
  else
    fprintf(fid,'\n Alpha:      UNKNOWN.');
  end
  if FM.Fixed(3)==1
    fprintf(fid,'\n Sideslip:   %g [deg].', FM.Value(3)*180/pi);
  else
    fprintf(fid,'\n Sideslip:      UNKNOWN.');
  end
  if FM.Fixed(4)==1
    fprintf(fid,'\n Roll rate:  %g [-] (p*BREF/(2VREF)).', FM.Value(4));
  else
    fprintf(fid,'\n Roll rate:      UNKNOWN.');
  end
  if FM.Fixed(5)==1
    fprintf(fid,'\n Pitch rate: %g [-] (q*CREF/(2VREF)).', FM.Value(5));
  else
    fprintf(fid,'\n Pitch rate:      UNKNOWN.');
  end
  if FM.Fixed(6)==1
    fprintf(fid,'\n Yaw rate:   %g [-] (r*BREF/(2VREF)).', FM.Value(6));
  else
    fprintf(fid,'\n Yaw rate:      UNKNOWN.');
  end
