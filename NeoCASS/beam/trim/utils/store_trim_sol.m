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
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************

function SOL = store_trim_sol(fid, UDD, UX, NAME, print)

  if nargin == 4
      print = 1;
  end

  SOL = [];
  SOL.Control = [];
  SOL.ACC = UDD;
  SOL.Alpha = rad2deg(UX(1));
  SOL.Betha = rad2deg(UX(2));
  SOL.P = UX(3);
  SOL.Q = UX(4);
  SOL.R = UX(5);
  nc = 0;
  if (print ~= 0)
      fprintf(fid,'\n - X acc:      %g [m/s^2].', SOL.ACC(1));
      fprintf(fid,'\n - Y acc:      %g [m/s^2].', SOL.ACC(2));
      fprintf(fid,'\n - Z acc:      %g [m/s^2].', SOL.ACC(3));
      fprintf(fid,'\n - P-DOT:      %g [rad/s^2].', SOL.ACC(4));
      fprintf(fid,'\n - Q-DOT:      %g [rad/s^2].', SOL.ACC(5));
      fprintf(fid,'\n - R-DOT:      %g [rad/s^2].\n', SOL.ACC(6));

      fprintf(fid,'\n - Alpha:      %g [deg].', SOL.Alpha);
      fprintf(fid,'\n - Sideslip:   %g [deg].', SOL.Betha);
      fprintf(fid,'\n - Roll rate:  %g [-] (p*BREF/(2VREF)).', SOL.P);
      fprintf(fid,'\n - Pitch rate: %g [-] (q*CREF/(2VREF)).', SOL.Q);
      fprintf(fid,'\n - Yaw rate:   %g [-] (r*BREF/(2VREF)).', SOL.R);
  end
  for n=6:size(UX,1)
    nc = nc+1;
    SOL.Control(nc) = rad2deg(UX(n)); 
    fprintf(fid,'\n - Control %s:   %g [deg].', NAME{nc}, SOL.Control(nc));
  end

end