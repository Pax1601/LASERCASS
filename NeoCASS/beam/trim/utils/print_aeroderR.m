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
function print_aeroderR(fid, RStab_Der, nr)

fprintf(fid, '\n\nAERODYNAMIC DERIVATIVES\n');

fprintf(fid,'\nALPHA\n');
fprintf(fid,'\n NAME                RIGID               ');
fprintf(fid,'\n Cy/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcs_dalpha);   fprintf(fid,'%s',r);                                                                       
fprintf(fid,'\n Cz/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcl_dalpha);   fprintf(fid,'%s',r);
fprintf(fid,'\n Cl/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcml_dalpha);  fprintf(fid,'%s',r);
fprintf(fid,'\n Cm/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcmm_dalpha);  fprintf(fid,'%s',r);
fprintf(fid,'\n Cn/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcmn_dalpha);  fprintf(fid,'%s',r);
fprintf(fid,'\n\nBETA\n');
fprintf(fid,'\n NAME                RIGID               ');
fprintf(fid,'\n Cy/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcs_dbeta);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cz/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcl_dbeta);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cl/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcml_dbeta);     fprintf(fid,'%s',r);
fprintf(fid,'\n Cm/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcmm_dbeta);     fprintf(fid,'%s',r);
fprintf(fid,'\n Cn/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcmn_dbeta);     fprintf(fid,'%s',r);
fprintf(fid,'\n\nROLL RATE\n');
fprintf(fid,'\n NAME                RIGID               ');
fprintf(fid,'\n Cy/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcs_dP);       fprintf(fid,'%s',r);
fprintf(fid,'\n Cz/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcl_dP);       fprintf(fid,'%s',r);
fprintf(fid,'\n Cl/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcml_dP);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cm/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcmm_dP);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cn/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcmn_dP);      fprintf(fid,'%s',r);
fprintf(fid,'\n\nPITCH RATE\n');
fprintf(fid,'\n NAME                RIGID               ');
fprintf(fid,'\n Cy/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcs_dQ);       fprintf(fid,'%s',r);
fprintf(fid,'\n Cz/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcl_dQ);       fprintf(fid,'%s',r);
fprintf(fid,'\n Cl/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcml_dQ);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cm/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcmm_dQ);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cn/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcmn_dQ);      fprintf(fid,'%s',r);
fprintf(fid,'\n\nYAW RATE\n');
fprintf(fid,'\n NAME                RIGID               ');
fprintf(fid,'\n Cy/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcs_dR);       fprintf(fid,'%s',r);
fprintf(fid,'\n Cz/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcl_dR);       fprintf(fid,'%s',r);
fprintf(fid,'\n Cl/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcml_dR);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cm/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcmm_dR);      fprintf(fid,'%s',r);
fprintf(fid,'\n Cn/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcmn_dR);      fprintf(fid,'%s',r);

for i=1:length(nr)
  fprintf(fid,'\n\n%s\n', RStab_Der.Control.Name{i});
  fprintf(fid,'\n NAME                RIGID               ');
%
  fprintf(fid,'\n Cy/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcs_dDelta(i));    fprintf(fid,'%s',r);
  fprintf(fid,'\n Cz/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcl_dDelta(i));    fprintf(fid,'%s',r);
  fprintf(fid,'\n Cl/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcml_dDelta(i));   fprintf(fid,'%s',r);
  fprintf(fid,'\n Cm/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcmm_dDelta(i));   fprintf(fid,'%s',r);
  fprintf(fid,'\n Cn/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcmn_dDelta(i));   fprintf(fid,'%s',r);
%
  fprintf(fid,'\n Ch/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcmh_dalpha(i));   fprintf(fid,'%s',r);
  fprintf(fid,'\n Ch/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcmh_dbeta(i));   fprintf(fid,'%s',r);
  fprintf(fid,'\n Ch/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcmh_dP(i));   fprintf(fid,'%s',r);
  fprintf(fid,'\n Ch/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcmh_dQ(i));   fprintf(fid,'%s',r);
  fprintf(fid,'\n Ch/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcmh_dR(i));   fprintf(fid,'%s',r);
%
  fprintf(fid,'\n Ch/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcmh_dDelta(i,i));   fprintf(fid,'%s',r);

%
end