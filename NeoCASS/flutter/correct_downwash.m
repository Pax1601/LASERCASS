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

function [rhs] = correct_downwash(fid, k, NMODES, Aero,Res, c_displ,cref)
% global beam_model
% this function correct the CP with terms due to x motion (on  dynamic
% pressure)
if isfield(Aero,'lattice_defo') && isstruct(Aero.lattice_defo)
    lattice = Aero.lattice_defo;
else
    lattice = Aero.lattice_vlm;
end 
% Aero = beam_model.Aero;
fprintf(fid,'\n Correct downwash with term due to x displacements');
np = size(Aero.lattice_vlm.COLLOC,1);

% firstly obtaining sweep angle (global) on each box 
% and tangent vector on surface

N = -lattice.N;

lambda = zeros(np,1);
tangent = cross(N,repmat([1,0,0],np,1));

for i = 1 : length(Aero.geo.SW)
    n1 = Aero.lattice_dlm.DOF(i,1,1);
    n2 = Aero.lattice_dlm.DOF(i,1,2);
    lambda(n1:n2) = Aero.geo.SW(i);
end




Alpha = zeros(np,NMODES);

if isfield(Res,'Aero')
    if isfield(Res.Aero,'DTrim_sol')
        alpha = Res.Aero.DTrim_sol.Alpha*pi/180;
        betha = Res.Aero.DTrim_sol.Betha*pi/180;
        Prate = Res.Aero.DTrim_sol.P; 
        Qrate = Res.Aero.DTrim_sol.Q;
        Rrate = Res.Aero.DTrim_sol.R;
    else
        alpha = Res.Aero.RTrim_sol.Alpha*pi/180;
        betha = Res.Aero.RTrim_sol.Betha*pi/180;
        Prate = Res.Aero.RTrim_sol.P;
        Qrate = Res.Aero.RTrim_sol.Q;
        Rrate = Res.Aero.RTrim_sol.R;
    end
    for j = 1: NMODES
        wind = 1*([cos(alpha)*cos(betha) sin(betha) sin(alpha)*cos(betha)]);
%         VEL = repmat(wind, np, 1);
        VEL = (c_displ(:,1,j) - tan(lambda).*dot(tangent,c_displ(:,:,j),2) )*wind; % c_displ(:,1,j)
        BODY_OMEGA = [Prate Qrate Rrate];
        if norm(BODY_OMEGA)
            
            ARM = (lattice.COLLOC - repmat(geo.CG, np, 1));
            
            OMEGA_P = cross(ARM, repmat(BODY_OMEGA, np, 1), 2);
            % add rigid body contributions to collocation point velocity
            VEL = VEL + OMEGA_P;
        end
     Alpha(:,j) = dot(VEL, N, 2);
    end
else
    fprintf(fid,'\n No Trim solution found!!!') 
    rhs = zeros(np,NMODES,length(k));
    return;
end


rhs = zeros(np,NMODES,length(k));




% corr = zeros(np,length(NMODES),length(k),NMACH);
% for m = 1 : NMACH
    for i = 1 : length(k)
%         for j = 1: NMODES
%             rhs(:,:,i) =(c_displ(:,1,j) - tan(lambda).*dot(tangent,c_displ(:,:,j),2) ).*sin(Alpha)*(k(i) / cref)*1i;
%         end
          rhs(:,:,i) = Alpha  *(k(i) / cref)*1i;
    end
% end

