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
%
%***********************************************************************************************************************
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Lorenzo Travaglini   <>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by FFAST partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Luca Cavagna
%***********************************************************************************************************************
%
% Extract dynamic aero derivatives in a flight mechanic reference frame
% x - from the tail to the nose
% y - right semiwing
% z - along gravity
%
function RStab_Der = get_dynder(NMACH, K, CHREF, CREF, BREF, SREF, HAM, counter, MASTER)

ALPHA = 2*CHREF/CREF;
NK = length(K);

for n=1:NMACH
        % ALPHA
        RStab_Der.Alpha.dcl_dalpha(:,n)  = ...
          -CHREF*squeeze(imag(HAM(3,3,:,n)))'./(SREF.*K); % ok
        RStab_Der.Alpha.dcmm_dalpha(:,n)  = ...
          -CHREF*squeeze(imag(HAM(5,3,:,n)))'./(SREF.*K*CREF); % ok
        %   ALPHA dot derivatives
        RStab_Der.Alpha.dcl_dalpha_dot(:,n)  = ...
           -ALPHA.*CHREF.*squeeze(real(HAM(3,3,:,n)))'./(SREF.*K.^2);%ok
        RStab_Der.Alpha.dcmm_dalpha_dot(:,n) = ...
          -ALPHA.*CHREF.*squeeze(real(HAM(5,3,:,n)))'./(CREF*SREF.*K.^2);%ok
        %   PITCH DERIVATIVES
        data = zeros(1,NK);
        data(1:NK) = RStab_Der.Alpha.dcl_dalpha_dot(:,n);
        RStab_Der.Q_rate.dcl_dQ(:,n)  = ...
          ALPHA.*squeeze(imag(HAM(3,5,:,n)))'./(SREF.*K) - data; % ok
        data(1:NK) = RStab_Der.Alpha.dcmm_dalpha_dot(:,n);
        RStab_Der.Q_rate.dcmm_dQ(:,n)  = ...
          ALPHA.*squeeze(imag(HAM(5,5,:,n)))'./(K.*(SREF*CREF)) - data; % ok
        %   BETA DERIVATIVES
        RStab_Der.Beta.dcs_dbeta(:,n) = ...
              squeeze(real(HAM(2,6,:,n)))'./SREF; %ok
        RStab_Der.Beta.dcml_dbeta(:,n) = ...
          -squeeze(real(HAM(4,6,:,n)))'./(SREF*BREF); %ok
        RStab_Der.Beta.dcmn_dbeta(:,n) = ...
          -squeeze(real(HAM(6,6,:,n)))'./(SREF*BREF); %ok
        % BETA dot DERIVATIVES
        RStab_Der.Beta.dcs_dbeta_dot(:,n) = ...
              -(CREF^2).*squeeze(real(HAM(2,2,:,n)))'./(2*SREF*BREF.*K.^2);%ok
        RStab_Der.Beta.dcml_dbeta_dot(:,n) = ...
              (CREF^2).*squeeze(real(HAM(4,2,:,n)))'./(2*SREF*BREF^2.*K.^2); %ok
        RStab_Der.Beta.dcmn_dbeta_dot(:,n) = ...
              (2*CHREF^2).*squeeze(real(HAM(6,2,:,n)))'./(SREF*BREF^2.*K.^2); %ok
        %   ROLL DERIVATIVES
        RStab_Der.P_rate.dcs_dP(:,n) = ...
              -CREF.*squeeze(imag(HAM(2,4,:,n)))'./(SREF*BREF.*K); %ok
        RStab_Der.P_rate.dcml_dP(:,n) = ...
              CREF.*squeeze(imag(HAM(4,4,:,n)))'./(SREF*BREF^2.*K);  %ok
        RStab_Der.P_rate.dcmn_dP(:,n) = ...
              CREF.*squeeze(imag(HAM(6,4,:,n)))'./(SREF*BREF^2.*K);  %ok
        %   ROLL dot DERIVATIVES
        RStab_Der.P_rate.dcs_dP_dot(:,n) = ...
            (CREF^2).*squeeze(real(HAM(2,4,:,n)))'./(SREF*BREF^2.*K.^2);
        RStab_Der.P_rate.dcml_dP_dot(:,n) = ...
              -(CREF^2).*squeeze(real(HAM(4,4,:,n)))'./(SREF*BREF^3.*K.^2);
        RStab_Der.P_rate.dcmn_dP_dot(:,n) = ...
              -(CREF^2).*squeeze(real(HAM(6,4,:,n)))'./(SREF*BREF^3.*K.^2);
      %   YAW DERIVATIVES
        data(1:NK) = RStab_Der.Beta.dcs_dbeta_dot(:,n);
        RStab_Der.R_rate.dcs_dR(:,n) = ...
              -CREF.*squeeze(imag(HAM(2,6,:,n)))'./(SREF*BREF.*K) + data; %ok

        data(1:NK) = RStab_Der.Beta.dcml_dbeta_dot(:,n);
        RStab_Der.R_rate.dcml_dR(:,n) = ...
              CREF.*squeeze(imag(HAM(4,6,:,n)))'./(SREF*BREF^2.*K) + data; %ok

        data(1:NK) = RStab_Der.Beta.dcmn_dbeta_dot(:,n);
        RStab_Der.R_rate.dcmn_dR(:,n) = ...
              2*CHREF.*squeeze(imag(HAM(6,6,:,n)))'./(SREF*BREF^2.*K) + data;%ok
%
%   Control SURFACE derivatives
%
  if (counter>0)
    for cindex=1:counter
      col = cindex + 6;
      RStab_Der.Control.dcs_dDelta(:,n,cindex) = ...
        squeeze(real(HAM(2,col,:,n)))'./SREF;
      RStab_Der.Control.dcl_dDelta(:,n,cindex) = ...
        squeeze(real(HAM(3,col,:,n)))'./SREF;
      RStab_Der.Control.dcml_dDelta(:,n,cindex) = ...
        -squeeze(real(HAM(4,col,:,n)))'./(SREF*BREF);
      RStab_Der.Control.dcmm_dDelta(:,n,cindex) = ...
        squeeze(real(HAM(5,col,:,n)))'./(SREF*CREF);
      RStab_Der.Control.dcmn_dDelta(:,n,cindex) = ...
        -squeeze(real(HAM(6,col,:,n)))'./(SREF*BREF);
      RStab_Der.Control.dcmh_dDelta(:,n,cindex) = ...
        squeeze(real(HAM(col,col,:,n)))'./(SREF*CREF);
      RStab_Der.Control.Name = MASTER;
    end
  end
%
end