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
%   Author: Luca Cavagna
%
function [nfig, Res] = rig_modes(fid, nfig, rho, Vel, M, c, b, S, m, I, RStab_Der, MINDEX, Res)
%
% estimated efficiency
E = 15;
%
Ixx = I(1);
Iyy = I(2);
Izz = I(3);
%----------------------------------------------
% long. derivatives
cla  =  RStab_Der.Alpha.dcl_dalpha(1,MINDEX);
cma  =  RStab_Der.Alpha.dcmm_dalpha(1,MINDEX);
cmq  =  RStab_Der.Q_rate.dcmm_dQ(1,MINDEX);
cmad =  RStab_Der.Alpha.dcmm_dalpha_dot(1,MINDEX);
% latero dir. derivatives
clp = RStab_Der.P_rate.dcml_dP(1,MINDEX);
clb = RStab_Der.Beta.dcml_dbeta(1,MINDEX);
cnb = RStab_Der.Beta.dcmn_dbeta(1,MINDEX);
clr = RStab_Der.R_rate.dcml_dR(1,MINDEX);
cnr = RStab_Der.R_rate.dcmn_dR(1,MINDEX);
cyb = RStab_Der.Beta.dcs_dbeta(1,MINDEX);
cyr = RStab_Der.R_rate.dcs_dR(1,MINDEX);
%
% Print summary
%
fprintf(fid, '\n- Evaluating rigid mode stability at Mach %g...', M);
fprintf(fid, '\n\n\tLongitudinal derivatives:');
fprintf(fid, '\n\t\tCz/alpha:    %g.', cla);
fprintf(fid, '\n\t\tCm/alpha:    %g.', cma);
fprintf(fid, '\n\t\tCm/q:        %g.', cmq);
fprintf(fid, '\n\t\tCm/alphadot: %g.', cmad);
%
fprintf(fid, '\n\n\tLatero-directional derivatives:');
fprintf(fid, '\n\t\tCl/p:        %g.', clp);
fprintf(fid, '\n\t\tCy/betha:    %g.', cyb);
fprintf(fid, '\n\t\tCl/betha:    %g.', clb);
fprintf(fid, '\n\t\tCn/betha:    %g.', cnb);
fprintf(fid, '\n\t\tCy/r:        %g.', cyr);
fprintf(fid, '\n\t\tCl/r:        %g.', clr);
fprintf(fid, '\n\t\tCn/r:        %g.\n', cnr);
%
%
nv = length(Vel);
damp_sp = zeros(nv,1);
damp_ph = zeros(nv,1);
damp_roll = zeros(nv,1);
damp_spiral = zeros(nv,1);
damp_dr = zeros(nv,1);
RE_sp = zeros(nv,1);
RE_ph = zeros(nv,1);
RE_roll = zeros(nv,1);
RE_spiral = zeros(nv,1);
RE_dr = zeros(nv,1);
IM_sp = zeros(nv,1);
IM_ph = zeros(nv,1);
IM_dr = zeros(nv,1);
omegan_sp = zeros(nv,1);
omegan_ph = zeros(nv,1);
omegan_dr = zeros(nv,1);
time12_sp = zeros(nv,1);
time12_ph = zeros(nv,1);
time12_dr = zeros(nv,1);
time12_roll = zeros(nv,1);
time12_spiral = zeros(nv,1);
cycle12_sp = zeros(nv,1);
cycle12_ph = zeros(nv,1);
cycle12_dr = zeros(nv,1);

for i=1:nv
%
  V = Vel(i);
  q = 0.5 * rho * V^2;
  CL = m*9.81/q/S;
  CD = CL / E;
  %
  % LONGITUDINAL STABILITY
  %
  cforce = q*S/m/V;
  cmom =   q*S*c/V/Iyy;
  %
  Zw  =-cforce * (cla+CD);
  Mw  =   cmom * cma;
  Mwd =   cmom * cmad * c/2/V;
  Mq  =   cmom * cmq  * c/2;
  %----------------------------------------------
  % short period
  A = [Zw V; Mw+Mwd*Zw Mq+Mwd*V];
  l_sp = eig(A);
  wd_sp = sqrt(l_sp(1)*l_sp(2));
  omegan_sp(i) = wd_sp;
  sigma = real(l_sp(1));
  if (sigma>0)
    fprintf(fid,'\n\t### Warning: short period unstable at %g m/s.',V);
  end
  RE_sp(i) = sigma;
  IM_sp(i) = abs(imag(l_sp(1)));
  eta_sp = -sigma/wd_sp; 
  damp_sp(i) = eta_sp;  
  T_sp = 2*pi/abs(imag(l_sp(1)));
  t12_sp = log(1/2)/sigma;
  time12_sp(i) = t12_sp;
  N12_sp = t12_sp / T_sp;
  cycle12_sp(i) = N12_sp;
  %----------------------------------------------
  % phugoid
  Xu = -2*cforce*CD;
  CLu = CL * M^2/(1-M^2);
  Zu = -cforce*(CLu + 2*CL);
  A = [Xu -9.81; -Zu/V 0];
  l_ph = eig(A);
  wd_ph = sqrt(l_ph(1)*l_ph(2));
  omegan_ph(i) = wd_ph;
  sigma = real(l_ph(1));
  if (sigma>0)
    fprintf(fid,'\n\t### Warning: phugoid unstable at %g m/s.',V);
  end
  RE_ph(i) = sigma;
  IM_ph(i) = abs(imag(l_ph(1)));
  eta_ph = -sigma/wd_ph;
  damp_ph(i) = eta_ph; 
  T_ph = 2*pi/abs(imag(l_ph(1)));
  t12_ph = log(1/2)/sigma;
  time12_ph(i) = t12_ph;
  N12_ph = t12_ph / T_ph;
  cycle12_ph(i) = N12_ph;
  %----------------------------------------------
  %
  % LATERO-DIRECTIONAL STABILITY
  %
  %----------------------------------------------
  cforcex = q*S*b/Ixx;
  cforcez = q*S*b/Izz;
  cmomx =   q*S*b^2/2/Ixx/V;
  cmomz =   q*S*b^2/2/Izz/V;
  % roll
  Lp = cmomx * clp;
  l_roll = Lp;
  if (l_roll>0)
    fprintf(fid,'\n\t### Warning: roll unstable at %g m/s.',V);
  end
  RE_roll(i) = l_roll;
  t12_roll = log(1/2)/l_roll;
  time12_roll(i) = t12_roll;
  % spiral
  Lb = cforcex * clb;
  Nb = cforcez * cnb;
  Lr = cmomx * clr;
  Nr = cmomz * cnr;
  l_spiral = (Lb * Nr - Lr * Nb)/Lb;
  RE_spiral(i) = l_spiral;
  if (l_spiral>0)
    fprintf(fid,'\n\t### Warning: spiral unstable at %g m/s.',V);
  end
  t12_spiral = log(1/2)/l_spiral;
  time12_spiral(i) = t12_spiral;
  % dutch roll
  Yb = q*S/m*cyb;
  Yr = q*S*b/2/m/V*cyr;
  A = [Yb/V -(1-Yr/V); Nb Nr];
  l_dr = eig(A);
  wd_dr = sqrt(l_dr(1)*l_dr(2));
  omegan_dr(i) = wd_dr;
  sigma = real(l_dr(1));
  if (sigma>0)
    fprintf(fid,'\n\t### Warning: dutch roll unstable at %g m/s.',V);
  end
  RE_dr(i) = sigma;
  IM_dr(i) = abs(imag(l_dr(1)));
  eta_dr = -sigma/wd_dr;
  damp_dr(i) = eta_dr; 
  T_dr = 2*pi/abs(imag(l_dr(1)));
  t12_dr = log(1/2)/sigma;
  time12_dr(i) = t12_dr;
  N12_dr = t12_dr / T_dr;
  cycle12_dr(i) = N12_dr;
end

if (nfig>-1)
  nfig = nfig +1;
  figure(nfig); close; figure(nfig); hold on;
  plot(Vel, damp_ph,'-ro')
  plot(Vel, damp_sp,'-ks')
  plot(Vel, damp_dr,'-g*')
  legend('Phugoid', 'Short period', 'Dutch roll');
  xlabel('Velocity');
  ylabel('Damping ratio');

  nfig = nfig +1;
  figure(nfig); close; figure(nfig); hold on;
  plot(Vel, omegan_ph,'-ro')
  plot(Vel, omegan_sp,'-ks')
  plot(Vel, omegan_dr,'-g*')
  legend('Phugoid', 'Short period', 'Dutch roll');
  xlabel('Velocity');
  ylabel('Natural frequency [rad/s]');

  nfig = nfig +1;
  figure(nfig); close; figure(nfig); hold on;
  plot(Vel, time12_ph,'-ro')
  plot(Vel, time12_sp,'-ks')
  plot(Vel, time12_dr,'-g*')
  plot(Vel, time12_roll,'-cx')
  plot(Vel, time12_spiral,'-mv')
  legend('Phugoid', 'Short period', 'Dutch roll', 'Roll', 'Spiral');
  xlabel('Velocity');
  ylabel('T1/2 [s]');

  nfig = nfig +1;
  figure(nfig); close; figure(nfig); hold on;
  plot(Vel, cycle12_ph,'-ro')
  plot(Vel, cycle12_sp,'-ks')
  plot(Vel, cycle12_dr,'-g*')
  legend('Phugoid', 'Short period', 'Dutch roll');
  xlabel('Velocity');
  ylabel('N1/2');

  nfig = nfig +1;
  figure(nfig); close; figure(nfig); hold on;
  plot(Vel, RE_ph,'-ro')
  plot(Vel, RE_sp,'-ks')
  plot(Vel, RE_dr,'-g*')
  plot(Vel, RE_roll,'-cx')
  plot(Vel, RE_spiral,'-mv')
  legend('Phugoid', 'Short period', 'Dutch roll', 'Roll', 'Spiral');
  xlabel('Velocity');
  ylabel('Real \lambda');
%
end
fprintf(fid, '\n  done.');
Res.label        = {'Phugoid', 'Short Period', 'Roll', 'Spiral', 'Dutch roll'};
Res.Velocity     = Vel;
%
Res.data(MINDEX).g{1}        = damp_ph;
Res.data(MINDEX).g{2}        = damp_sp;
Res.data(MINDEX).g{5}        = damp_dr;
%
Res.data(MINDEX).Freq{1}     = omegan_ph/2/pi;
Res.data(MINDEX).Freq{2}     = omegan_sp/2/pi;
Res.data(MINDEX).Freq{5}     = omegan_dr/2/pi;
%
Res.data(MINDEX).RealE{1}    = RE_ph;
Res.data(MINDEX).RealE{2}    = RE_sp;
Res.data(MINDEX).RealE{3}    = RE_roll;
Res.data(MINDEX).RealE{4}    = RE_spiral;
Res.data(MINDEX).RealE{5}    = RE_dr;
%
Res.data(MINDEX).ImagE{1}    = IM_ph;
Res.data(MINDEX).ImagE{2}    = IM_sp;
Res.data(MINDEX).ImagE{5}     = IM_dr;
