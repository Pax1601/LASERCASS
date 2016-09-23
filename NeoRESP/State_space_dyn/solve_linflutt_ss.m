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
% The template of this solver comes from M. Ripepi 
% The code has been adapted to NeoRESP
%
function [eigV, om_vec,root] = solve_linflutt_ss(dyn_model, MACH)

fid = 1;
%
beam_model = dyn_model.beam;
%
vrange = [];
rho   = beam_model.Param.RHO_VG;
vmax  =  beam_model.Param.VMAX;
vstep = vmax/beam_model.Param.NVSTEP;
UU = [vstep:vstep:vmax];
linestyle = ['bo-';'gx-';'r+-';'c*-';'ms-';'yd-';'kv-';'b<-';'g>-';'rp-';'ch-';'m.-';'c+-';'m*-';'ys-';...
            'kd-';'bv-';'g<-';'r>-';'kp-';'md-';'m.-';'c+-';'m*-';'ys-';'kd-';'bv-';'g<-';'r>-';'kp-'];
nlines = length(linestyle);
%
MO_INDEX = [2 4 6];
MM = beam_model.Struct.Mmm(MO_INDEX,MO_INDEX);
KK = beam_model.Struct.Kmm(MO_INDEX,MO_INDEX);
ndof = size(MM,1);
%
% modal data
Res = beam_model.Struct;
% aero data
dlm = dyn_model.dlm;
b = dlm.aero.cref;
kk = dlm.aero.k;
nk = length(kk);
%-------------------------------------------------------------------------------
% Reference Mach aero database
[~,MINDEX] = min(abs(dlm.aero.M - MACH));
HH = dlm.data.Qhh(MO_INDEX,MO_INDEX,:,MINDEX);
nr = size(HH,1);nc = size(HH,2);
%
[Aid, Bid, Cid, Eid] = aero_ss(kk, HH);
nXa = size(Aid,1); % number of aerodynamic states
%
% Evaluation of the identified aerodynamic transfer matrix 
%HHid = zeros(nr,nc,nk);
%for i = 1:nk
%  p   = 1i*kk(i);
%  BBs = Bid{1} + p*Bid{2} + p^2*Bid{3};
%  EEs = Eid{1} + p*Eid{2} + p^2*Eid{3};
%  HHid(:,:,i) = Cid*( (p*eye(nXa) - Aid)\ BBs ) + EEs;
%end
%-------------------------------------------------------------------------------
% Damping
SDAMP = beam_model.Param.SDAMP;
CC = zeros(ndof, ndof);
if (SDAMP)
  DAMP = beam_model.Damp;
  fprintf(fid,'\n - Structural damping required: '); 
  if (beam_model.Param.KDAMP == 1)
    fprintf(fid,'viscous type.'); 
    CC = modal_damp(DAMP.g{SDAMP}, DAMP.Freq{SDAMP}, DAMP.Type(SDAMP), beam_model.Param.KDAMP, ...
                        beam_model.Struct.Mmm, beam_model.Struct.Omega./(2*pi));
  else
    fprintf(fid,'complex stiffness.'); 
  end
else
  fprintf(fid,'\n - No structural damping required.'); 
end
%-------------------------------------------------------------------------------
%%
% =========================================================================
%                          FLUTTER CALCULATION
% =========================================================================

% -------------------------------------------------------------------------
% Structural eigenvalues
% -------------------------------------------------------------------------
eigval  = eig(MM\KK,'nobalance');
[~,ord] = sort(imag(eigval),'descend');
eigval  = eigval(ord);
om_struct = sqrt(eigval);

Atot = [zeros(ndof) eye(ndof); -MM\KK -MM\CC ];
clear eigval eigvec
[Veigvec Deigval] = eig(Atot,'nobalance');
[dummy,ord] = sort(imag(diag(Deigval)),'descend');
eigval = diag(Deigval(ord,ord));
eigvec = Veigvec(:,ord);
sn_struct = eigval;
%
%
N  = length(UU);
% initialization
root   = zeros(N,2*ndof+nXa);
k_vec  = zeros(N,2*ndof+nXa);
g_vec  = zeros(N,2*ndof+nXa);
om_vec = zeros(N,2*ndof+nXa);

% support matrices
O  = zeros(ndof);
Oa = zeros(ndof,nXa);
I  = eye(ndof);
Ia = eye(nXa);

for nU = 1:length(UU)
    
    U    = UU(nU);
    qdyn = 0.5*rho*U^2;
    E0 = Eid{1};
    E1 = Eid{2}*(b/U)^+1;
    E2 = Eid{3}*(b/U)^+2;
    B0 = Bid{1}.*(b/U)^-1;
    B1 = Bid{2}.*(b/U)^0;
    B2 = Bid{3}.*(b/U)^+1;
    Aa = Aid.*(b/U)^-1;
    Ca = Cid;
%
    Mbar = MM - qdyn*E2;
    Cbar = CC - qdyn*E1;
    Kbar = KK - qdyn*E0;
    % Assembling aeroelastic matrices
%    EE = [ Mbar O Oa; O I Oa; ( B2*(Mbar\Cbar)-B1 ) (-B2/Mbar) Ia ];
%    AA = [ -Cbar I Oa; -Kbar O qdyn*Ca; B0 Oa' Aa ];
    EE = [ I O Oa; O Mbar Oa; Oa' -B2 Ia];
    AA = [ O I Oa; -Kbar -Cbar qdyn*Ca; B0 B1 Aa];
    Aae = EE\AA;
    % Aeroelastic eigenvalues
    [sX, sn] = eig(Aae);
    sn = diag(sn);
    nXae = size(Aae,1); % number of aeroelastic states
    if nU > 1 % reordering eigenvalues
        sntmp = sn;
        sXtmp = sX;
        clear sn;
        clear sX;
        iclosest = [];
        iv = 1:nXae;
        for idof = 1:nXae
            iv(iclosest) = [];
            if isreal(sntmp(idof))
                % if real and very small, the value is set to zero, so avoiding problems with the sign +0 and -0;
                if abs(sntmp(idof)) < 10^-6, sntmp(idof) = 0; end
                [dummy,iclosest] = min( abs(root(nU-1,iv).' - sntmp(idof) ));
            else
                [dummy,iclosest] = min( root(nU-1,iv).' - sntmp(idof) );
            end
            sn(iv(iclosest),1) = sntmp(idof);
            sX(:,iv(iclosest)) = sXtmp(:,idof);
        end
    else
      % mode tracking
save('pippo.mat')
      defo_modes = find(MO_INDEX>6);
      label = zeros(length(defo_modes),2);
      for k=defo_modes
          [dummy,label(k,1)] = min( abs(sn - sn_struct(k) ));
          [dummy,label(k,2)] = min( abs(sn + sn_struct(k) ));
      end
    end
    
    om  = imag(sn);
    sig = real(sn);
    kk  = om*b/U;
    gg  = 2*real(sn)./abs(sn);
    
    % Storage of the results for plotting
    root(nU,:)   = sn;
    k_vec(nU,:)  = kk;
    g_vec(nU,:)  = gg;
    om_vec(nU,:) = om;
    eigV(:,:,nU) = sX;
end

% -------------------------------------------------------------------------
% Searching flutter points
% -------------------------------------------------------------------------
iV = [1:length(UU)]; % check all velocities
instabmodes = find(real(root(end,:)>0)); % instable modes (s.t. real(eigval) > 0)

% initialization
V_flutter    = zeros( 1, 2*ndof+nXa );
f_flutter    = zeros( 1, 2*ndof+nXa );
root_flutter = zeros( 1, 2*ndof+nXa );
%flutterlabel = cell ( 1, 2*ndof+nXa );

for idof = instabmodes;
    
    if sum( g_vec(iV(1),idof)==g_vec(iV,idof) ) == length(UU(iV)) ...  % constant damping
            || sum( isnan(g_vec(iV,idof)) ) == length(UU(iV)) ... % isNaN
            || sum( g_vec(iV(1),idof)==g_vec(iV,idof) ) + sum( isnan(g_vec(iV,idof)) ) == length(UU(iV))...  % constant damping
            || sum( 2 == abs(g_vec(iV,idof)) ) == length(UU(iV))  % damping = -+2; i.e. zet = 1/2g = -+1
        V_flutter(idof) = NaN;
    else
        gneg = find(g_vec(iV,idof)<0);
        if ~isempty(gneg) && ~sum(isnan(g_vec(iV,idof)))
          V_flutter(idof) = interp1(g_vec(iV,idof),UU(iV),0);
        else
          fprintf(fid,'\n#### Warning: mode %d is always unstable. No crossing zero damping line.', idof);
          V_flutter(idof) = NaN;
        end
    end
    
    f_flutter(idof)    = interp1( UU(iV), om_vec(iV,idof)/(2*pi), V_flutter(idof) );
    root_flutter(idof) = interp1( UU(iV), root(iV,idof), V_flutter(idof) );
    
end

figure(1); close; figure(1);
% -------------------------------------------------------------------------
hold on

plot(real(sn_struct),imag(sn_struct),'kx','MarkerSize',10,'MarkerFaceColor','k')

% for legend purpose    
plot(real(root(1,:)),imag(root(1,:)),'ok','linewidth',1.,'MarkerSize',5,'MarkerFaceColor','k')
hleg = legend('coupled structural eigenvalues','approx. state-space model',1);
set(hleg,'FontSize',12);
set(hleg, 'Box', 'on');
set(hleg, 'EdgeColor','w');
set(hleg, 'Color', 'w');
count = 1;
for i = 1:nXae
    plot(real(root(:,i)),imag(root(:,i)),linestyle(count,:),'MarkerSize',2,'MarkerFaceColor','k')
    count = count+1;
    if (count > nlines) 
      count = 1;
    end
end
%for idof = instabmodes;
%    Vf = V_flutter(idof);
%    root_f = root_flutter(idof);
%    if imag(root_f) > 0
%        annotation('textarrow',[1 0]./20+0.525,[imag(root_f)+sign(imag(root_f))*3 imag(root_f)]./43+0.235,...
%            'string', strcat(num2str(round(Vf*10)/10),',{ }',flutterlabel{idof}),...
%            'FontSize',14,'FontWeight','Bold','HorizontalAlignment','center')
%        plot(real(root_f),imag(root_f),'ko','MarkerFaceColor',[1 1 1]/2,'MarkerSize',6)
%    end
%end
axes1 = gca;
set(axes1,'FontSize',14);
xlabel('Re','FontSize',16,'FontWeight','Bold'),
ylabel('Im','FontSize',16,'FontWeight','Bold'),
grid on, box on
%xlim([-10 10]) %([-20 20])
%ylim([-5 30]) %([-5 30])
title('Root locus','FontSize',14,'FontWeight','Bold'),
% -------------------------------------------------------------------------
figure(2); close; figure(2);
% -------------------------------------------------------------------------
hold on
count = 1;
for i = 1:nXae
    plot(UU,g_vec(:,i),linestyle(count,:),'MarkerSize',2,'MarkerFaceColor','k')
    count = count+1;
    if (count > nlines) 
      count = 1;
    end
end
for idof = instabmodes; %1:nXae
    Vf = V_flutter(idof);
    plot(Vf,0,'ko','MarkerFaceColor',[1 1 1]/2,'MarkerSize',6)
end
axes1 = gca;
set(axes1,'FontSize',14);
xlabel('U (m/s)','FontSize',14,'FontWeight','Bold')
ylabel('Damping g','FontSize',14,'FontWeight','Bold')
grid on, box on
%xlim([0 350])
%ylim([-1 1])
title('Flutter damping','FontSize',14,'FontWeight','Bold'),
% -------------------------------------------------------------------------

figure(3)
% -------------------------------------------------------------------------
hold on
plot(UU,om_vec/(2*pi),'ko','MarkerSize',2,'MarkerFaceColor','k')
for idof = instabmodes; %1:nXae
    Vf     = V_flutter(idof);
    freq_f = f_flutter(idof);
    text(Vf,freq_f+0.12,num2str(round(freq_f*1000)/1000),...
        'FontSize',14,'FontWeight','Bold','HorizontalAlignment','center', 'BackgroundColor','w');
    plot(Vf,freq_f,'ko','MarkerFaceColor',[1 1 1]/2,'MarkerSize',6)
end
xlabel('U (ft/s)','FontSize',14,'FontWeight','Bold'),
ylabel('Frequency (Hz)','FontSize',14,'FontWeight','Bold')
grid on, box on
%xlim([0 300])
%ylim([0 4])
title('Flutter Frequency','FontSize',14,'FontWeight','Bold'),
% -------------------------------------------------------------------------

