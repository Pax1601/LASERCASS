% =========================================================================
%       FLUTTER CALCULATION OF A 3DOF AIRFOIL IN INCOMPRESSIBLE FLOW
% =========================================================================
%
% Description:
% flutter calculation of an unrestrained three-degree-of-freedom (DOF)
% airfoil in incompressible flow, representing a 2-DOF pitch-plunge
% aeroelastic problem with an additional fuselage free–free plunge mode.
%
% -------------------------------------------------------------------------
%
%   Copyright (C) 2012 Matteo Ripepi      <ripepi@aero.polimi.it>
%
%   This program is free software; you can redistribute it and/or
%   modify it under the terms of the GNU General Public License as
%   published by the Free Software Foundation; either version 3 of the
%   License, or (at your option) any later version.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
%
%   Ref: Ripepi M., Mantegazza P., 'An improved matrix fraction approximation of
%        aerodynamic transfer matrices', AIAA journal, submitted for publication.
%
% -------------------------------------------------------------------------
%                            Other References
% -------------------------------------------------------------------------
% - "Aeroelasticity", Bisplinghoff, eqs. 5-311 and 5-312.
%
% - "Flutter and Divergence Analysis Using the Generalized Aeroelastic
%   Analysis Method", J. Edwards, C. Wieseman, NASA Langley Research Center
%   Journal of Aircraft 2008, 0021-8669 vol.45 no.3 (906-915)
%
% - "Unsteady aerodynamic modeling for arbitrary motions",
%    EDWARDS, J. W., NASA, Flight Research Center, Edwards, Calif.
%    AIAA Journal 1979, 0001-1452 vol.17 no.4 (365-374)
% =========================================================================

close all
clear all
clc

workdir = pwd;

%%
% =========================================================================
%                             USER PARAMETERS
% =========================================================================

% -------------------------------------------------------------------------
% Select Aeroelastic Case
% -------------------------------------------------------------------------
ndof = 3;    % number of DOFs (2 or 3)
CG   = '37'; % position of center of gravity ('37' or '45')

% -------------------------------------------------------------------------
% Rational Matrix Fraction Approximation Parameters
% -------------------------------------------------------------------------

% directory of the "improvedMFD" matlab function
RMFAdir = '/home/matteo/Documents/PhD/matlab/Identification/improvedMFD/';

% Identification options
opt{1} = 3;      % MFD order
opt{2} = 1;      % MFD algorithm
opt{3} = 'lmfd'; % left or right MFD
opt{4} = 2;      % ro parameter in treating discrete gusts using the RMFD
opt{5} = 100;    % weight parameter value W^2

% Levenberg-Marquardt parameters
tau     = 1e-3;
tolg    = 1e-4;
tolx    = 1e-6;
maxiter = 100;
optsLM = [tau tolg tolx maxiter];

% stabilizations parameters
eigsopt.threshold = -1e-4;
eigsopt.type      = 'bound';      % 'bound' or 'flip'
eigsopt.bound     =  -0.005;
eigsopt.method    = 'polesplace'; %'polesplace' or 'eigshift'

% Model reduction algorithm
algROM = 'balance'; % 'schur', 'hankel', 'bst'
restart = [];


%%
% =========================================================================
%                                  DATA
% =========================================================================
% Aerodynamic center: 25%c
% Elastic axis: 40%c
% Center of gravity: 37%c ==> xa = -0.06 or 45%c ==> xa = 0.10
ft2m  = 0.3048;
chord = 6;          % ft
chord = chord*ft2m; % m
b     = chord/2;    % half chord, m
a     = -0.20;      % b*a = distance of Elastic axis from half chord

% -------------------------------------------------------------------------
% Mass Matrix
% -------------------------------------------------------------------------
switch CG
    case '37', xa = -0.06;
    case '45', xa  = 0.10;
end

slug2kg = 14.5939029;

mw  = 1.3447*slug2kg;  %slugs
mf  = mw;
ra2 = 0.25;
mu  = 20; % mass ratio
rho = mw/(pi*mu*b^2); % density, kg/m^3

switch ndof
    case 2, MM = [1 xa; xa ra2];
    case 3, MM = [1 xa 0; xa ra2 0; 0 0 mf/mw];
end

% -------------------------------------------------------------------------
% Stiffness Matrix
% -------------------------------------------------------------------------
omh = 10; %rad/s
oma = 25; %rad/s
switch ndof
    case 2, KK = [omh^2 0 ; 0 ra2*oma^2;];
    case 3, KK = [omh^2 0 -omh^2; 0 ra2*oma^2 0; -omh^2 0 omh^2];
end

% -------------------------------------------------------------------------
% Damping Matrix
% -------------------------------------------------------------------------
g  = 0.03;
xi = 0.5*g;
switch ndof
    case 2, CC = [2*xi*omh 0; 0 2*xi*ra2*oma];
    case 3, CC = [2*xi*omh 0 0; 0 2*xi*ra2*oma 0; 0 0 0];
end

% -------------------------------------------------------------------------
% Aerodynamic Transfer Matrix QQ
% where F = qdyn*QQ*[h;alfa]
% and [-L;M] = (s^2*Mnc + s*Bnc + Knc + Ck*R*(s*S2 + S1) )*[h;alfa]
% -------------------------------------------------------------------------
U = 175;
qdyn = 0.5*rho*U^2; % Dynamic pressure

signL = -1;
signM = 1;
Mnc = pi*[ signL*[1 -a]; signM*[a -(1/8+a^2)] ];
Bnc = pi*[ signL*[0  1]; signM*[0   -(0.5-a)] ];
Knc = [signL*[0 0]; signM*[0 0] ];
R  = 2*pi*[ signL*1; (0.5+a) ];
S1 = [ 0 1 ];
S2 = [ 1 (0.5-a) ];

qhat = (U/b)^2/(pi*mu);

% Variable redefining to obtain the aerodynamic tranfer matrix Ha
% as stated in the paper: (cl*c,cm) = Ha*(h,alpha)
Mnc = Mnc*(b/U)^2;
Bnc = Bnc*(b/U);
S2  = S2*(b/U);

% Reduced frequencies
kk = [ 0
    0.0100
    0.0200
    0.0300
    0.0550
    0.0800
    0.1100
    0.1500
    0.2000
    0.2500
    0.3000
    0.4000
    0.5000
    0.7000
    0.9000
    1.1500
    1.5000
    2.0000
    3.0000
    4.0
    5.0];

% Aerodynamic tranfer matrix evaluation
nk = length(kk);
Ck = Theod(kk); % otherwise: Ck = GenTheod(1i*kk);
HH = zeros(2,2,nk);
for i = 1:nk
    om = kk(i)*U/b;
    s  = 1i*om;
    HH(:,:,i) = ( s^2*Mnc + s*Bnc + Knc + Ck(i)*R*(s*S2 + S1) );
end

% Building of a finer discretization of reduced frequency for a smoother
% plot of the aerodynamic transfer matrix
kkpl  = 0:.01:3;
nkpl  = length(kkpl);
Ckpl  = Theod(kkpl);
HHpl  = zeros(2,2,nkpl);
for i = 1:nkpl
    om = kkpl(i)*U/b;
    s  = 1i*om;
    HHpl(:,:,i) = ( s^2*Mnc + s*Bnc + Knc + Ckpl(i)*R*(s*S2 + S1) );
end

% storing aerodynamic transfer matrix for plotting
nHH = 1;
HHorig      = HHpl;
Heval{nHH}  = HHorig;
kkeval{nHH} = kkpl;

%%
% =========================================================================
%         LINEAR IDENTIFICATION THROUGH IMPROVED RMFA ALGORITHM
% =========================================================================
kidmax = 4;
ik = find(kk<=kidmax);
%cd(RMFAdir)
solution = improvedMFDfun(kk(ik),HH(:,:,ik),opt,optsLM,eigsopt,algROM,restart);
%cd(workdir)

% extract matrices from "solution" structure
clear Aid Bid Cid Did
Aid    = solution.inoutresid.AA;
Bid{1} = solution.inoutresid.BB{1};
Bid{2} = solution.inoutresid.BB{2};
Bid{3} = solution.inoutresid.BB{3};
Cid    = solution.inoutresid.CC;
Eid{1} = solution.inoutresid.DD{1};
Eid{2} = solution.inoutresid.DD{2};
Eid{3} = solution.inoutresid.DD{3};

nXa = size(Aid,1); % number of aerodynamic states

% Evaluation of the identified aerodynamic transfer matrix 
HHid = zeros(2,2,nk);
for i = 1:nk
    p   = 1i*kk(i);
    BBs = Bid{1} + p*Bid{2} + p^2*Bid{3};
    EEs = Eid{1} + p*Eid{2} + p^2*Eid{3};
    HHid(:,:,i) = Cid*( (p*eye(nXa) - Aid)\ BBs ) + EEs;
end

% Storage of the results for plotting
kmax = 3;
ik = find(kk<=kmax);
HHapprox = HHid(:,:,ik);
kkapprox = kk(ik);
nHH = 2;
Heval{nHH}  = HHapprox;
kkeval{nHH} = kkapprox;


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
sn_init = sn_struct;


% -------------------------------------------------------------------------
% Aeroelastic eigenvalues
% -------------------------------------------------------------------------
kmax    = 30;
maxiter = 100;
rfac    = 0.8; % relaxation factor

UU = (6:1:1000)*ft2m;
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

% 
for nU = 1:length(UU)
    
    U    = UU(nU);
    qdyn = 0.5*rho*U^2;
    qhat = (U/b)^2/(pi*mu);
    
    switch ndof
        case 2
            E0 = Eid{1};
            E1 = Eid{2}*(b/U)^+1;
            E2 = Eid{3}*(b/U)^+2;
            B0 = Bid{1}.*(b/U)^-1;
            B1 = Bid{2}.*(b/U)^0;
            B2 = Bid{3}.*(b/U)^+1;
            Aa = Aid.*(b/U)^-1;
            Ca = Cid;
            
        case 3
            E0 = blkdiag(Eid{1},0);
            E1 = blkdiag(Eid{2},0)*(b/U)^+1;
            E2 = blkdiag(Eid{3},0)*(b/U)^+2;
            B0 = [ Bid{1} zeros(nXa,1)].*(b/U)^-1;
            B1 = [ Bid{2} zeros(nXa,1)].*(b/U)^0;
            B2 = [ Bid{3} zeros(nXa,1)].*(b/U)^+1;
            Aa = Aid.*(b/U)^-1;
            Ca = [ Cid; zeros(1,nXa)];
    end
    
    Mbar = MM - qhat*E2;
    Cbar = CC - qhat*E1;
    Kbar = KK - qhat*E0;
    
    % Assembling aeroelastic matrices
    EE = [ Mbar O Oa; O I Oa; ( B2*(Mbar\Cbar)-B1 ) (-B2/Mbar) Ia ];
    AA = [ -Cbar I Oa; -Kbar O qhat*Ca; B0 Oa' Aa ];
    Aae = EE\AA;
    
    % Aeroelastic eigenvalues
    sn  = eig(Aae);
    
    nXae = size(Aae,1); % number of aeroelastic states
    if nU > 1 % reordering eigenvalues
        sntmp = sn;
        clear sn;
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
    
end


% -------------------------------------------------------------------------
% Searching flutter points
% -------------------------------------------------------------------------
iV = find( (UU/ft2m<290).*(UU/ft2m>150)); % selecting velocity < 300 ft/s

instabmodes = find(real(root(end,:)>0)); % instable modes (s.t. real(eigval) > 0)

% initialization
V_flutter    = zeros( 1, 2*ndof+nXa );
f_flutter    = zeros( 1, 2*ndof+nXa );
root_flutter = zeros( 1, 2*ndof+nXa );
flutterlabel = cell ( 1, 2*ndof+nXa );

for idof = instabmodes;
    
    if sum( g_vec(iV(1),idof)==g_vec(iV,idof) ) == length(UU(iV)) ...  % constant damping
            || sum( isnan(g_vec(iV,idof)) ) == length(UU(iV)) ... % isNaN
            || sum( g_vec(iV(1),idof)==g_vec(iV,idof) ) + sum( isnan(g_vec(iV,idof)) ) == length(UU(iV))...  % constant damping
            || sum( 2 == abs(g_vec(iV,idof)) ) == length(UU(iV))  % damping = -+2; i.e. zet = 1/2g = -+1
        V_flutter(idof) = NaN;
    else
        V_flutter(idof) = interp1(g_vec(iV,idof),UU(iV),0);
    end
    
    f_flutter(idof)    = interp1( UU(iV), om_vec(iV,idof)/(2*pi), V_flutter(idof) );
    root_flutter(idof) = interp1( UU(iV), root(iV,idof), V_flutter(idof) );
    
    switch idof
        case{1,2}, flutterlabel{idof} = 'flutter';
        case{6,7}, flutterlabel{idof} = 'dynamic divergence';
        otherwise, flutterlabel{idof} = 'instability';
    end
    
end

% -------------------------------------------------------------------------
% Comparison HH(jk) e HH(sig+jk)
% -------------------------------------------------------------------------

clear QQ k_vec

kvec = 0:0.01:3;

nk = length(kvec);
QQ   = zeros(2,2,nk);
QQ_k = zeros(2,2,nk);
QQapprox_p = zeros(2,2,nk);
QQapprox_k = zeros(2,2,nk);

for ii = 1:length(kvec)

    xibar = -0.5;
    p = (xibar + 1i)*kvec(ii);
    
    kbar = 1i*imag(p);
    
    % ---------------------------------------------------------------------
    % Aerodynamic Transfer Matrix QQ: F=qdyn*QQ*[h;alfa]
    % ---------------------------------------------------------------------
    % [-L;M] = (s^2*Mnc + s*Bnc + Knc + Ck*R*(s*S2 + S1) )*[h;alfa]
    signL = -1;
    signM = 1;
    Mnc = pi*[ signL*[1 -a]; signM*[a -(1/8+a^2)] ];
    Bnc = pi*[ signL*[0  1]; signM*[0   -(0.5-a)] ];
    Knc = [signL*[0 0]; signM*[0 0] ];
    R   = 2*pi*[ signL*1; (0.5+a) ];
    S1  = [ 0 1 ];
    S2  = [ 1 (0.5-a) ];
    
    Ck = GenTheod(p);
    QQ(:,:,ii) = ( p^2*Mnc + p*Bnc + Knc + Ck*R*(p*S2 + S1) );
    
    Ck_k = GenTheod(kbar);
    QQ_k(:,:,ii) = ( kbar^2*Mnc + kbar*Bnc + Knc + Ck_k*R*(kbar*S2 + S1) );
    
    % ---------------------------------------------------------------------
    % Aerodynamic Transfer Matrix Approximation
    % ---------------------------------------------------------------------
    BBs = Bid{1} + p*Bid{2} + p^2*Bid{3};
    EEs = Eid{1} + p*Eid{2} + p^2*Eid{3};
    QQapprox_p(:,:,ii) = Cid*( (p*eye(nXa) - Aid)\ BBs ) + EEs;
    
    BBs = Bid{1} + kbar*Bid{2} + kbar^2*Bid{3};
    EEs = Eid{1} + kbar*Eid{2} + kbar^2*Eid{3};
    QQapprox_k(:,:,ii) = Cid*( (kbar*eye(nXa) - Aid)\ BBs ) + EEs;
    
end



%%
% =========================================================================
%          PLOT OF THE AERODYNAMIC TRANSFER MATRIX IDENTIFICATION 
% =========================================================================
linespec{1} = '-'; linecol{1} = 'b';
linespec{2} = 'o'; linecol{2} = 'r';
labelleg{1} = 'Analytical H(jk)';
labelleg{2} = 'Identified H(jk)';
for sel = 1:nHH
    
    Hplot  = Heval{sel};
    kkplot = kkeval{sel};
    
    figure(1)
    % ---------------------------------------------------------------------
    subplot(2,2,1)
    plot(kkplot,squeeze(real(Hplot(1,1,:))),linespec{sel},kkplot,squeeze(imag(Hplot(1,1,:))),linespec{sel})
    xlabel('Reduced Freq.'), ylabel('H_{hh}'), legend('Re','Im','Location','Best'), grid on, hold on
    xlim([0 0.5])
    subplot(2,2,2)
    plot(kkplot,squeeze(real(Hplot(1,2,:))),linespec{sel},kkplot,squeeze(imag(Hplot(1,2,:))),linespec{sel})
    xlabel('Reduced Freq.'), ylabel('H_{h\theta}'), legend('Re','Im','Location','Best'), grid on, hold on
    xlim([0 0.5])
    subplot(2,2,3)
    plot(kkplot,squeeze(real(Hplot(2,1,:))),linespec{sel},kkplot,squeeze(imag(Hplot(2,1,:))),linespec{sel})
    xlabel('Reduced Freq.'), ylabel('H_{\thetah}'), legend('Re','Im','Location','Best'), grid on, hold on
    xlim([0 0.5])
    subplot(2,2,4)
    plot(kkplot,squeeze(real(Hplot(2,2,:))),linespec{sel},kkplot,squeeze(imag(Hplot(2,2,:))),linespec{sel})
    xlabel('Reduced Freq.'), ylabel('H_{\theta\theta}'), legend('Re','Im','Location','Best'), grid on, hold on
    xlim([0 0.5])
    
    figure(2)
    % ---------------------------------------------------------------------
    subplot(2,2,1)
    plot(squeeze(real(Hplot(1,1,:))),squeeze(imag(Hplot(1,1,:))),linespec{sel},'color',linecol{sel},'linewidth',1.2,'MarkerSize',5)
    xlabel('Re','FontSize',12,'FontWeight','Bold'),
    ylabel('Im','FontSize',12,'FontWeight','Bold'),
    title('H_{hh}','FontSize',14,'FontWeight','Bold'), grid off, hold on, box on
    if sel == nHH, legend(labelleg{1},labelleg{2},'Location','NorthEast'), end
    legend boxoff
    subplot(2,2,2)
    plot(squeeze(real(Hplot(1,2,:))),squeeze(imag(Hplot(1,2,:))),linespec{sel},'color',linecol{sel},'linewidth',1.2,'MarkerSize',5)
    xlabel('Re','FontSize',12,'FontWeight','Bold'),
    ylabel('Im','FontSize',12,'FontWeight','Bold'),
    title('H_{h\theta}','FontSize',14,'FontWeight','Bold'), grid off, hold on, box on
    subplot(2,2,3)
    plot(squeeze(real(Hplot(2,1,:))),squeeze(imag(Hplot(2,1,:))),linespec{sel},'color',linecol{sel},'linewidth',1.2,'MarkerSize',5)
    xlabel('Re','FontSize',12,'FontWeight','Bold'),
    ylabel('Im','FontSize',12,'FontWeight','Bold'),
    title('H_{\thetah}','FontSize',14,'FontWeight','Bold'), grid off, hold on, box on
    subplot(2,2,4)
    plot(squeeze(real(Hplot(2,2,:))),squeeze(imag(Hplot(2,2,:))),linespec{sel},'color',linecol{sel},'linewidth',1.2,'MarkerSize',5)
    xlabel('Re','FontSize',12,'FontWeight','Bold'),
    ylabel('Im','FontSize',12,'FontWeight','Bold'),
    title('H_{\theta\theta}','FontSize',14,'FontWeight','Bold'), grid off, hold on, box on

end


figure(3)
% -------------------------------------------------------------------------
hold on

plot(real(sn_struct),imag(sn_struct),'kx','MarkerSize',10,'MarkerFaceColor','k')

% for legend purpose    
plot(real(root(1,1))+1000,imag(root(1,1))+1000,'ok','linewidth',1.,'MarkerSize',5,'MarkerFaceColor','k')
hleg = legend('coupled structural eigenvalues','approx. state-space model',1);
set(hleg,'FontSize',12);
set(hleg, 'Box', 'on');
set(hleg, 'EdgeColor','w');
set(hleg, 'Color', 'w');

for i = 1:nXae
    plot(real(root(:,i)),imag(root(:,i)),'ko','MarkerSize',2,'MarkerFaceColor','k')
end
for idof = instabmodes;
    Vf = V_flutter(idof)/ft2m;
    root_f = root_flutter(idof);
    if imag(root_f) > 0
        annotation('textarrow',[1 0]./20+0.525,[imag(root_f)+sign(imag(root_f))*3 imag(root_f)]./43+0.235,...
            'string', strcat(num2str(round(Vf*10)/10),',{ }',flutterlabel{idof}),...
            'FontSize',14,'FontWeight','Bold','HorizontalAlignment','center')
        plot(real(root_f),imag(root_f),'ko','MarkerFaceColor',[1 1 1]/2,'MarkerSize',6)
    end
end
axes1 = gca;
set(axes1,'FontSize',14);
xlabel('Re','FontSize',16,'FontWeight','Bold'),
ylabel('Im','FontSize',16,'FontWeight','Bold'),
grid on, box on
xlim([-10 10]) %([-20 20])
ylim([-5 30]) %([-5 30])
title('Root locus','FontSize',14,'FontWeight','Bold'),
% -------------------------------------------------------------------------


figure(4)
% -------------------------------------------------------------------------
hold on
for i = 1:nXae
    plot(UU/ft2m,g_vec(:,i),'ko','MarkerSize',2,'MarkerFaceColor','k')
end
for idof = instabmodes; %1:nXae
    Vf = V_flutter(idof)/ft2m;
    plot(Vf,0,'ko','MarkerFaceColor',[1 1 1]/2,'MarkerSize',6)
    switch flutterlabel{idof}
        case 'flutter'
            text(Vf,0.1,num2str(round(Vf*10)/10),...
                'FontSize',16,'FontWeight','Bold','HorizontalAlignment','center', 'BackgroundColor','w');
        case 'dynamic divergence'
            text(Vf-20,0.1,num2str(round(Vf*10)/10),...
                'FontSize',16,'FontWeight','Bold','HorizontalAlignment','center', 'BackgroundColor','w');
    end
end
axes1 = gca;
set(axes1,'FontSize',14);
xlabel('U (ft/s)','FontSize',14,'FontWeight','Bold')
ylabel('Damping g','FontSize',14,'FontWeight','Bold')
grid on, box on
xlim([0 350])
ylim([-1 1])
title('Flutter damping','FontSize',14,'FontWeight','Bold'),
% -------------------------------------------------------------------------

figure(5)
% -------------------------------------------------------------------------
hold on
plot(UU/ft2m,om_vec/(2*pi),'ko','MarkerSize',2,'MarkerFaceColor','k')
for idof = instabmodes; %1:nXae
    Vf     = V_flutter(idof)/ft2m;
    freq_f = f_flutter(idof);
    text(Vf,freq_f+0.12,num2str(round(freq_f*1000)/1000),...
        'FontSize',14,'FontWeight','Bold','HorizontalAlignment','center', 'BackgroundColor','w');
    plot(Vf,freq_f,'ko','MarkerFaceColor',[1 1 1]/2,'MarkerSize',6)
end
xlabel('U (ft/s)','FontSize',14,'FontWeight','Bold'),
ylabel('Frequency (Hz)','FontSize',14,'FontWeight','Bold')
grid on, box on
xlim([0 300])
ylim([0 4])
title('Flutter Frequency','FontSize',14,'FontWeight','Bold'),
% -------------------------------------------------------------------------


figure(6)
% -------------------------------------------------------------------------
kplot = [0 0.05 0.2 0.5 0.7 1 1.3 1.6 2 2.5 3 0.02 0.1 0.3];
nkp = length(kplot);

% initialization
QQ_coarse = zeros(2,2,nkp);
HHorig_coarse = zeros(2,2,nkp);
QQapprox_p_coarse = zeros(2,2,nkp);
HHapprox_coarse = zeros(2,2,nkp);

for k = 1:nkp
    
    % initialization
    GAFp = zeros(2,2);
    GAFk = zeros(2,2);    
    GAFidp = zeros(2,2);
    GAFidk = zeros(2,2);
    
    % interpolation for plotting purposes
    for row = 1:2
        GAFp(row,:)   = interp1(kvec.',squeeze(QQ(row,:,:)).',kplot(k),'spline');
        GAFk(row,:)   = interp1(kkpl.',squeeze(HHorig(row,:,:)).',kplot(k),'spline');
        GAFidp(row,:) = interp1(kvec.',squeeze(QQapprox_p(row,:,:)).',kplot(k),'spline');
        GAFidk(row,:) = interp1(kkapprox.',squeeze(HHapprox(row,:,:)).',kplot(k),'spline');
    end
    
    % storage of the results for plotting
    QQ_coarse(:,:,k)         = GAFp;
    HHorig_coarse(:,:,k)     = GAFk;
    QQapprox_p_coarse(:,:,k) = GAFidp;
    HHapprox_coarse(:,:,k)   = GAFidk;
end

nQQ = 4;

QQeval{1} = QQ_coarse;
QQeval{2} = HHorig_coarse;
QQeval{3} = HHapprox;
QQeval{4} = QQapprox_p;

labelQQ{1} = 'Analytical H(p)';
labelQQ{2} = 'Analytical H(k)';
labelQQ{3} = 'Identified H(k)';
labelQQ{4} = 'Identified H(p)';

clear linespec
linespec{1} = 's';
linespec{2} = 'o';
linespec{3} = '--';
linespec{4} = '-';
colorspec{1} = 'k';
colorspec{2} = 'k';
colorspec{3} = 'k';
colorspec{4} = 'k';
fontaxis = 10;
fontitle = 12;
markersize{1} = 5;
markersize{2} = 5;
markersize{3} = 4;
markersize{4} = 5;

for sel = 1:nQQ
    
    Hplot = QQeval{sel};
    
    subplot(2,3,1)
    plot(squeeze(real(Hplot(1,1,:))),squeeze(imag(Hplot(1,1,:))),linespec{sel},'color',colorspec{sel},'linewidth',0.5,'MarkerSize',markersize{sel})
    xlabel('Re','FontSize',fontaxis,'FontWeight','normal'),
    ylabel('Im','FontSize',fontaxis,'FontWeight','normal'),
    title('H_{hh}','FontSize',fontitle,'FontWeight','Bold'), grid off, hold on, box on
    subplot(2,3,2)
    plot(squeeze(real(Hplot(1,2,:))),squeeze(imag(Hplot(1,2,:))),linespec{sel},'color',colorspec{sel},'linewidth',0.5,'MarkerSize',markersize{sel})
    xlabel('Re','FontSize',fontaxis,'FontWeight','normal'),
    ylabel('Im','FontSize',fontaxis,'FontWeight','normal'),
    title('H_{h\theta}','FontSize',fontitle,'FontWeight','Bold'), grid off, hold on, box on
    subplot(2,3,4)
    plot(squeeze(real(Hplot(2,1,:))),squeeze(imag(Hplot(2,1,:))),linespec{sel},'color',colorspec{sel},'linewidth',0.5,'MarkerSize',markersize{sel})
    xlabel('Re','FontSize',fontaxis,'FontWeight','normal'),
    ylabel('Im','FontSize',fontaxis,'FontWeight','normal'),
    title('H_{\thetah}','FontSize',fontitle,'FontWeight','Bold'), grid off, hold on, box on
    subplot(2,3,5)
    plot(squeeze(real(Hplot(2,2,:))),squeeze(imag(Hplot(2,2,:))),linespec{sel},'color',colorspec{sel},'linewidth',0.5,'MarkerSize',markersize{sel})
    xlabel('Re','FontSize',fontaxis,'FontWeight','normal'),
    ylabel('Im','FontSize',fontaxis,'FontWeight','normal'),
    title('H_{\theta\theta}','FontSize',fontitle,'FontWeight','Bold'), grid off, hold on, box on
    
    subplot(2,3,3)
    plot(squeeze(real(Hplot(1,1,:))),squeeze(imag(Hplot(1,1,:))),linespec{sel},'color',colorspec{sel},'linewidth',0.5,'MarkerSize',markersize{sel})
    xlabel('Re','FontSize',fontaxis,'FontWeight','normal'),
    ylabel('Im','FontSize',fontaxis,'FontWeight','normal'),
    title('H_{hh} zoomed','FontSize',fontitle,'FontWeight','Bold'), grid off, hold on, box on
    xlim([-0.2 0.8])
    ylim([-3 0.5])
    subplot(2,3,6)
    plot(squeeze(real(Hplot(2,2,:))),squeeze(imag(Hplot(2,2,:))),linespec{sel},'color',colorspec{sel},'linewidth',0.5,'MarkerSize',markersize{sel})
    xlabel('Re','FontSize',fontaxis,'FontWeight','normal'),
    ylabel('Im','FontSize',fontaxis,'FontWeight','normal'),
    title('H_{\theta\theta} zoomed','FontSize',fontitle,'FontWeight','Bold'), grid off, hold on, box on
    xlim([1.2 2.4001])
    ylim([-2. 0])
    
    if sel == 2
        for n = 1:length(kplot)
            subplot(2,3,1), text(squeeze(real(Hplot(1,1,n))),squeeze(imag(Hplot(1,1,n))),num2str(kplot(n)),'FontSize',8,'color','r');
            subplot(2,3,3), text(squeeze(real(Hplot(1,1,n))),squeeze(imag(Hplot(1,1,n))),num2str(kplot(n)),'FontSize',8,'color','r');
            subplot(2,3,2), text(squeeze(real(Hplot(1,2,n))),squeeze(imag(Hplot(1,2,n))),num2str(kplot(n)),'FontSize',8,'color','r');
            subplot(2,3,4), text(squeeze(real(Hplot(2,1,n))),squeeze(imag(Hplot(2,1,n))),num2str(kplot(n)),'FontSize',8,'color','r');
            subplot(2,3,5), text(squeeze(real(Hplot(2,2,n))),squeeze(imag(Hplot(2,2,n))),num2str(kplot(n)),'FontSize',8,'color','r');
            subplot(2,3,6), text(squeeze(real(Hplot(2,2,n))),squeeze(imag(Hplot(2,2,n))),num2str(kplot(n)),'FontSize',8,'color','r');
        end
    end
    
end
subplot(2,3,1)
hleg1 = legend(labelQQ{1},labelQQ{2},labelQQ{3},labelQQ{4},2);
set(hleg1,'Interpreter','none')
set(hleg1,'FontSize',10)
legend boxoff
orient landscape

