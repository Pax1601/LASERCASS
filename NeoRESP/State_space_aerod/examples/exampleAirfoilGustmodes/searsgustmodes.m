% =========================================================================
%           SEARS FUNCTION OF AN AIRFOIL IN INCOMPRESSIBLE FLOW
% =========================================================================
%
% Description:
% aerodynamic transfer matrix related to a gust (Sears function) of an 
% airfoil in incompressible flow, calculated with the gust modes approach. 
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
% - Bisplinghoff, R. L., Ashley, H., and Halfman, R. L., "Aeroelasticity",
%   Dover, 1996. (p. 277, 281-293)
% =========================================================================

close all
clear all
clc

%%
% =========================================================================
%                                  Data
% =========================================================================
% NOTE: consider decreasing the integration step "dx", by increasing nX, to 
% improve the accuracy of the results 

b    = 1;       % half chord, m
V    = 1;       % airspeed, m/s
rho0 = 1;       % density, kg/m^3
w0   = 1;       % vertical gust intensity, m/s
qdyn = 0.5*rho0*V^2;

nX = 101;
dx = (2*b)/(nX-1); % integration step
X  = -b:dx:b; 

kk  = 1e-9:.01:10; % gust reduced frequency
nkk = length(kk);
om  = kk*V/b;
lambda = 2*pi*b./kk;

% -------------------------------------------------------------------------
% Theodorsen's Function
% -------------------------------------------------------------------------
J0 = besselj(0,kk);
J1 = besselj(1,kk);
Y0 = bessely(0,kk);
Y1 = bessely(1,kk);
H0 = J0 - 1i*Y0;
H1 = J1 - 1i*Y1;
Ctheod = H1./(H1 + 1i*H0);

% -------------------------------------------------------------------------
% Sears' Function
% -------------------------------------------------------------------------
% Response relative to mid-airfoil
Sears  = Ctheod.*(J0 - 1i*J1) + 1i*J1;

% Response relative to leading edge (LE)
SearsLE = Sears.*exp(-1i*kk);

% Aerodynamic tranfer matrix Hag s.t F = qdyn*Hag*(wg/V)
Hag_Sears = Sears*4*pi*b/V;


%%
% =========================================================================
%         Sears function calculated using the pressure integration
%         from "Aeroelasticity", Bisplinghoff (p. 277, eq. 5-342)
% =========================================================================
% NOTE: consider decreasing the integration step "dcsi" by increasing nCSI
% to improve the accuracy of the results 

nCSI = 101;
dcsi = (2*b)/(nCSI-1);  % integration step
csi  = -b:dcsi:b; 

% Calculate Lambda 1, function of (x,csi) and of the second integrand
% -------------------------------------------------------------------------
Lambda1    = zeros(nX,nCSI);
dint2_xcsi = zeros(nX,nCSI);
for i = 1:nX
    
    x  = X(i);
    
    Lambda1xi = (1/2)*log( ( 1-x*csi + (1-x^2)^.5*(1-csi.^2).^.5 )./( 1-x*csi - (1-x^2)^.5*(1-csi.^2).^.5 ) );
    
    % to avoid singularity points
    i_inf   = isinf(Lambda1xi); 
    inf_pos = find(i_inf==1); 
    
    % mean of the values in the points just before and just after the singularity
    Lambda1xi(inf_pos) = 0.5*( Lambda1xi(inf_pos+1) + Lambda1xi(inf_pos-1) ); 
    
    Lambda1(i,:) = Lambda1xi; 
    
    dint2_xcsi(i,:) = ((1-x)/(1+x))^.5 * ((1+csi)./(1-csi)).^.5./((x-csi)./b);
    
end

% Aerodynamic tranfer matrix Hag calculation
% -------------------------------------------------------------------------
Hag = zeros(1,nkk);
for n = 1:nkk
    
    k_g   = kk(n);
    om_g  = k_g*V/b;
    
    wa  = - exp(1i*om_g*(-csi./V));
    
    % Calculate first integral, function only of csi 
    dint1 = wa.*( (1+csi)./(1-csi) ).^.5;
    dint1(isinf(dint1)) = 0;
    int1 = sum(dint1(1:end-1)+dint1(2:end))*dcsi/2;
    
    % Calculate second integral
    Aero  = zeros(1,nX);
    for i = 1:nX
        
        x  = X(i);
        
        dint2 = wa.*( dint2_xcsi(i,:) - 1i*k_g*Lambda1(i,:) );
        
        % to avoid nan and inf
        dint2(isinf(dint2)) = 0;
        dint2(isnan(dint2)) = 0;
        
        int2 = sum( dint2(1:end-1) + dint2(2:end) )*dcsi/2;

        Aero(i)  = 4/pi*( (1-Ctheod(n))*( (1-x)/(1+x) )^.5*int1 + int2);
        
    end
    % to avoid nan and inf
    Aero(isnan(Aero)) = 0;
    Aero(isinf(Aero)) = 0;
    
    Hag(n) = sum( Aero(1:end-1) + Aero(2:end) )*dx/2;
    
end

Lift = qdyn*Hag*(w0/V);
adim = real(Lift(1)); % in order to have Hag(0) = 1, since using a large integration step may result in having Hag(0) ~= 1


%%
% =========================================================================
%             Sears function calculated using the gust modes
% =========================================================================
Nmodes = 4;
Nelem = Nmodes-1;
xgustnodes = -b : 2*b/(Nmodes-1) : b;

% Linear Bases
% -------------------------------------------------------------------------
[Nlin,~] = GustModesShapeFunctions(csi,xgustnodes,'lin');

% int(N^T*N)dx
Aq = zeros(Nmodes,Nmodes);
ak = zeros(Nmodes,Nmodes);
for k = 1:Nelem
    [~,k0x] = min(abs(csi-xgustnodes(k)));
    [~,k1x] = min(abs(csi-xgustnodes(k+1)));
  
    for i = k:k+1
        dl = dcsi; % NOTE: dlloc in "GustModesShapeFunctions" is constant on the element 
        
        for j = k:k+1
            NN = Nlin(k0x:k1x,i).*Nlin(k0x:k1x,j);
            ak(i,j) = sum( (NN(2:end) + NN(1:end-1)) *dl/2 );
            Aq(i,j) = Aq(i,j) + ak(i,j);
        end
    end
end

% integration over frequency
qgls = zeros(Nmodes,nkk);
for ik = 1:nkk
    
    k_g  = kk(ik);
    om_g = k_g*V/b;
    
    bf = zeros(Nmodes,1);
    wa = - exp(1i*om_g*(-csi'./V));
    wg = - wa;

    fk = zeros(1,Nmodes);
    for k = 1:Nelem
        
        [~,k0x] = min(abs(csi-xgustnodes(k)));
        [~,k1x] = min(abs(csi-xgustnodes(k+1)));
        
        for i = k:k+1
            
            dl = dcsi; % NOTE: dlloc in "GustModesShapeFunctions" is constant on the element 
            
            Nf = Nlin(k0x:k1x,i).*wg(k0x:k1x,1);
            fk(i) = sum( (Nf(2:end) + Nf(1:end-1)) *dl/2 );
            bf(i) = bf(i) + fk(i);
            
        end %end i for
        
    end % end k for
    
    qgls(:,ik) = ((Aq'*Aq)\Aq')*bf;
    
end %end it for

% initialization
Hag_modes   = zeros(nkk,Nmodes);
Hag_recover = zeros(nkk,1);
Lift_modes  = zeros(nkk,1);

for n = 1:nkk
    
    k_g   = kk(n);
    om_g  = k_g*V/b;
    
    for k = 1:Nmodes
        
        wa  = - Nlin(:,k).';
        
        % Calculate first integral function only of csi
        dint1 = wa.*( (1+csi)./(1-csi) ).^.5;
        dint1(isinf(dint1)) = 0;
        dint1(isnan(dint1)) = 0;
        int1 = sum(dint1(1:end-1)+dint1(2:end))*dcsi/2;
        
        % Calculate second integral
        Aero  = zeros(1,nX);
        for i = 1:nX
            
            x  = X(i);

            dint2 = wa.*( dint2_xcsi(i,:) - 1i*k_g*Lambda1(i,:) );
            dint2(isinf(dint2)) = 0;
            dint2(isnan(dint2)) = 0;
            
            int2 = sum(dint2(1:end-1)+dint2(2:end))*dcsi/2;
            Aero(i)  = 4/pi*( (1-Ctheod(n))*( (1-x)/(1+x) )^.5*int1 + int2);
            
        end
        Aero(isnan(Aero)) = 0;
        Aero(isinf(Aero)) = 0;
        Hag_modes(n,k) = sum( Aero(1:end-1) + Aero(2:end) )*dx/2;
        
    end
    
    Hag_recover(n) = Hag_modes(n,:)*qgls(:,n);
    Lift_modes(n)  = qdyn*Hag_modes(n,:)*(w0/V)*qgls(:,n);
    
end


%%
% =========================================================================
%                                  Plot
% =========================================================================

figure
% -------------------------------------------------------------------------
subplot(2,2,1)
plot(kk,real(Hag_Sears), kk,real(Hag), kk,real(Hag_recover),'--')
xlabel('reduced frequency','Fontsize',14), ylabel('Real','Fontsize',14)
title('H_{ag}','Fontsize',14,'Fontweight','bold')
set(gca,'Fontsize',14)
legend('Hag using Sears function S(k)','Hag from pressure integration',...
    strcat('Hag using {}',num2str(Nmodes),' gust modes') )
grid on, box on

subplot(2,2,2)
plot(kk,imag(Hag_Sears), kk,imag(Hag), kk,imag(Hag_recover),'--')
xlabel('reduced frequency','Fontsize',14), ylabel('Imag','Fontsize',14)
title('H_{ag}','Fontsize',14,'Fontweight','bold')
set(gca,'Fontsize',14)
grid on, box on

subplot(2,2,3:4)
plot(real(Sears),imag(Sears),...
     real(Lift/adim),imag(Lift/adim),...
     real(Lift_modes/adim),imag(Lift_modes/adim),'--')
xlabel('Real','Fontsize',14), ylabel('Imag','Fontsize',14)
set(gca,'Fontsize',14)
title('Sears function','Fontsize',14,'Fontweight','bold')
grid on, box on
legend('Sears function S(k)','S(k) from pressure integration',...
    strcat('S(k) using {}',num2str(Nmodes),' gust modes'))

orient landscape


figure
% -------------------------------------------------------------------------
plot(real(Sears),imag(Sears), real(SearsLE),imag(SearsLE),...
     real(Lift_modes/adim),imag(Lift_modes/adim),'--')
xlabel('Real','Fontsize',14), ylabel('Imag','Fontsize',14)
title('Sears function S(k)','Fontsize',14,'Fontweight','bold')
set(gca,'Fontsize',14)
grid on, box on
legend('S(k) w.r.t to mid-chord','S(k) w.r.t to leading edge',... %'S(k) from pressure integration {}',...
    strcat('S(k) using {}',num2str(Nmodes),' gust modes'))
xlim([-0.4 1])

% place points
kg_eval = [ 0 0.02 0.1 0.2 0.5 1 2 4 5 8 10 ];
kgLE_eval = [ 0.02 0.1 0.2 0.5 1 2  5  10 ];
Sk   = interp1(kk,Sears,kg_eval,'linear','extrap'); 
SkLE = interp1(kk,SearsLE,kgLE_eval,'linear','extrap');
hold on
plot(real(Sk),imag(Sk),'.',real(SkLE),imag(SkLE),'.','MarkerSize',16)
% wrote values
for k = 1:length(kg_eval)
    text(real(Sk(k)),imag(Sk(k)),strcat('k_g= ',num2str(kg_eval(k))),'BackgroundColor',[1 1 1],'Margin',0.1);
end
for k = 1:length(kgLE_eval) 
    if k == 1, % do nothing
    else text(real(SkLE(k)),imag(SkLE(k)),strcat('k_g= ',num2str(kgLE_eval(k))),'BackgroundColor',[1 1 1],'Margin',0.1);
    end
end


