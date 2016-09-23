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
%     091014      1.3.8   Travaglini       Creation 
%   
%*******************************************************************************
%
% function Geo_Fus_ShapeT
%
%   DESCRIPTION: Define fudselage geometry
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                aircraft       struct     structure aircraft, therefore
%                                          with geometry description
% 
%                N              vect       vector with number of desidered
%                                          points along x (nose aft fore
%                                          tail) and on circle
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                OUT            struct     point coordinates that define
%                                          fuselage geometry
%    REFERENCES:
%
%**************************************************************************
%*****
function [OUT] = Geo_Fus_Shape(aircraft,N)

% --> OUT.scx x point
% --> OUT.scy y point
% --> OUT.scz z point

Fus = aircraft.fuselage;
% extract data

ome = [Fus.omega_nose, Fus.omega_tail]*pi/180;
phi = [Fus.phi_nose, Fus.phi_tail]*pi/180;

dv = [Fus.Forefuse_X_sect_vertical_diameter, Fus.Aftfuse_X_sect_vertical_diameter];
xi = [Fus.Forefuse_Xs_distortion_coefficient, Fus.Aftfuse_Xs_distortion_coefficient];
xmax = [Fus.Nose_length, Fus.Tail_length];
f = Fus.fraction_fore;
shift = Fus.shift_fore;
eps = [ Fus.epsilon_nose, Fus.epsilon_tail];
L = Fus.Total_fuselage_length;
A = [Fus.a0_fore, Fus.a1_fore, Fus.b1_fore; Fus.a0_aft, Fus.a1_aft, Fus.b1_aft];

% define costant values needed to describe fuselage cross section
beta = 0.54 + 0.1*tan(ome+phi);
alpha = dv.^(1-beta)./(2*eps.^beta);

% compute lengths of aft and fore
l = [f*(L-sum(xmax)) , (1-f)*(L-sum(xmax))];

% compute Nose geometry
x_nose = linspace(0,xmax(1),N(1));
z1_nose = alpha(1)*x_nose.^beta(1)-(eps(1).*dv(1)-x_nose)*tan(phi(1)) ;
z2_nose = -alpha(1)*x_nose.^beta(1)-(eps(1).*dv(1)-x_nose)*tan(phi(1));

% figure
% plot(x_nose,z1_nose,'r')
% hold on
% plot(x_nose,z2_nose,'b')
% grid on

theta = linspace(0,2*pi,N(end));
r_nose = A(1,1) + A(1,2)*sin(theta) + A(1,3)*cos(2*theta);

Y_nose = zeros(N(end),N(1));
Z_nose = Y_nose;

for i = 1 : N(1)
   Y_nose(:,i) = -cos(theta).*r_nose*(z1_nose(i)-z2_nose(i))/dv(1);
   Z_nose(:,i) = (r_nose.*sin(theta)-(xi(1)-0.5)*dv(1))*(z1_nose(i)-z2_nose(i))/dv(1)-(eps(1).*dv(1)-x_nose(i))*tan(phi(1)) - (xi(2)-0.5)*dv(2) + shift*dv(2);
end

% tratto fore
x_fore = linspace(xmax(1),xmax(1)+l(1),N(2));
Y_fore = zeros(N(end),N(2));
Z_fore = Y_fore;

for i = 1 : N(2)
    r = (A(1,1)+(A(2,1)-A(1,1))/l(1)*(x_fore(i)-x_fore(1)) + (A(1,2)+(A(2,2)-A(1,2))/l(1)*(x_fore(i)-x_fore(1)))*sin(theta) + (A(1,3)+(A(2,3)-A(1,3))/l(1)*(x_fore(i)-x_fore(1)))*cos(2*theta));
    dvloc = dv(1)+(dv(2)-dv(1))/l(1)*(x_fore(i)-x_fore(1));
    Y_fore(:,i) = -cos(theta).*r;
    Z_fore(:,i) = (r.*sin(theta)-((xi(1)+(xi(2)-xi(1))/l(1)*(x_fore(i)-x_fore(1)))-0.5)*dvloc)+(shift*(1-1/l(1)*(x_fore(i)-x_fore(1))) )*dv(2)- (xi(2)-0.5)*dv(2);
end

% tratto aft

x_aft = linspace(xmax(1)+l(1),xmax(1)+sum(l),N(3));
Y_aft = zeros(N(end),N(3));
Z_aft = Y_aft;

for i = 1 : N(3)
%     r = (A(1,1)+(A(2,1)-A(1,1))/l(1)*(x_fore(i)-x_fore(1)) + (A(1,2)+(A(2,2)-A(1,2))/l(1)*(x_fore(i)-x_fore(1)))*sin(theta) + (A(1,3)+(A(2,3)-A(1,3))/l(1)*(x_fore(i)-x_fore(1)))*cos(2*theta));
%     dvloc = dv(1)+(dv(2)-dv(1))/l(1)*(x_fore(i)-x_fore(1));
    Y_aft(:,i) = -cos(theta).*(A(2,1) + A(2,2)*sin(theta) + A(2,3)*cos(2*theta));
    Z_aft(:,i) = (A(2,1) + A(2,2)*sin(theta) + A(2,3)*cos(2*theta)).*sin(theta)-(xi(2)-0.5)*dv(2);
end


% cono di coda
x_tail = linspace(x_aft(end),L,N(4));

z1_tail = +alpha(2)*(L-x_tail).^beta(2)+(eps(2).*dv(2)-(L-x_tail))*tan(phi(2)) ;
z2_tail = -alpha(2)*(L-x_tail).^beta(2)+(eps(2).*dv(2)-(L-x_tail))*tan(phi(2)) ;

% figure
% plot(x_tail,z1_tail,'r')
% hold on
% plot(x_tail,z2_tail,'b')
% grid on

r_tail = A(2,1) + A(2,2)*sin(theta) + A(2,3)*cos(2*theta);

Y_tail = zeros(N(end),N(4));
Z_tail = Y_tail;

for i = 1 : N(4)
   Y_tail(:,i) = -cos(theta).*r_tail*(z1_tail(i)-z2_tail(i))/dv(2);
   Z_tail(:,i) = (r_tail.*sin(theta)-(xi(2)-0.5)*dv(2))*(z1_tail(i)-z2_tail(i))/dv(2)+(eps(2).*dv(2)-(L-x_tail(i)))*tan(phi(2));
end


OUT.scx = meshgrid([x_nose,x_fore(2:end),x_aft(2:end),x_tail(2:end)],1:length(theta));
OUT.scy = [Y_nose,Y_fore(:,2:end),Y_aft(:,2:end),Y_tail(:,2:end)];
OUT.scz = [Z_nose,Z_fore(:,2:end),Z_aft(:,2:end),Z_tail(:,2:end)];