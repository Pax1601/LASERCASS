function [E, A] = ss_rigid(M, VREF, THETA0)
%
g = 9.81;
V = zeros(1,3);
V(3) = VREF;
m = M(1,1);
S = M(4:6,1:3);
J = M(4:6,4:6);
phi   = THETA0(1);
theta = THETA0(2);
psi   = THETA0(3);
%
O3 = zeros(3,3);
%G = (m*g).*crossm([-sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)]);
%
%invC = [1, sin(phi)*tan(theta), cos(phi)*tan(theta); 
%        0, cos(phi),            -sin(phi); 
%        0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
invC = eye(3);
%
E = blkdiag(M,eye(3));
%
G =  [0                   , -cos(theta)          ,  0;
      cos(theta)*cos(phi), -sin(theta)*sin(phi),  0;
      -cos(theta)*sin(phi),-sin(theta)*cos(phi),  0];
%
Rtheta = [1, sin(phi)*tan(theta), cos(phi)*tan(theta); 
          0, cos(phi),            -sin(phi); 
          0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
%
%A = -[m.*crossm(OMEGA0),                   -m.*crossm(V) + crossm(OMEGA0)*S' - crossm(S'*OMEGA0),              -G; 
%     crossm(OMEGA0)*S-crossm(S'*OMEGA0),   -crossm(S*V) + crossm(V)*S' + crossm(OMEGA0)*J -crossm(J*OMEGA0),  O3; 
%     O3,                                    invC,                                                                 O3];
A = -[O3,  -m.*crossm(V),              -(m*g).*G*Rtheta; 
     O3,   -crossm(S*V) + crossm(V)*S' -g.*S*G*Rtheta; 
     O3,   invC,                        O3];
