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
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************
%
% This function is extracted from Tornado by T. Melin and A. Berard, KTH
% Changes are required to adapt Tornado geo struct to Doublet Lattice database 
%

function [lattice] = dlm_lattice_setup(fid, geo, symm, cref, INT)

fprintf(fid, '\n\tSetting aerodynamic mesh for DLM solver...');

void = 0;
% lattice section
lattice.ref_sys = [];   % box reference system
lattice.e = [];         % sending box semiwidth
lattice.np = 0;         % number of panels
lattice.npsymm = 0;     % number of panels for symmetric
lattice.symm_col = [];
lattice.lambda = [];    % sending box sweep angle
lattice.gamma = [];     % sending box dihedral angle
lattice.dx = [];        % sending box mean chord
lattice.N = [];         % box normal
lattice.COLLOC = [];    % boc collocation 3/4c
lattice.XYZ = [];     % nodes
lattice.DOUBLET = [];   % doublet line
lattice.MID_DPOINT = [];% doublet midpoint 
lattice.area = [];      % panel area
lattice.DOF = [];       % patches DOF
lattice.INT = [];       % interference

try
	geo.meshtype;
catch 
	geo.meshtype=ones(size(geo.T));
end

nt = size(geo.T,2);
TOTAL = zeros(size(geo.T));
for i = 1 :nt
    TOTAL(:,i) = geo.meshtype(:,i)==6;
    if ~isempty(find(TOTAL(:,i)==1))
        geo.meshtype(TOTAL(:,i)==1,i) = ones(size(TOTAL(TOTAL(:,i)==1,i)));
    end
end
clear nt


X=[];
Y=[];
Z=[];

S = 1;
Cmac = 0;
temp = 0;
CHORDS=[];

nwings = size(geo.nelem');


for s = 1:nwings

	CHORDS(s,1)=geo.c(s);
	SX(s,1)=geo.startx(s);
	SY(s,1)=geo.starty(s);
	SZ(s,1)=geo.startz(s); 

end

t=0;

for s = 1:nwings

	for t = 1:(geo.nelem(s))
      
      CHORDS(s,t+1)=CHORDS(s,t)*geo.T(s,t);
            
      SX(s,t+1) = 0.25*CHORDS(s,t)+geo.b(s,t)*(tan(geo.SW(s,t))) - 0.25 * CHORDS(s,t+1) + SX(s,t);					
      SY(s,t+1) = geo.b(s,t)*cos(geo.dihed(s,t))+SY(s,t);   
      SZ(s,t+1) = geo.b(s,t)*sin(geo.dihed(s,t))+SZ(s,t);

   end

end
flapc = 0;
dofc = 0;

%MAIN GEOMETRY SETUP LOOP, CREATES Partition QUAD PANELS, VORTICIES AND COLL-POINTS
for s=1:nwings
   for t=1:geo.nelem(s) %setuploop
      [C D N P ndof cdof hinge] = set_lattice_geometry(geo.fnx(s,t),geo.ny(s,t),geo.nx(s,t),...
         geo.fsym(s,t),geo.fc(s,t,:),geo.flapped(s,t),geo.TW(s,t,:),...
         geo.T(s,t),geo.SW(s,t),CHORDS(s,t),geo.dihed(s,t),geo.b(s,t),...
         geo.symetric(s),SX(s,t),SY(s,t),SZ(s,t),geo.meshtype(s,t));
        % new features: reflect data when negative span is detected
        if (geo.b(s,t)<0)
          fprintf(fid, '\n\t\tWarning: patch %d - part %d will be reverted (negative span given).', s,t);
          N = -N; % reflect normals
          P(1:end,:,:) = P(1:end,[2 1 4 3 2],:);
          D(1:end,:,:) = D(1:end,6:-1:1,:); 
          %hinge = hinge(1,[2 1],:)
        end

      lattice.DOF(s,t,1) = 1 + dofc;
	    lattice.DOF(s,t,2) = dofc + ndof;

      lattice.COLLOC=[lattice.COLLOC; C];
      lattice.DOUBLET=[lattice.DOUBLET; D];
      lattice.N=[lattice.N; N];
      lattice.INT = [lattice.INT; INT(s).*ones(ndof,1)];
      	  if geo.flapped(s,t)
	          
		  flapc = flapc +1;

		  lattice.Control.Patch(flapc) = s;
		  lattice.Control.Part(flapc) = t;
		  lattice.Control.Hinge(flapc,:,:) = hinge;
		  if TOTAL(s,t) == 1
              lattice.Control.DOF(flapc).data =  dofc+1:dofc + ndof;
          else
              lattice.Control.DOF(flapc).data = cdof + dofc;
          end
          
          
          lattice.Control

	  end	  

	  dofc = dofc + ndof;

	  Cmgc(s,t) = CHORDS(s,t) * ((1+geo.T(s,t)))/2;
      S(s,t) = geo.b(s,t) * Cmgc(s,t);  % patch area 
      
      if geo.symetric(s)==1
 
         S(s,t) = S(s,t)*2;
 
      end

      lattice.XYZ=[lattice.XYZ;P];

   end
end

lattice.np = size(lattice.N, 1);

%ref.b_ref = 0;
%ref.S_ref = 0;
%ref.C_mac = 0;
%ref.mac_pos = 0;

%B = sum(geo.b,2);
%ref.b_ref = B(1);	%reference span = half-span of first wing
%ref.b_ref = ref.b_ref*(geo.symetric(1)+1);

%S_r = sum(S,2);
%ref.S_ref = S_r(1);	%reference area = area of first wing

%C_m=sum(Cmgc.*S,2);	
%ref.C_mgc = C_m(1) / ref.S_ref;		%Mean Geometric Chord  Gross surface  Main (first)

%[ref.C_mac void]=fCmac(CHORDS(1,1),geo.b(1,:),geo.SW(1,:),...
%SX(1,:),SY(1,:),SZ(1,:),geo.dihed(1,:),geo.symetric(1)); %Main (first) wing Mean aerodymaic chord calculation 

%[void ref.mac_pos]=fCmac(CHORDS(1,1),geo.b(1,:),geo.SW(1,:),...
%SX(1,:),SY(1,:),SZ(1,:),geo.dihed(1,:),geo.symetric(1)); %Main (first) wing Mean aerodymaic chord calculation   
 
% scale geometry
lattice.XYZ = lattice.XYZ ./ cref;
ndoublet = size(lattice.DOUBLET, 2) / 2;

switch (ndoublet)

	case 3

    case 4
	
	otherwise

	error('Wrong size for vlm_lattice database (6 or 8 cols for VORTEX field).');
end

lattice.DOUBLET = lattice.DOUBLET(:, ndoublet:1:(ndoublet + 1), :) ./ cref;
lattice.COLLOC = lattice.COLLOC(:, :, :) ./ cref;

% determine for each panel its local coordinates system as follows:
% x_local along free-stream
% z_local along panel normal
% y_local by hand-right rule
totp = lattice.np + lattice.npsymm;
area = sqrt(dot(lattice.N, lattice.N, 2));

lattice.ref_sys = zeros(3, 3, totp);
lattice.gamma = zeros(totp, 1);
lattice.lambda = zeros(totp, 1);
lattice.e = zeros(totp, 1);
lattice.MID_DPOINT = zeros(totp, 3);
lattice.dx = zeros(totp, 1);
lattice.area = zeros(totp, 1);
lattice.cord = zeros(totp, 1);

x_loc = [1 0 0];
y_loc = zeros(1,3);
ds = zeros(1,3);
dx1 = zeros(1,3);
dx2 = zeros(1,3);

for j= 1: lattice.np

	lattice.N(j, :) = -lattice.N(j, :) ./ area(j); % revert Tornado normal convention
	y_loc = cross(lattice.N(j, :), x_loc);
	y_loc = y_loc ./ norm(y_loc);
	lattice.ref_sys(: , : , j) = [x_loc; y_loc; lattice.N(j, :)]; % rotation matrix from global to local coordinates	 
	ds(1) = lattice.DOUBLET(j, 2, 1) - lattice.DOUBLET(j, 1, 1);              
	ds(2) = lattice.DOUBLET(j, 2, 2) - lattice.DOUBLET(j, 1, 2);
	ds(3) = lattice.DOUBLET(j, 2, 3) - lattice.DOUBLET(j, 1, 3);
	% box semiwidth
	%lattice.e(j) = norm(ds) / 2;
	lattice.e(j) = sqrt(ds(2)^2 + ds(3)^2) / 2;
	% box sweep
	dx1(1,:) = lattice.DOUBLET(j, 2, :) - lattice.XYZ(j, 1, :); 
	dx2(1,:) = lattice.DOUBLET(j, 1, :) - lattice.XYZ(j, 1, :);
	lattice.MID_DPOINT(j,1:3) = (lattice.DOUBLET(j, 2, :) + lattice.DOUBLET(j, 1, :)) ./ 2; 
	% box dihedral
	lattice.gamma(j) = atan2(y_loc(3) , y_loc(2));
	% local points
	dx1 = (lattice.ref_sys(:,:,j) * dx1')'; 
	dx2 = (lattice.ref_sys(:,:,j) * dx2')';
	lattice.lambda(j) = atan( (dx1(1) - dx2(1)) / (dx1(2) - dx2(2)));
	% box mean chord
	dx1(1,:) = lattice.XYZ(j, 1, :) - lattice.XYZ(j, 4, :);
	dx2(1,:) = lattice.XYZ(j, 2, :) - lattice.XYZ(j, 3, :);
	dc14 = norm(dx1);
	dc23 = norm(dx2);
	lattice.dx(j) = (dc14 + dc23) / 2;
	% calculate real box area
    lattice.cord(j) = (dc14 + dc23)*0.5.* cref;
	lattice.area(j) = lattice.e(j) * (dc14 + dc23) .* cref^2;

end

if (symm ~= 0)

	index = find(abs(lattice.COLLOC(:,2)) > eps);

	lattice.npsymm = length(index);
	lattice.symm_col = index;

	coord = lattice.COLLOC(index,:);
	coord(:,2) = -coord(:,2);
	lattice.COLLOC = [lattice.COLLOC; coord];
	coord = lattice.XYZ(index,[2 1 4 3 2],:);
	coord(:,:,2) = -coord(:,:,2);
	lattice.XYZ = [lattice.XYZ; coord];
	coord = lattice.DOUBLET(index,2:-1:1,:);
	coord(:,:,2) = -coord(:,:,2);
	lattice.DOUBLET = [lattice.DOUBLET; coord];
	coord = lattice.MID_DPOINT(index,:);
	coord(:,2) = -coord(:,2);
	lattice.MID_DPOINT = [lattice.MID_DPOINT; coord];
	coord = lattice.N(index,:);
	coord(:,2) = -coord(:,2);
  lattice.N = [lattice.N; coord];
	lattice.gamma = [lattice.gamma; -lattice.gamma(index)];
	lattice.lambda = [lattice.lambda; -lattice.lambda(index)];
	lattice.e = [lattice.e; lattice.e(index)];
	lattice.dx = [lattice.dx; lattice.dx(index)];
	lattice.INT = [lattice.INT; lattice.INT(index)];
	
	offset = lattice.np;
	np_tot = offset + lattice.npsymm;

	for (j=offset+1 : np_tot)

		y_loc = cross(lattice.N(j, :), x_loc);
		y_loc = y_loc ./ norm(y_loc);
		lattice.ref_sys(: , : , j) = [x_loc; y_loc; lattice.N(j, :)];	 
	
	end
end

fprintf(fid, 'done.');

end

function[panel] = flat_mesh(wx,wy,wz,nx,ny,meshtype);
   panel = [];
   if (nx==0)
     return;
   end
   a1=[wx(1) wy(1) wz(1)];
   b1=[wx(2) wy(2) wz(2)];
   
   b2=[wx(3) wy(3) wz(3)];
   a2=[wx(4) wy(4) wz(4)];
    
   percent_cy=(0:ny)./ny;
   percent_cx=(0:nx)./nx;
   
   
   switch meshtype
       case 1
                %Linear lattice, both in x and y
       case 2
                %Linear in x, half cosine in y
                percent_cy=cos(pi/2*(1-percent_cy));
       case 3   
                %Cosine in x, half cosine in y
                percent_cx=(cos(pi*(1-percent_cx))+1)/2 ;   
                percent_cy=cos(pi/2*(1-percent_cy));                         
       case 4
                %Cosine in x, cosine in y
                percent_cx=(cos(pi*(1-percent_cx))+1)/2;    
                percent_cy=(cos(pi*(1-percent_cy))+1)/2;
       case 5
                
                percent_cx=2.2*percent_cx.^3 - 3.3*percent_cx.^2 + 2.1*percent_cx;
                percent_cy=2.2*percent_cy.^3 - 3.3*percent_cy.^2 + 2.1*percent_cy;
                
                
       otherwise
                disp('NOT IMPLEMENTED') 
                %Put new functione here for panel distribution scheme.
   end
   
   
   
  

for i=1:ny+1
   perc_y=percent_cy(i);
   
     	c1=b1-a1;
   	l1=norm(c1);
   	c1_hat=c1./l1;
   	d1=(perc_y)*l1*c1_hat;
      m=a1+d1;
      
      c2=b2-a2;
   	%l2=norm(c2);
      %c2_hat=c2./l2;
      %d2=(perc_y)*l2*c2_hat;
      d2=(perc_y)*c2;

      n=a2+d2;
   
   
   for j=1:nx+1
      
      perc_x=percent_cx(j);
      
     	c3=n-m;
 	 	%l3=norm(c3);
  		%c3_hat=c3./l3;
      %d3=(perc_x)*l3*c3_hat;
        d3=(perc_x)*c3;
        p=m+d3;
  
  		A(i,j,:)=[p];
  
	end
end

t=0;
for i=1:ny
   for j=1:nx
      t=t+1;
      panel(t,1,:)=A(i,j,:);
      panel(t,2,:)=A(i+1,j,:);
      panel(t,3,:)=A(i+1,j+1,:);
      panel(t,4,:)=A(i,j+1,:);
      panel(t,5,:)=A(i,j,:);   
   end
end

end

function norm_vec= flat_normal(colloc,vortex)

N=[];
step = size(colloc);
[d e f]=size(vortex);
a = e/2;
b = a+1;
      
for t=1:step
   
   	for s = 1:3
      	ra(s) = vortex(t,a,s);
      	rb(s) = vortex(t,b,s);
      	rc(s) = colloc(t,s);
    end

    r0 = rb-ra;
    r0(1) = 0;
    r1 = rc-ra;
    r2 = rc-rb;
   	n = cross(r1,r2);
    nl = sqrt(sum((n.^2),2));
    R = n / nl;
    N = [N; R]; 

end

norm_vec = N;

end

function [C_mac,mac_start_coo]=fCmac(C,b,SW,sx,sy,sz,dihed,sym)

[void noofpan]=size(C)								%Create counter

for i=1:noofpan-1;
   T(i)=C(i+1)/C(i);									%partition tapers

	Cb=C(i);												%Base chord
	Ct=C(i+1);											%Tip chord

	b_mac(i)=b(i)*(2*Ct+Cb)/(3*(Ct+Cb));			%Tp pos
	Cmac(i)=Cb-(Cb-Ct)/b(i)*b_mac(i);				%Chord at tp pos
   
   start(i,1)=0.25*Cb+b_mac(i)*tan(SW(i))-0.25*Cmac(i)+sx(i); %Mac start x position
   start(i,2)=cos(dihed(i))*b_mac(i)+sy(i);
   start(i,3)=sin(dihed(i))*b_mac(i)+sz(i);
end

if sym
   start(:,2)=0;  
end


A=(1+T).*C(1:end-1).*b./2;							%Area of partitions 
   
C_mac=sum(Cmac.*A)./sum(A);						% Area weighing for multiple  
mac_start_coo(1)=sum((start(:,1).*A')./sum(A));   		% partition wing
mac_start_coo(2)=sum((start(:,2).*A')./sum(A));
mac_start_coo(3)=sum((start(:,3).*A')./sum(A));   		% partition wing% partition wing
 
end

function [C,Vor,N,P,ndof,cdof, hinge] = set_lattice_geometry(fnx, ny, nx, fsym, fc, flapped, TW, T, SW, c, dihed, b, sym, sx, sy, sz, meshtype)

TEP=[];
TEP1=[];
TEP2=[];
INF=[];
INF1=[];
INF2=[];
%
% WARNING: erase given TWIST and place all panels along x-axis
% This value can  be in future overwritten along dlm_model.aero.V direction
%
TW = zeros(1,1,2);
%
ox=sx;
oy=sy;
oz=sz;
neqns=(nx+fnx)*ny;

if sym==1
	ndof = 2 * neqns;
else
	ndof = neqns;
end	

dx=(c*(1-fc(1))/nx);
if flapped==1
   fdx=(c*fc/fnx);
else
   fdx=0;
end   
%a1=ones(nx,1)*dx;
%a2=ones(fnx,1)*fdx;
%dr=[a1' a2'];

lem(1)=0.25*c;
lem(2)=0.25*T*c;
lem(3)=-0.75*T*c;
lem(4)=-0.75*c;

DX=[(1-cos(TW(1,1,1)))*cos(SW) (1-cos(TW(1,1,2)))*cos(SW)...
      (1-cos(TW(1,1,2)))*cos(SW) (1-cos(TW(1,1,1)))*cos(SW)].*lem;

DY=-[sin(TW(1,1,1))*sin(dihed)*cos(SW) sin(TW(1,1,2))*sin(dihed)*cos(SW)...
      sin(TW(1,1,2))*sin(dihed)*cos(SW) sin(TW(1,1,1))*sin(dihed)*cos(SW)].*lem;

DZ=[sin(TW(1,1,1))*cos(dihed) sin(TW(1,1,2))*cos(dihed) sin(TW(1,1,2))*cos(dihed)...
      sin(TW(1,1,1))*cos(dihed)].*lem;

wingx=[0 0.25*c+b*tan(SW)-0.25*T*c 0.25*c+b*tan(SW)+0.75*T*c c]+ox+DX;
wingy=[0 b*cos(dihed) b*cos(dihed) 0]+oy+DY;
wingz=[0 b*sin(dihed) b*sin(dihed) 0]+oz+DZ;

hinge = zeros(1,2,3);
cdof = [];

if flapped==1

	[flapx flapy flapz] = drawhinge(wingx, wingy, wingz, fc);

	hinge(1,1,1) = flapx(1);
	hinge(1,2,1) = flapx(2);
	
	hinge(1,1,2) = flapy(1);
	hinge(1,2,2) = flapy(2);
	
	hinge(1,1,3) = flapz(1);
	hinge(1,2,3) = flapz(2);
	
	offset = 0;
	
	for i = 1:ny
	
		cdof = [cdof, [(offset + nx +1) : (offset + nx + fnx)]];
		
		offset = offset + nx + fnx;
	
	end	

end
if flapped==0
	[p]=flat_mesh(wingx,wingy,wingz,nx,ny,meshtype);
	PX(:,:)=p(:,:,1);
	PY(:,:)=p(:,:,2);
	PZ(:,:)=p(:,:,3);
else
   tempx=wingx(3:4);
   tempy=wingy(3:4);
   tempz=wingz(3:4);
   
   wingx(3:4)=fliplr(flapx(1:2));
   wingy(3:4)=fliplr(flapy(1:2));
   wingz(3:4)=fliplr(flapz(1:2));
   
   flapx(3:4)=tempx;
   flapy(3:4)=tempy;
   flapz(3:4)=tempz;
   
   [p]=flat_mesh(wingx,wingy,wingz,nx,ny,meshtype);
   [q]=flat_mesh(flapx,flapy,flapz,fnx,ny,meshtype);
   
	 r=[];
    for i=1:ny 
       count1=((1:nx)+(nx*(i-1)));
       count2=(1:fnx)+(fnx*(i-1));
       r=[r;p(count1,:,:);q(count2,:,:)];      
    end
    
   PX(:,:)=r(:,:,1);
	PY(:,:)=r(:,:,2);
	PZ(:,:)=r(:,:,3);
end
nx=nx+fnx;

t=0;
for j=0:(ny-1);
  for i=0:(nx-1);
      t=t+1;
      
      px=PX(t,:);
      py=PY(t,:);
      pz=PZ(t,:);
       
      if i==(nx-fnx-1) %if the panel is the rearest chordwise on wing, forward of flap. 
      	for s=0:(nx-fnx-1);          
            HP(t-s,1,:)=[px(4) py(4) pz(4)];
            HP(t-s,2,:)=[px(3) py(3) pz(3)];         
            % TEP=Trailing edge points, Vortex points on the trailing edge            
            if sym==1
               %Port side points
               HP(t-s+neqns,1,:)=[px(3) -py(3) pz(3)];
               HP(t-s+neqns,2,:)=[px(4) -py(4) pz(4)];        
            end
      	end
      end
      
      
      if i==(nx-1);		%if the panel is the rearest chordwise on both wing and flap
         for s=0:(nx-1);          
            TEP1(t-s,1,:)=[px(4) py(4) pz(4)];
            TEP1(t-s,2,:)=[px(3) py(3) pz(3)];         
            % TEP=Trailing edge points, Vortex points on the trailing edge            
            if sym==1
               %Port side points
               TEP1(t-s+neqns,1,:)=[px(3) -py(3) pz(3)];
             	TEP1(t-s+neqns,2,:)=[px(4) -py(4) pz(4)];        
            end
               
            for u=0:(fnx-1)	%Hinge points for flap (equals trailing points)
               HP(t-u,1,:)=[px(4) py(4) pz(4)];
  					HP(t-u,2,:)=[px(3) py(3) pz(3)];  
               if sym==1
               	%Port side points
               	HP(t-u+neqns,1,:)=[px(3) -py(3) pz(3)];
               	HP(t-u+neqns,2,:)=[px(4) -py(4) pz(4)];        
               end  
            end    
         end
      end
      
      mx=sum(px(1:4))/4;		%panel midpoint x-coord
      my=sum(py(1:4))/4;
      mz=sum(pz(1:4))/4;
      
      bkx=(px(3)+px(4))/2;		%panel rear edge avarage x-coord
      
       C1(t,1)=(mx+bkx)/2;				%SB-Collocation point x-coord.      
       C1(t,2)=(py(3)+py(4)+2*my)/4;		%SB-Collocation point y-coord.
       C1(t,3)=(pz(3)+pz(4)+2*mz)/4;		%SB-Collocation point z-coord.
      if sym==1
 	     	C2(t,1)=C1(t,1);					%P-Collpoint x-coord. 
  	   	C2(t,2)=-C1(t,2);					%P-Collpoint y-coord.
   	   C2(t,3)=C1(t,3);					%P-Collpoint z-coord.
      else
         C2=[];
      end
		
   

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %Vortex tensor generation and plot % 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      ax=((px(1)+px(4))/2+px(1))*0.5;	%vortex first point
      ay=(3*py(1)+py(4))/4;
      az=(3*pz(1)+pz(4))/4;
      
      bx=((px(2)+px(3))/2+px(2))*0.5;	%vortex second point
      by=(3*py(2)+py(3))/4;
      bz=(3*pz(2)+pz(3))/4;
      
      V1(t,1,1)=ax;
      V1(t,1,2)=ay;
      V1(t,1,3)=az;
      V1(t,2,1)=bx;
      V1(t,2,2)=by;
      V1(t,2,3)=bz;
      
      if sym==1;
         V1(t+neqns,1,1)=bx;
      	V1(t+neqns,1,2)=-by;
      	V1(t+neqns,1,3)=bz;
      	V1(t+neqns,2,1)=ax;
      	V1(t+neqns,2,2)=-ay;
         V1(t+neqns,2,3)=az;
      end
      
      
     end
end 

     C=[C1;C2];
     V=V1;
     Vor=[TEP1(:,1,:) HP(:,1,:) V(:,:,:) HP(:,2,:) TEP1(:,2,:)];
     
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculating normals              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = flat_normal(C,V);
V=Vor;

if sym==1
   PX2(:,1)=PX(:,2);
   PX2(:,2)=PX(:,1);
   PX2(:,3)=PX(:,4);
   PX2(:,4)=PX(:,3);
   PX2(:,5)=PX2(:,1);
   
   PY2(:,1)=PY(:,2);
   PY2(:,2)=PY(:,1);
   PY2(:,3)=PY(:,4);
   PY2(:,4)=PY(:,3);
   PY2(:,5)=PY2(:,1);
   
   PZ2(:,1)=PZ(:,2);
   PZ2(:,2)=PZ(:,1);
   PZ2(:,3)=PZ(:,4);
   PZ2(:,4)=PZ(:,3);
   PZ2(:,5)=PZ2(:,1);
   
   
   PX=[PX;PX2];
   PY=[PY;-PY2];
   PZ=[PZ;PZ2];   
end

P(:,:,1)=PX;
P(:,:,2)=PY;
P(:,:,3)=PZ;

end

function[x,y,z]=drawhinge(wx,wy,wz,fc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DRAWHINGE: subsidary function to TORNADO	%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function that draws the hinge line on		%
% on a wing division. It also returns the 	%
% coordinates on the foremost flap corners	%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Called by: Geometry									
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT: WX,WY,WZ = wing cornerpoint coor-		
%   		dinates.										
%			fc is the percentage of total chord	
%			built up by the flap						
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUT: 	graph (in figure (2))				
%				flap cornerpoint coor-				
%				dinates									
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isempty(fc);
   
   x=[];
   y=[];
   z=[];
   
else
   
	for i=0:1
  	 	
		a=[wx(1+i) wy(1+i) wz(1+i)];
   		b=[wx(4-i) wy(4-i) wz(4-i)];
   
   		c=b-a;
   		l=norm(c);
   		c_hat=c./l;
   		d=(1-fc(i+1))*l*c_hat;

   		r=a+d;

   		R1(i+1,:)=[r];
   		R2(i+1,:)=[r];
	end

	x=[R1(:,1)'];
	y=[R1(:,2)'];
	z=[R1(:,3)'];
	
	end
end
