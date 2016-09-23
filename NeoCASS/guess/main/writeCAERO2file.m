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
%--------------------------------------------------------------------------------------------------
% Define necessary parameters to write CAERO1 card
% 
% CAERO1 card
% 
% Called by:    MainCode.m
% 
% Calls:        writeCAERO2file_Wing.m, writeCAERO12file_Vert.m, writeCAERO12file_Hori.m 
% 
% MODIFIED 2008-07-23
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [stick] = writeCAERO2file(outf, fid, aircraft, stick, geo, SELECT, BODYEXP)
% Modified by Travaglini 19/11/2009 to improve the mesh on control surface
if (isempty(SELECT))
  INDEX=1;
else
  INDEX = SELECT;
end
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Bodies and lifting surfaces definition');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
if (outf~=0)
  fprintf(outf, '\n\tExporting aerodynamic lifting surfaces and bodies...');
end
if (INDEX==0 || INDEX==1)
  if (BODYEXP)
    if isequal(stick.model.fuse, 1)
      P1 = [geo.fus.xx(3); 0; geo.fus.zz(3)]; % origin
      P3 = [geo.fus.xx(4); 0; geo.fus.zz(4)]-P1; % x axis
      P2 = [0; 1; 0]; % y axis
      P2 = crossm(P3)* P2; % z axis
%      ORIGIN = geo.wing.PANE(:,1)';
%      ALPHA_FUS = aircraft.wing1.root_incidence*pi/180;
%      ROT = Rmat([0 ALPHA_FUS 0]);
%      P2 = ROT(:,3);      
%      P3 = ROT(:,1);
%     export body reference frame
      BULKdataCORD2R(fid, stick.IDSET.fuse, 0, zeros(3,1), P2, P3);
      P1 = [geo.fus.xx(1); 0; geo.fus.zz(1)] - ...
           [geo.fus.xx(3); 0; geo.fus.zz(3)];
      P2 = [geo.fus.xx(4); 0; geo.fus.zz(4)] - ...
           [geo.fus.xx(3); 0; geo.fus.zz(3)];
      P2 = P2 ./ norm(P2);
      dl = dot(P1, P2);
%     body projected nose
      AP1 = P2.*dl + [geo.fus.xx(3); 0; geo.fus.zz(3)]; 
      P1 = [geo.fus.xx(end); 0; geo.fus.zz(end)] - ...
           [geo.fus.xx(3); 0; geo.fus.zz(3)];
      dl = dot(P1, P2);
      AP2  = P2.*dl + [geo.fus.xx(3); 0; geo.fus.zz(3)]; 
%     projected body length
      L = norm(AP2-AP1);
%     number of elements
      NE = length(geo.fus.thick_dom);
      BULKdataCAEROB(fid, stick.IDSET.fuse, AP1, stick.IDSET.fuse, L, NE, ...
                     stick.IDSET.fuse, geo.fus.thick_dom./max(geo.fus.thick_dom), geo.fus.thick_ver);
    end
  end
end
if (INDEX==0 || INDEX==2)
  if isequal(stick.model.winr, 1)
      [stick] = writeCAERO12file_Wing(fid, aircraft, stick, geo);
  end
end
%
if (INDEX==0 || INDEX==4)
  if isequal(stick.model.horr, 1)
      if isfield(aircraft.Horizontal_tail,'Allmovable') && aircraft.Horizontal_tail.Allmovable==1
          meshtype = 6;
      else
          meshtype = 1;
      end
     [stick] = writeCAERO12file_Hori(fid,  stick, geo,meshtype);
  end
end
%
if (INDEX==0 || INDEX==3)
  if isequal(stick.model.vert, 1)
      if isfield(aircraft.Vertical_tail,'Allmovable') && aircraft.Vertical_tail.Allmovable==1
          meshtype = 6;
      else
          meshtype = 1;
      end
      [stick] = writeCAERO12file_Vert(fid, aircraft, stick, geo,meshtype);
  end
end
%
if (INDEX==0 || INDEX==6)
  if isequal(stick.model.vert, 1)
      if isfield(aircraft.Vertical_tail,'Allmovable') && aircraft.Vertical_tail.Allmovable==1
          meshtype = 6;
      else
          meshtype = 1;
      end
      [stick] = writeCAERO12file_Vert2(fid, aircraft, stick, geo,meshtype);
  end
end
%
if (INDEX==0 || INDEX==5)
  if isequal(stick.model.canr, 1) && aircraft.Strut_wing.present~=1
      if isfield(aircraft.Canard,'Allmovable') && aircraft.Canard.Allmovable==1
          meshtype = 6;
      else
          meshtype = 1;
      end
     [stick] = writeCAERO12file_CanardT(fid,  stick, geo,meshtype);
  end
end
if (outf~=0)
  fprintf(outf, 'done.');
end
end % end of writeCAERO12file.m, DO NOT REMOVE
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
function [stick] = writeCAERO12file_Wing(fid, aircraft, stick, geo)
% The function provides the necessary calculations to write CAERO1 card
%
IDCAERORN = [];
IDCAEROLN = [];
NXSR = [];
NYSR = [];
NXSL = [];
NYSL = [];
stick.IDCAERO1.winrP = [];
stick.IDCAERO1.winlP = [];
stick.part.winrP = [];
for i = 1:length(stick.IDCAERO1.winr)
    %
    k = 1 + (i-1)*4;
    t = 1 + (i-1)*2; 
    %
    % write CAERO1 card 
    [IDMR, NXMR, NYMR, IDSR, NXSR, NYSR] = BULKdataCAERO1_mod(fid, stick.IDCAERO1.winr(i), geo.wing.CAERO1.dihedral(i), 0, stick.ny.wing(i), stick.nx.wing(i),...
        cell2mat(geo.wing.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.wing.CAERO1.airfoil(2*i,:)), 1,...
        stick.ptospanel.winr(1,k), stick.ptospanel.winr(2,k), stick.ptospanel.winr(3,k),...
        geo.wing.CAERO1.chord(i), geo.wing.CAERO1.span(i), geo.wing.CAERO1.taper(i),...
        geo.wing.CAERO1.sweepQC(i), geo.wing.CAERO1.incidence(i), geo.wing.CAERO1.incidence(i+1),...
        geo.wing.CAERO1.sup_control.frc(t), geo.wing.CAERO1.sup_control.frc(t+1),...
        stick.nx.sup_control.wing(i), geo.wing.CAERO1.sup_control.nme(i,:),...
        geo.wing.CAERO1.sup_control.frs(i), geo.wing.CAERO1.sup_control.position(i), geo.wing.CAERO1.sup_control.frc(t+2),... 
        cell2mat(geo.wing.CAERO1.airfoil(i+1,:)));
        stick.nTOT.wing(i) = NXMR * NYMR;
        if(~isempty(IDSR))
          stick.nTOT.wingP = NYSR .* NXSR;
          stick.IDCAERO1.winrP = IDSR;
          stick.part.winrP = [stick.part.winrP, repmat(stick.part.winr(i),1,length(IDSR))];
        end
    %
    % symmetry
    if isequal(stick.model.symmXZ, 1)
        % write CAERO1 card
        [IDML, NXML, NYML, IDSL, NXSL, NYSL] = BULKdataCAERO1_mod(fid, stick.IDCAERO1.winl(i), -geo.wing.CAERO1.dihedral(i), 0, stick.ny.wing(i), stick.nx.wing(i),...
            cell2mat(geo.wing.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.wing.CAERO1.airfoil(2*i,:)), 1,...
            stick.ptospanel.winr(1,k), -stick.ptospanel.winr(2,k), stick.ptospanel.winr(3,k),...
            geo.wing.CAERO1.chord(i), -geo.wing.CAERO1.span(i), geo.wing.CAERO1.taper(i),...
            -geo.wing.CAERO1.sweepQC(i), geo.wing.CAERO1.incidence(i), geo.wing.CAERO1.incidence(i+1),...
            geo.wing.CAERO1.sup_control.frc(t), geo.wing.CAERO1.sup_control.frc(t+1),...
            stick.nx.sup_control.wing(i), geo.wing.CAERO1.sup_control.nme(i+length(stick.IDCAERO1.winr),:),...
            geo.wing.CAERO1.sup_control.frs(i),geo.wing.CAERO1.sup_control.position(i), geo.wing.CAERO1.sup_control.frc(t+2),...
            cell2mat(geo.wing.CAERO1.airfoil(i+1,:)));
%        stick.nTOT.wing(i) = NXML * NYML;
        if(~isempty(IDSL))
          stick.IDCAERO1.winlP = IDSL;
        end
    end
end
end % end of writeCAERO12file_Wing.m, DO NOT REMOVE
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
function [stick] = writeCAERO12file_Hori(fid,  stick, geo,meshtype)

% The function provides the necessary calculations to write CAERO1 card
%

for i = 1:length(stick.IDCAERO1.horr)
    %
    k = 1 + (i-1)*4;
    t = 1 + (i-1)*2;
    %
    % write CAERO1 card
    BULKdataCAERO1(fid, stick.IDCAERO1.horr(i), geo.htail.CAERO1.dihedral(i), 0, stick.ny.hori(i), stick.nx.hori(i),...
                   cell2mat(geo.htail.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.htail.CAERO1.airfoil(2*i,:)), meshtype,...
                   stick.ptospanel.horr(1,k), stick.ptospanel.horr(2,k), stick.ptospanel.horr(3,k),...
                   geo.htail.CAERO1.chord(i), geo.htail.CAERO1.span(i), geo.htail.CAERO1.taper(i),...
                   geo.htail.CAERO1.sweepQC(i), geo.htail.CAERO1.incidence(i), geo.htail.CAERO1.incidence(i+1),...
                   geo.htail.CAERO1.sup_control.frc(t), geo.htail.CAERO1.sup_control.frc(t+1),...
                   stick.nx.sup_control.hori(i), geo.htail.CAERO1.sup_control.nme(i,:));
    %
	% symmetry
    if isequal(stick.model.symmXZ, 1)               
        % write CAERO1 card
        BULKdataCAERO1(fid, stick.IDCAERO1.horl(i), -geo.htail.CAERO1.dihedral(i), 0, stick.ny.hori(i), stick.nx.hori(i),...
                       cell2mat(geo.htail.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.htail.CAERO1.airfoil(2*i,:)), meshtype,...
                       stick.ptospanel.horr(1,k), -stick.ptospanel.horr(2,k), stick.ptospanel.horr(3,k),...
                       geo.htail.CAERO1.chord(i), -geo.htail.CAERO1.span(i), geo.htail.CAERO1.taper(i),...
                       -geo.htail.CAERO1.sweepQC(i), geo.htail.CAERO1.incidence(i), geo.htail.CAERO1.incidence(i+1),...
                       geo.htail.CAERO1.sup_control.frc(t), geo.htail.CAERO1.sup_control.frc(t+1),...
                       stick.nx.sup_control.hori(i), geo.htail.CAERO1.sup_control.nme(i+length(stick.IDCAERO1.horr),:));
    end       	
end

end % end of writeCAERO12file_Hori.m, DO NOT REMOVE
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
function [stick] = writeCAERO12file_Vert(fid, aircraft, stick, geo,meshtype)

% The function provides the necessary calculations to write CAERO1 card


for i = 1:length(stick.IDCAERO1.vert)                   
    %
    k = 1 + (i-1)*4;
    %
    % write CAERO1 card
    BULKdataCAERO1(fid, stick.IDCAERO1.vert(i), geo.vtail.CAERO1.dihedral(i), 0, stick.ny.vert(i), stick.nx.vert(i),...
                   cell2mat(geo.vtail.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.vtail.CAERO1.airfoil(2*i,:)),meshtype,...
                   stick.ptospanel.vert(1,k), stick.ptospanel.vert(2,k), stick.ptospanel.vert(3,k),...
                   geo.vtail.CAERO1.chord(i), geo.vtail.CAERO1.span(i), geo.vtail.CAERO1.taper(i),...
                   geo.vtail.CAERO1.sweepQC(i), geo.vtail.CAERO1.incidence(i), geo.vtail.CAERO1.incidence(i+1),...
                   geo.vtail.CAERO1.sup_control.frc(i), geo.vtail.CAERO1.sup_control.frc(i+1),...
                   stick.nx.sup_control.vert(i), geo.vtail.CAERO1.sup_control.nme(i,:));
end

end % end of writeCAERO12file_Vert.m, DO NOT REMOVE
%--------------------------------------------------------------------------

function [stick] = writeCAERO12file_Vert2(fid, aircraft, stick, geo,meshtype)

% The function provides the necessary calculations to write CAERO1 card

dim=size(geo.vtail.CAERO1.sup_control.nme);
dim=dim(1);
if isequal(aircraft.Vertical_tail.Twin_tail, 1)
    for i = 1:length(stick.IDCAERO1.vert)
        %
        k = 1 + (i-1)*4;
        %
        % write CAERO1 card  
        BULKdataCAERO1(fid, stick.IDCAERO1.vert2(i), -geo.vtail.CAERO1.dihedral(i), 0, stick.ny.vert(i), stick.nx.vert(i),...
            cell2mat(geo.vtail.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.vtail.CAERO1.airfoil(2*i,:)),meshtype,...
            stick.ptospanel.vert2(1,k), stick.ptospanel.vert2(2,k), stick.ptospanel.vert2(3,k),...
            geo.vtail.CAERO1.chord(i), -geo.vtail.CAERO1.span(i), geo.vtail.CAERO1.taper(i),...
            -geo.vtail.CAERO1.sweepQC(i), geo.vtail.CAERO1.incidence(i), geo.vtail.CAERO1.incidence(i+1),...
            geo.vtail.CAERO1.sup_control.frc(i), geo.vtail.CAERO1.sup_control.frc(i+1),...
            stick.nx.sup_control.vert(i), geo.vtail2.CAERO1.sup_control.nme(i+dim,:));
    end
end

end % end of writeCAERO12file_Vert.m, DO NOT REMOVE
%--------------------------------------------------------------------------

function [stick] = writeCAERO12file_CanardT(fid,  stick, geo,meshtype)

% The function provides the necessary calculations to write CAERO1 card
%
for i = 1:length(stick.IDCAERO1.canr)
    %
    k = 1 + (i-1)*4;
    t = 1 + (i-1)*2;
    %
    % write CAERO1 card
    BULKdataCAERO1(fid, stick.IDCAERO1.canr(i), geo.canard.CAERO1.dihedral(i), 0, stick.ny.canr(i), stick.nx.canr(i),... 
                   cell2mat(geo.canard.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.canard.CAERO1.airfoil(2*i,:)),meshtype,...
                   stick.ptospanel.canr(1,k), stick.ptospanel.canr(2,k), stick.ptospanel.canr(3,k),...
                   geo.canard.CAERO1.chord(i), geo.canard.CAERO1.span(i), geo.canard.CAERO1.taper(i),...
                   geo.canard.CAERO1.sweepQC(i), geo.canard.CAERO1.incidence(i), geo.canard.CAERO1.incidence(i+1),...
                   geo.canard.CAERO1.sup_control.frc(t), geo.canard.CAERO1.sup_control.frc(t+1),...
                   stick.nx.sup_control.canr(i), geo.canard.CAERO1.sup_control.nme(i,:));
    %
	% symmetry
    if isequal(stick.model.symmXZ, 1)               
        % write CAERO1 card
        BULKdataCAERO1(fid, stick.IDCAERO1.canl(i), -geo.canard.CAERO1.dihedral(i), 0, stick.ny.canr(i), stick.nx.canr(i),...
                       cell2mat(geo.canard.CAERO1.airfoil((i-1)*2+1,:)), cell2mat(geo.canard.CAERO1.airfoil(2*i,:)),meshtype,...
                       stick.ptospanel.canr(1,k), -stick.ptospanel.canr(2,k), stick.ptospanel.canr(3,k),...
                       geo.canard.CAERO1.chord(i), -geo.canard.CAERO1.span(i), geo.canard.CAERO1.taper(i),...
                       -geo.canard.CAERO1.sweepQC(i), geo.canard.CAERO1.incidence(i), geo.canard.CAERO1.incidence(i+1),...
                       geo.canard.CAERO1.sup_control.frc(t), geo.canard.CAERO1.sup_control.frc(t+1),...
                       stick.nx.sup_control.canr(i), geo.canard.CAERO1.sup_control.nme(i+length(stick.IDCAERO1.canr),:));
    end       	
end


end % end of writeCAERO12file_canard.m, DO NOT REMOVE
%--------------------------------------------------------------------------





