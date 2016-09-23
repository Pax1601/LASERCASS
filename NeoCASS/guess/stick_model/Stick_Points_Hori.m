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
%---------------------------------------------------------------------------------------------------------------------------
%
%
%
% Called by:    Stick_Points.m
%
% Calls:        RotInc, rot3D.m
%
%   <andreadr@kth.se>
%---------------------------------------------------------------------------------------------------------------------------
% Mofified by Travaglini 19/11/2009 Htail can be linked to fuselage or
% vtail. when islinked to vertical tail a new sectoris defined both for
% htail and vtail (when twintail is present too).
function [stick, geo] = Stick_Points_Hori(aircraft, stick, geo)
% Quarter-chord points
stick.ptos.horr = geo.htail.QC;
% Elastic line points
stick.ptos.horrC2 = geo.htail.C2;
% Panel points
stick.ptospanel.horr = geo.htail.PANE;
stick.ptospanel.horl = stick.ptospanel.horr;
stick.ptospanel.horl(2,:) = -stick.ptospanel.horl(2,:);
%
% Cut double poins generated in geometry modulus
if length(geo.htail.CAERO1.n) > 1
    stick.ptos.horr(:,3:2:end-1) = [];
    stick.ptos.horrC2(:,3:2:end-1) = [];
end
stick.part.horr = [];
%--------------------------------------------------------------------------------------------------
% sector 1
if (geo.htail.span_inboard ~= 0)
    stick.part.horr = [stick.part.horr, 2];
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.horr)
        stick.IDCAERO1.horr = 401;
    else
        stick.IDCAERO1.horr = [stick.IDCAERO1.horr; stick.IDCAERO1.horr(end)+1];
    end
    %
    stick.nx.hori = [stick.nx.hori; stick.nx.hori_inboard];
    stick.ny.hori = [stick.ny.hori; stick.ny.hori_inboard];
    % Define total number of panels in each CAERO1 panel
    stick.nTOT.hori = [stick.nTOT.hori; stick.nx.hori_inboard*stick.ny.hori_inboard];
    % Number of panels within control surface
    stick.nx.sup_control.hori = [stick.nx.sup_control.hori; stick.nx.sup_control.hori_inboard];
    
end
%
% sector 2
if (geo.htail.span_outboard ~= 0)
    stick.part.horr = [stick.part.horr, 3];
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.horr)
        stick.IDCAERO1.horr = 402;
    else
        stick.IDCAERO1.horr = [stick.IDCAERO1.horr; stick.IDCAERO1.horr(end)+1];
    end
    %
    stick.nx.hori = [stick.nx.hori; stick.nx.hori_outboard];
    stick.ny.hori = [stick.ny.hori; stick.ny.hori_outboard];
    % Define total number of panels in each CAERO1 panel
    stick.nTOT.hori = [stick.nTOT.hori; stick.nx.hori_outboard*stick.ny.hori_outboard];
    % Number of panels within control surface
    stick.nx.sup_control.hori = [stick.nx.sup_control.hori; stick.nx.sup_control.hori_outboard];
    
end
%
% carrythrough sector
if geo.htail.twc > 0.0
    stick.part.horr = [1, stick.part.horr];
    av_size(1) =  geo.wing.span_inboard / stick.ny.wing_inboard;
    av_size(2) =  geo.wing.span_outboard / stick.ny.wing_outboard;
    av_size = ceil(abs(stick.ptos.horr(2,2))./av_size);
    index = find(av_size>0);
    stick.ny.hori_carryth = av_size(index(1));
    % Define ID numbers for CAERO1 panels
    stick.IDCAERO1.horr = [400;stick.IDCAERO1.horr];
    % Number of panels
    stick.nx.hori = [stick.nx.hori(1)+stick.nx.sup_control.hori(1); stick.nx.hori];
    stick.ny.hori = [stick.ny.hori_carryth; stick.ny.hori];
    % Define total number of panels in each CAERO1 panel
    stick.nTOT.hori = [stick.nx.hori(1)*stick.ny.hori(1); stick.nTOT.hori];
    % Number of panels within control surface
    stick.nx.sup_control.hori = [0; stick.nx.sup_control.hori];
    
end
%
%--------------------------------------------------------------------------------------------------
% Update total number of panels if some control surfaces are defined
stick.nTOT.hori = (stick.nx.hori + stick.nx.sup_control.hori).*stick.ny.hori;
%--------------------------------------------------------------------------------------------------
% 1 CASE
%
% VT/HT or HT/fuselage connection point estabilishment
%
if isequal(stick.model.vert, 1) && isequal(aircraft.Vertical_tail.Twin_tail, 0)
  % Horizontal tail can be link to fuselage or to vertical tail
  [row, col] = size(stick.ptos.vert);
  %
  Ind = [];

  for i = 1:col-1
    if (stick.ptos.horrC2(3,1) > stick.ptos.vert(3,i) && stick.ptos.horrC2(3,1) < stick.ptos.vert(3,i+1))
      % HT is within a segment in VT
      Ind = i;
    end
  end
%---------------------
%  NO CONNECTION FOUND
  if (isempty(Ind))
    npf = size(stick.ptos.fuse,2);  
    dist = (stick.ptos.fuse - repmat(stick.ptos.horrC2(:,1),1,npf))';
    [ZMINFUSE,fuseindex] = (min(sqrt(dot(dist,dist,2))));
    npf = size(stick.ptos.vert,2);  
    dist = (stick.ptos.vert - repmat(stick.ptos.horrC2(:,1),1,npf))';
    [ZMINVT, vtindex] = (min(sqrt(dot(dist,dist,2))));
    if ZMINVT < ZMINFUSE
      IndD = vtindex;
      stick.link.hori = 'vert';

    % verify if z-coord for HT coincide with one for VT
    % check the vertival position of HT
    if vtindex==1
      IndD = 1;
      stick.ptos.horrC2(3,1) = stick.ptos.vert(3,1);
    else
      IndD = length(stick.ptos.vert(3,:));
      stick.ptos.horrC2(3,1) = stick.ptos.vert(3,end);
    end
else

      ptofwZ = interp1( stick.ptos.fuse(1,:), stick.ptos.fuse(3,:), stick.ptos.horrC2(1,1) );
      ptofw = [stick.ptos.horrC2(1,1); stick.ptos.horrC2(2,1); ptofwZ];
      if isempty( find( ptofw(1)==stick.ptos.fuse(1,:) ,1) )
      %         check the distance between ptofw and other points, if it is to
      %         near to another one it will sobstitute by ptos
      %  check if HT is downstream wrt fuselage
        if isempty(find(abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse,1)) && (ptofw(1)<stick.ptos.fuse(1,end))
          stick.ptos.fuse = [stick.ptos.fuse, ptofw];
        else
        % The node is not sobsituted because it could be the
        % master node of fus-vert link
          Near = stick.ptos.fuse(1,:)-ptofw(1);
          [dummy,indDX] = min(abs(Near));
          stick.htail.DX =Near(indDX);
          %                     stick.ptos.fuse(:,abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.001*geo.fus.bodl) = ptofw;
        end
      end
      [Y, J] = sort(stick.ptos.fuse(1,:), 'ascend');
      % Sort points
      stick.ptos.fuse(1,:) = Y;
      stick.ptos.fuse(2,:) = stick.ptos.fuse(2,J);
      stick.ptos.fuse(3,:) = stick.ptos.fuse(3,J);
      stick.link.hori = 'fuse';
    end
%---------------------
%  CONNECTION FOUND
  else
    dz = stick.ptos.vert(3,Ind+1)-stick.ptos.vert(3,Ind);
    % add new point for VT
    xHorVer = stick.ptos.vert(1,Ind) + (stick.ptos.vert(1,Ind+1)-stick.ptos.vert(1,Ind))/dz*(stick.ptos.horrC2(3,1)-stick.ptos.vert(3,Ind));
    ptoHorr = [xHorVer;0;stick.ptos.horrC2(3,1)];
    if isempty(find(abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse),1))
      %
      chord = geo.vtail.CAERO1.chord(Ind) + (geo.vtail.CAERO1.chord(Ind+1)-geo.vtail.CAERO1.chord(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
      geo.vtail.CAERO1.chord = [geo.vtail.CAERO1.chord(1:Ind) ; chord; geo.vtail.CAERO1.chord(Ind+1:end)];
      dihedral = geo.vtail.CAERO1.dihedral(Ind);
      geo.vtail.CAERO1.dihedral = [geo.vtail.CAERO1.dihedral(1:Ind); dihedral; geo.vtail.CAERO1.dihedral(Ind+1:end)];

      geo.vtail.CAERO1.span(Ind) = ptoHorr(3)-stick.ptos.vert(3,Ind);
      span = stick.ptos.vert(3,Ind+1)-ptoHorr(3);
      geo.vtail.CAERO1.span = [geo.vtail.CAERO1.span(1:Ind);span;geo.vtail.CAERO1.span(Ind+1:end)];

      geo.vtail.CAERO1.taper = [geo.vtail.CAERO1.taper(1:Ind-1);chord/geo.vtail.CAERO1.chord(Ind); geo.vtail.CAERO1.chord(Ind+2)/chord ;geo.vtail.CAERO1.taper(Ind+1:end)];

      geo.vtail.CAERO1.sweepLE = [geo.vtail.CAERO1.sweepLE(1:Ind); geo.vtail.CAERO1.sweepLE(Ind);geo.vtail.CAERO1.sweepLE(Ind+1:end)];

      geo.vtail.CAERO1.sweepQC = [geo.vtail.CAERO1.sweepQC(1:Ind);geo.vtail.CAERO1.sweepQC(Ind);geo.vtail.CAERO1.sweepQC(Ind+1:end)];

      incidence = geo.vtail.CAERO1.incidence(Ind) + (geo.vtail.CAERO1.incidence(Ind+1)-geo.vtail.CAERO1.incidence(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
      geo.vtail.CAERO1.incidence = [geo.vtail.CAERO1.incidence(1:Ind) ; incidence; geo.vtail.CAERO1.incidence(Ind+1:end)];


      geo.vtail.CAERO1.airfoil = [geo.vtail.CAERO1.airfoil(1:(Ind-1)*2+2);...
      geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
      geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
      geo.vtail.CAERO1.airfoil(Ind*2+1:end)];
      if aircraft.Vertical_tail.Rudder.present == 1
        sup_control_frc = geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) + (geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2)-geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) )/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
        geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
        geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
        geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);' rudder3';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
        geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;1];
      else
        sup_control_frc = 0;
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
        geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
        geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
        geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
        geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;0];
      end
      stick.IDCAERO1.vert = [stick.IDCAERO1.vert; stick.IDCAERO1.vert(end)+1];
      stick.part.vert = [stick.part.vert(1:Ind), Ind, stick.part.vert(Ind+1:end)]; 
      ny_vert =  round(stick.ny.vert(Ind)*(stick.ptos.horrC2(3,1)-stick.ptos.vert(3,Ind))/dz);
      if isequal(ny_vert, stick.ny.vert(Ind))
        ny_vert= ny_vert - 1;
      elseif ny_vert==0
        ny_vert = 1;
      end
      stick.ny.vert = [stick.ny.vert(1:Ind-1); ny_vert; stick.ny.vert(Ind)-ny_vert; stick.ny.vert(Ind+1:end) ];
      stick.nx.vert = [stick.nx.vert(1:Ind); stick.nx.vert(Ind);stick.nx.vert(Ind+1:end)];
      stick.nx.sup_control.vert = [stick.nx.sup_control.vert(1:Ind);stick.nx.sup_control.vert(Ind:end)];
      if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
        nTOTInd = (stick.nx.vert(Ind) + stick.nx.sup_control.vert(Ind))*stick.ny.vert(Ind);
      else
        nTOTInd = (stick.nx.vert(Ind))*stick.ny.vert(Ind);
      end
      if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2) > 0
        nTOTInd2 = (stick.nx.vert(Ind+1) + stick.nx.sup_control.vert(Ind+1))*stick.ny.vert(Ind+1);
      else
        nTOTInd2 = (stick.nx.vert(Ind+1))*stick.ny.vert(Ind+1);
      end
      stick.nTOT.vert = [stick.nTOT.vert(1:Ind-1); nTOTInd; nTOTInd2; stick.nTOT.vert(Ind+1:end) ];
      T = geo.vtail.CAERO1.chord(Ind+2)/chord;
      TW(1,1) = incidence ;
      TW(2,1) = geo.vtail.CAERO1.incidence(Ind+2);
      SW = geo.vtail.CAERO1.sweepQC(Ind)*pi/180;
      dihed = dihedral *pi/180;
      ox = 0;
      oy = 0;
      oz = 0;

      alpha(1,1) = (geo.vtail.csi_root + (geo.vtail.ni_root - geo.vtail.csi_root)/2)/geo.vtail.CR;
      alpha(2,1) = (geo.vtail.csi_kink + (geo.vtail.ni_kink - geo.vtail.csi_kink)/2)/geo.vtail.CR_kink;
      alpha(3,1) = (geo.vtail.csi_tip + (geo.vtail.ni_tip - geo.vtail.csi_tip)/2)/geo.vtail.CT;

      A(1,1) = alpha(Ind) + (alpha(Ind+1)-alpha(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
      A(2,1) = alpha(Ind+1);

      REFx = ptoHorr(1);
      REFy = ptoHorr(2);
      REFz = ptoHorr(3);
      [dummy, dummy, dummy, pane] = caero_geo(chord, span, T, TW, SW, dihed, ox, oy, oz, A, REFx, REFy, REFz);

      pane(2,:) = [0,0,0,0];

      stick.ptospanel.vert(:,(Ind-1)*4+2:(Ind-1)*4+3) = pane(:,[1,4]);
      stick.ptospanel.vert = [stick.ptospanel.vert(:,1:(Ind-1)*4+4),pane,stick.ptospanel.vert(:,(Ind)*4+1:end)];


      stick.ptos.vert = [stick.ptos.vert(:,1:Ind), ptoHorr , stick.ptos.vert(:,Ind+1:end)];

      n = geo.vtail.CAERO1.n(Ind)*(stick.ptos.horrC2(3,1)-stick.ptos.vert(3,Ind))/dz;
      n_coarse = geo.vtail.CAERO1.n_coarse(Ind)*(stick.ptos.horrC2(3,1)-stick.ptos.vert(3,Ind))/dz;
      nfore = round(n);
      nfore_coarse = round(n_coarse);
      if isequal(nfore, geo.vtail.CAERO1.n(Ind))
        nfore = nfore - 1;
      elseif nfore==0
        nfore = 1;
      end
      if isequal(nfore_coarse, geo.vtail.CAERO1.n_coarse(Ind))
        nfore_coarse = nfore_coarse - 1;
      elseif nfore_coarse==0
        nfore_coarse = 1;
      end
      naft = geo.vtail.CAERO1.n(Ind) - nfore;
      nadd = [nfore; naft];
      geo.vtail.CAERO1.n = [geo.vtail.CAERO1.n(1:Ind-1); nadd; geo.vtail.CAERO1.n(Ind+1:end)];

      naft_coarse = geo.vtail.CAERO1.n_coarse(Ind) - nfore_coarse;
      nadd_coarse = [nfore_coarse; naft_coarse];
      geo.vtail.CAERO1.n_coarse = [geo.vtail.CAERO1.n_coarse(1:Ind-1); nadd_coarse; geo.vtail.CAERO1.n_coarse(Ind+1:end)];
    else
      Dz = stick.ptos.vert(3,abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse)) - ptoHorr(3);
      stick.ptos.horrC2(3,:) = stick.ptos.horrC2(3,:)+Dz;
      stick.ptos.horr(3,:) = stick.ptos.horr(3,:)+Dz;
      stick.ptospanel.horr(3,:) = stick.ptospanel.horr(3,:) + Dz;
      stick.ptospanel.horl(3,:) = stick.ptospanel.horl(3,:) + Dz;
    end
    stick.link.hori = 'vert';
  end
%--------------------------------------------------------------------------------------------------
% 2 CASE
%
% TWIN TAILS and TBOOM
%
elseif isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present

  if isequal(stick.model.vert, 1)
    if stick.ptos.vert(2,1) == stick.ptos.tbooms(2,1)
    % twin tail linked to tailbooms, htail linked to twin tail
      stick.link.hori = 'vert';
      [row, col] = size(stick.ptos.horrC2);
      Ind = [];
      for i = 1:col-1
        if (stick.ptos.vert(2,1) > stick.ptos.horrC2(2,i) && stick.ptos.vert(2,1) < stick.ptos.horrC2(2,i+1))
          % VT is within a segment in HT
          Ind = i;
        end
      end
      if (isempty(Ind))
        % verify if y-coord for HT coincide with one for VT
        IndD = find(stick.ptos.vert(2,1) == stick.ptos.horrC2(2,:));
        % check the vertival position of HT
        if abs(stick.ptos.horrC2(2,end)-stick.ptos.vert(2,1))<=0.001*geo.htail.b
          IndD = 1;
          stick.ptos.vert(2,:) = ones(1,length(stick.ptos.vert(2,:)))*stick.ptos.horrC2(2,end) ;
          stick.ptospanel.vert(2,:) = ones(1,length(stick.ptospanel.vert(2,:)))*stick.ptos.horrC2(2,end) ;
        end
        if isempty(IndD)
          stick.ptos.horrC2(2,end) = stick.ptos.vert(2,end);
          stick.ptospanel.horr(2,end-1) = stick.ptos.vert(2,end);
          stick.ptospanel.horr(2,end-2) = stick.ptos.vert(2,end);
        end
          ptoHorr = stick.ptos.horrC2(:,end); 
      else
        dy = stick.ptos.horrC2(2,Ind+1)-stick.ptos.horrC2(2,Ind);
        % add new point for HT
        xHorVer = stick.ptos.horrC2(1,Ind) + (stick.ptos.horrC2(1,Ind+1)-stick.ptos.horrC2(1,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
        zHorVer = stick.ptos.horrC2(3,Ind) + (stick.ptos.horrC2(3,Ind+1)-stick.ptos.horrC2(3,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
        ptoHorr = [xHorVer;stick.ptos.vert(2,1);zHorVer];
        if isempty(find(abs(stick.ptos.horrC2(2,:)-ptoHorr(2))<=0.02*geo.htail.b/sum(geo.htail.CAERO1.n_coarse),1))
          chord = geo.htail.CAERO1.chord(Ind) + (geo.htail.CAERO1.chord(Ind+1)-geo.htail.CAERO1.chord(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          geo.htail.CAERO1.chord = [geo.htail.CAERO1.chord(1:Ind) ; chord; geo.htail.CAERO1.chord(Ind+1:end)];
          dihedral = geo.htail.CAERO1.dihedral(Ind);
          geo.htail.CAERO1.dihedral = [geo.htail.CAERO1.dihedral(1:Ind); dihedral; geo.htail.CAERO1.dihedral(Ind+1:end)];
          geo.htail.CAERO1.span(Ind) = (ptoHorr(2)-stick.ptos.horrC2(2,Ind)) / cos(deg2rad(geo.htail.CAERO1.dihedral(Ind)));
          span = (stick.ptos.horrC2(2,Ind+1)-ptoHorr(2)) / cos(deg2rad(geo.htail.CAERO1.dihedral(Ind)));
          geo.htail.CAERO1.span = [geo.htail.CAERO1.span(1:Ind);span;geo.htail.CAERO1.span(Ind+1:end)];
          geo.htail.CAERO1.taper = [geo.htail.CAERO1.taper(1:Ind-1);chord/geo.htail.CAERO1.chord(Ind); geo.htail.CAERO1.chord(Ind+2)/chord ;geo.htail.CAERO1.taper(Ind+1:end)];
          geo.htail.CAERO1.sweepLE = [geo.htail.CAERO1.sweepLE(1:Ind-1); geo.htail.CAERO1.sweepLE(Ind);geo.htail.CAERO1.sweepLE(Ind:end)];
          geo.htail.CAERO1.sweepQC = [geo.htail.CAERO1.sweepQC(1:Ind-1);geo.htail.CAERO1.sweepQC(Ind);geo.htail.CAERO1.sweepQC(Ind:end)];
          incidence = geo.htail.CAERO1.incidence(Ind) + (geo.htail.CAERO1.incidence(Ind+1)-geo.htail.CAERO1.incidence(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          geo.htail.CAERO1.incidence = [geo.htail.CAERO1.incidence(1:Ind) ; incidence; geo.htail.CAERO1.incidence(Ind+1:end)];
          stick.part.horr = [stick.part.horr(1:Ind), Ind, stick.part.horr(Ind+1:end)]; 
          geo.htail.CAERO1.airfoil = [geo.htail.CAERO1.airfoil(1:(Ind-1)*2+1);...
          geo.htail.CAERO1.airfoil((Ind-1)*2+1);...
          geo.htail.CAERO1.airfoil((Ind-1)*2+1);...
          geo.htail.CAERO1.airfoil(Ind*2:end)];
          if aircraft.Horizontal_tail.Elevator.present == 1
            sup_control_frc = geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) + (geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1)-geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) )/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
            geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc(1:(Ind-1)*2);sup_control_frc;sup_control_frc;...
            geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1);...
            geo.htail.CAERO1.sup_control.frc((Ind)*2:end)];
            LIND = length(geo.htail.CAERO1.span)-1;
            if geo.htail.twc >0
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'  elev3r';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'  elev3l';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            else
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'  elev3r';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'  elev3l';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            end
            geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ;-1];
          else
            sup_control_frc = 0;
            geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc(1:(Ind-1)*2);sup_control_frc;sup_control_frc;...
            geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1);...
            geo.htail.CAERO1.sup_control.frc((Ind)*2:end)];
            LIND = length(geo.htail.CAERO1.span)-1;
            if geo.htail.twc >0
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            else
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            end
              geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ;0];
          end
          stick.IDCAERO1.horr = [stick.IDCAERO1.horr; stick.IDCAERO1.horr(end)+1];
          ny_hori =  round(stick.ny.hori(Ind)*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind))/dy);
          if isequal(ny_hori, stick.ny.hori(Ind))
            ny_hori= ny_hori - 1;
          elseif ny_hori==0
            ny_hori = 1;
          end
          stick.ny.hori = [stick.ny.hori(1:Ind-1); ny_hori; stick.ny.hori(Ind)-ny_hori; stick.ny.hori(Ind+1:end) ];
          stick.nx.hori = [stick.nx.hori(1:Ind); stick.nx.hori(Ind);stick.nx.hori(Ind+1:end)];
          stick.nx.sup_control.hori = [stick.nx.sup_control.hori(1:Ind);stick.nx.sup_control.hori(Ind:end)];
          if geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
            nTOTInd = (stick.nx.hori(Ind) + stick.nx.sup_control.hori(Ind))*stick.ny.hori(Ind);
          else
            nTOTInd = (stick.nx.hori(Ind))*stick.ny.hori(Ind);
          end
          if geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
            nTOTInd2 = (stick.nx.hori(Ind+1) + stick.nx.sup_control.hori(Ind+1))*stick.ny.hori(Ind+1);
          else
            nTOTInd2 = (stick.nx.hori(Ind+1))*stick.ny.hori(Ind+1);
          end
          stick.nTOT.hori = [stick.nTOT.hori(1:Ind-1); nTOTInd; nTOTInd2; stick.nTOT.hori(Ind+1:end) ];
          T = geo.htail.CAERO1.chord(Ind+2)/chord;
          TW(1,1) = incidence*pi/180 ;
          TW(2,1) = geo.htail.CAERO1.incidence(Ind+2)*pi/180;
          SW = geo.htail.CAERO1.sweepQC(Ind)*pi/180;
          dihed = dihedral *pi/180;
          alpha(1,1) = (geo.htail.csi_root + (geo.htail.ni_root - geo.htail.csi_root)/2)/geo.htail.CR;
          alpha(2,1) = alpha(1,1);
          alpha(3,1) = (geo.htail.csi_kink + (geo.htail.ni_kink - geo.htail.csi_kink)/2)/geo.htail.CR_kink;
          alpha(4,1) = (geo.htail.csi_tip + (geo.htail.ni_tip - geo.htail.csi_tip)/2)/geo.htail.CT;

          A(1,1) = alpha(Ind) + (alpha(Ind+1)-alpha(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          A(2,1) = alpha(Ind+1);

          ptos_horr(1) = stick.ptos.horr(1,Ind) + (stick.ptos.horr(1,Ind+1)-stick.ptos.horr(1,Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          ptos_horr(3) = stick.ptos.horr(3,Ind) + (stick.ptos.horr(3,Ind+1)-stick.ptos.horr(3,Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          ptos_horr(2) = ptoHorr(2);

          stick.ptos.horr = [stick.ptos.horr(:,1:Ind), ptos_horr' , stick.ptos.horr(:,Ind+1:end)];

          REFx = ptoHorr(1);
          REFy = ptoHorr(2);
          REFz = ptoHorr(3);

          ox = ptos_horr(1);
          oy = ptos_horr(2);
          oz = ptos_horr(3);

          [dummy, qc, dummy, pane] = caero_geo(chord, span, T, TW, SW, dihed, ox, oy, oz, A, REFx, REFy, REFz);
          stick.ptospanel.horr(:,(Ind-1)*4+2:(Ind-1)*4+3) = pane(:,[1,4]);
          stick.ptospanel.horr = [stick.ptospanel.horr(:,1:(Ind-1)*4+4),pane,stick.ptospanel.horr(:,(Ind)*4+1:end)];
          stick.ptos.horrC2 = [stick.ptos.horrC2(:,1:Ind), ptoHorr , stick.ptos.horrC2(:,Ind+1:end)];
          n = geo.htail.CAERO1.n(Ind)*(ptoHorr(2)-stick.ptos.horrC2(2,Ind))/dy;
          n_coarse = geo.htail.CAERO1.n_coarse(Ind)*(ptoHorr(2)-stick.ptos.horrC2(2,Ind))/dy;
          nfore = round(n);
          nfore_coarse = round(n_coarse);
          if isequal(nfore, geo.htail.CAERO1.n(Ind))
            nfore = nfore - 1;
          elseif nfore==0
            nfore = 1;
          end
          if isequal(nfore_coarse, geo.htail.CAERO1.n_coarse(Ind))
            nfore_coarse = nfore_coarse - 1;
          elseif nfore_coarse==0
            nfore_coarse = 1;
          end
          naft = geo.htail.CAERO1.n(Ind) - nfore;
          nadd = [nfore; naft];
          geo.htail.CAERO1.n = [geo.htail.CAERO1.n(1:Ind-1); nadd; geo.htail.CAERO1.n(Ind+1:end)];

          naft_coarse = geo.htail.CAERO1.n_coarse(Ind) - nfore_coarse;
          nadd_coarse = [nfore_coarse; naft_coarse];
          geo.htail.CAERO1.n_coarse = [geo.htail.CAERO1.n_coarse(1:Ind-1); nadd_coarse; geo.htail.CAERO1.n_coarse(Ind+1:end)];
          geo.htail.index = zeros(length(geo.htail.CAERO1.n)+1,1);
          geo.htail.index(1) = 1;
          for i = 1 : length(geo.htail.CAERO1.n)
            geo.htail.index(i+1) = geo.htail.index(i)+geo.htail.CAERO1.n(i);
          end
        else
          Dy = stick.ptos.horrC2(2,abs(stick.ptos.horrC2(2,:)-ptoHorr(2))<=0.02*geo.htail.b/sum(geo.htail.CAERO1.n_coarse)) - ptoHorr(2);
          stick.ptos.vert(2,:) = stick.ptos.vert(2,:)+Dy;
          stick.ptospanel.vert(2,:) = stick.ptospanel.vert(2,:) + Dy;
          end
          %         end
          [Y, J] = sort(stick.ptos.horrC2(2,:), 'ascend');
          % Sort points
          stick.ptos.horrC2(1,:) = stick.ptos.horrC2(1,J);
          stick.ptos.horrC2(2,:) = Y;
          stick.ptos.horrC2(3,:) = stick.ptos.horrC2(3,J);
        end
        % Adding the HorVer point of horizontal tail to vertical tail
        [row, col] = size(stick.ptos.vert);
        Ind = [];
        for i = 1:col-1
          if (ptoHorr(3) > stick.ptos.vert(3,i) && ptoHorr(3) < stick.ptos.vert(3,i+1))
          % HT is within a segment in VT
            Ind = i;
          end
        end
        if (isempty(Ind))
          % verify if z-coord for HT coincide with one for VT
          IndD = find(ptoHorr(3) == stick.ptos.vert(3,:));
          % check the vertival position of HT
          if abs(ptoHorr(3)-stick.ptos.vert(3,1))<=0.001*geo.vtail.b
            IndD = 1;
            stick.ptos.vert(3,1) = ptoHorr(3);
          end
          if isempty(IndD)
            if ptoHorr(3) > stick.ptos.vert(3,end)
              stick.ptos.horrC2(3,:) = stick.ptos.horrC2(3,:) - (ptoHorr(3)-stick.ptos.vert(3,end));
              stick.ptospanel.horr(3,:) = stick.ptospanel.horr(3,:)- (ptoHorr(3)-stick.ptos.vert(3,end));
            else
              stick.ptos.horrC2(3,:) = stick.ptos.horrC2(3,:) - (ptoHorr(3)-stick.ptos.vert(3,1));
              stick.ptospanel.horr(3,:) = stick.ptospanel.horr(3,:)- (ptoHorr(3)-stick.ptos.vert(3,1));
            end
          end
        else
          dz = stick.ptos.vert(3,Ind+1)-stick.ptos.vert(3,Ind);
          % add new point for VT
          xHorVer = stick.ptos.vert(1,Ind) + (stick.ptos.vert(1,Ind+1)-stick.ptos.vert(1,Ind))/dz*(-stick.ptos.vert(3,Ind)+zHorVer);
          %             zHorVer = stick.ptos.horrC2(3,Ind) + (stick.ptos.horrC2(3,Ind+1)-stick.ptos.horrC2(3,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
          ptoHorr = [xHorVer;stick.ptos.vert(2,1);zHorVer];
          if isempty(find(abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse),1))
            chord = geo.vtail.CAERO1.chord(Ind) + (geo.vtail.CAERO1.chord(Ind+1)-geo.vtail.CAERO1.chord(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
            geo.vtail.CAERO1.chord = [geo.vtail.CAERO1.chord(1:Ind) ; chord; geo.vtail.CAERO1.chord(Ind+1:end)];

            dihedral = geo.vtail.CAERO1.dihedral(Ind);
            geo.vtail.CAERO1.dihedral = [geo.vtail.CAERO1.dihedral(1:Ind); dihedral; geo.vtail.CAERO1.dihedral(Ind+1:end)];

            geo.vtail.CAERO1.span(Ind) = (ptoHorr(3)-stick.ptos.vert(3,Ind))/cos(deg2rad(geo.htail.CAERO1.dihedral(Ind)));
            span = (stick.ptos.vert(3,Ind+1)-ptoHorr(3))/cos(deg2rad(geo.htail.CAERO1.dihedral(Ind)));
            geo.vtail.CAERO1.span = [geo.vtail.CAERO1.span(1:Ind);span;geo.vtail.CAERO1.span(Ind+1:end)];

            geo.vtail.CAERO1.taper = [geo.vtail.CAERO1.taper(1:Ind-1);chord/geo.vtail.CAERO1.chord(Ind); geo.vtail.CAERO1.chord(Ind+2)/chord ;geo.vtail.CAERO1.taper(Ind+1:end)];

            geo.vtail.CAERO1.sweepLE = [geo.vtail.CAERO1.sweepLE(1:Ind); geo.vtail.CAERO1.sweepLE(Ind);geo.vtail.CAERO1.sweepLE(Ind+1:end)];

            geo.vtail.CAERO1.sweepQC = [geo.vtail.CAERO1.sweepQC(1:Ind);geo.vtail.CAERO1.sweepQC(Ind);geo.vtail.CAERO1.sweepQC(Ind+1:end)];

            incidence = geo.vtail.CAERO1.incidence(Ind) + (geo.vtail.CAERO1.incidence(Ind+1)-geo.vtail.CAERO1.incidence(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
            geo.vtail.CAERO1.incidence = [geo.vtail.CAERO1.incidence(1:Ind) ; incidence; geo.vtail.CAERO1.incidence(Ind+1:end)];
            geo.vtail.CAERO1.airfoil = [geo.vtail.CAERO1.airfoil(1:(Ind-1)*2+2);...
            geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
            geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
            geo.vtail.CAERO1.airfoil(Ind*2+1:end)];
            if aircraft.Vertical_tail.Rudder.present == 1
              sup_control_frc = geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) + (geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2)-geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) )/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
              geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
              geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
              geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
              LIND = length(geo.vtail.CAERO1.span)-1;
              geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);' rudder3';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
              geo.vtail2.CAERO1.sup_control.nme = [geo.vtail2.CAERO1.sup_control.nme(1:Ind,:);' rudder3';geo.vtail2.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'rudder3l';geo.vtail2.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];

              geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;1];
            else
              sup_control_frc = 0;
              geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
              geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
              geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
              geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
              geo.vtail2.CAERO1.sup_control.nme = [geo.vtail2.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.vtail2.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'rudder3l';geo.vtail2.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];

              geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;0];
            end
            stick.IDCAERO1.vert = [stick.IDCAERO1.vert; stick.IDCAERO1.vert(end)+1];
            stick.part.vert = [stick.part.vert(1:Ind), Ind, stick.part.vert(Ind+1:end)]; 
            ny_vert =  round(stick.ny.vert(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz);
            if isequal(ny_vert, stick.ny.vert(Ind))
              ny_vert= ny_vert - 1;
            elseif ny_vert==0
              ny_vert = 1;
            end
            stick.ny.vert = [stick.ny.vert(1:Ind-1); ny_vert; stick.ny.vert(Ind)-ny_vert; stick.ny.vert(Ind+1:end) ];
            stick.nx.vert = [stick.nx.vert(1:Ind); stick.nx.vert(Ind);stick.nx.vert(Ind+1:end)];
            stick.nx.sup_control.vert = [stick.nx.sup_control.vert(1:Ind);stick.nx.sup_control.vert(Ind:end)];
            if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
              nTOTInd = (stick.nx.vert(Ind) + stick.nx.sup_control.vert(Ind))*stick.ny.vert(Ind);
            else
              nTOTInd = (stick.nx.vert(Ind))*stick.ny.vert(Ind);
            end
            if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2) > 0
              nTOTInd2 = (stick.nx.vert(Ind+1) + stick.nx.sup_control.vert(Ind+1))*stick.ny.vert(Ind+1);
            else
              nTOTInd2 = (stick.nx.vert(Ind+1))*stick.ny.vert(Ind+1);
            end
            stick.nTOT.vert = [stick.nTOT.vert(1:Ind-1); nTOTInd; nTOTInd2; stick.nTOT.vert(Ind+1:end) ];
            T = geo.vtail.CAERO1.chord(Ind+2)/chord;
            TW(1,1) = incidence ;
            TW(2,1) = geo.vtail.CAERO1.incidence(Ind+2);
            SW = geo.vtail.CAERO1.sweepQC(Ind)*pi/180;
            dihed = dihedral *pi/180;
            ox = ptoHorr(1);
            oy = ptoHorr(2);
            oz = ptoHorr(3);

            alpha(1,1) = (geo.vtail.csi_root + (geo.vtail.ni_root - geo.vtail.csi_root)/2)/geo.vtail.CR;
            alpha(2,1) = (geo.vtail.csi_kink + (geo.vtail.ni_kink - geo.vtail.csi_kink)/2)/geo.vtail.CR_kink;
            alpha(3,1) = (geo.vtail.csi_tip + (geo.vtail.ni_tip - geo.vtail.csi_tip)/2)/geo.vtail.CT;

            A(1,1) = alpha(Ind) + (alpha(Ind+1)-alpha(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
            A(2,1) = alpha(Ind+1);

            REFx = ptoHorr(1);
            REFy = ptoHorr(2);
            REFz = ptoHorr(3);
            [dummy, dummy, dummy, pane] = caero_geo(chord, span, T, TW, SW, dihed, ox, oy, oz, A, REFx, REFy, REFz);
            stick.ptospanel.vert(:,(Ind-1)*4+2:(Ind-1)*4+3) = pane(:,[1,4]);
            stick.ptospanel.vert = [stick.ptospanel.vert(:,1:(Ind-1)*4+4),pane,stick.ptospanel.vert(:,(Ind)*4+1:end)];
            stick.ptos.vert = [stick.ptos.vert(:,1:Ind), ptoHorr , stick.ptos.vert(:,Ind+1:end)];
            n = geo.vtail.CAERO1.n(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz;
            n_coarse = geo.vtail.CAERO1.n_coarse(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz;
            nfore = round(n);
            nfore_coarse = round(n_coarse);
            if isequal(nfore, geo.vtail.CAERO1.n(Ind))
              nfore = nfore - 1;
            elseif nfore==0
              nfore = 1;
            end
            if isequal(nfore_coarse, geo.vtail.CAERO1.n_coarse(Ind))
              nfore_coarse = nfore_coarse - 1;
            elseif nfore_coarse==0
              nfore_coarse = 1;
            end
            naft = geo.vtail.CAERO1.n(Ind) - nfore;
            nadd = [nfore; naft];
            geo.vtail.CAERO1.n = [geo.vtail.CAERO1.n(1:Ind-1); nadd; geo.vtail.CAERO1.n(Ind+1:end)];

            naft_coarse = geo.vtail.CAERO1.n_coarse(Ind) - nfore_coarse;
            nadd_coarse = [nfore_coarse; naft_coarse];
            geo.vtail.CAERO1.n_coarse = [geo.vtail.CAERO1.n_coarse(1:Ind-1); nadd_coarse; geo.vtail.CAERO1.n_coarse(Ind+1:end)];
          else
            Dz = stick.ptos.vert(3,abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse)) - ptoHorr(3);
            stick.ptos.horrC2(3,:) = stick.ptos.horrC2(3,:) +Dz;
            stick.ptospanel.horr(3,:) = stick.ptospanel.horr(3,:) + Dz;
          end
          %         end
          [Y, J] = sort(stick.ptos.vert(3,:), 'ascend');
          % Sort points
          stick.ptos.vert(1,:) = stick.ptos.vert(1,J);
          stick.ptos.vert(2,:) = stick.ptos.vert(2,J);
          stick.ptos.vert(3,:) = Y;
        end
      else
        % htail linked to tailbooms, vtail linked to htail
        Xhtail = interp1(stick.ptos.horrC2(2,:),stick.ptos.horrC2(1,:),geo.tbooms.y,'linear');
        Zhtail = interp1(stick.ptos.horrC2(2,:),stick.ptos.horrC2(3,:),geo.tbooms.y,'linear');
        [dy,IND] = min(abs(stick.ptos.horrC2(2,:) - geo.tbooms.y));
        [dx,IND2] = min(abs(Xhtail - stick.ptos.tbooms(1,:)));
        if dy <= 0.001*geo.wing.b && dx <= 0.001*geo.wing.b && (abs(Zhtail - stick.ptos.tbooms(3,IND2)))<= 0.001*geo.wing.b
        stick.ptos.tbooms(:,IND2) =  stick.ptos.horrC2(:,IND);
        stick.htail.X =Xhtail;
        else
          stick.htail.X =Xhtail;
          if dx <= 0.2*geo.tbooms.bodl/geo.tbooms.CAERO1.n_coarse
            stick.ptos.tbooms(1,IND2) = Xhtail;
          else
          stick.ptos.tbooms = [stick.ptos.tbooms,[Xhtail;stick.ptos.tbooms(2:3,1)]];
          [Y, J] = sort(stick.ptos.tbooms(1,:), 'ascend');
          % Sort points
          stick.ptos.tbooms(1,:) = Y;
          stick.ptos.tbooms(2,:) = stick.ptos.tbooms(2,J);
          stick.ptos.tbooms(3,:) = stick.ptos.tbooms(3,J);
          end
        end
        stick.link.hori = 'tbooms';
        stick.link.vert = 'hori';
    %   FIND INTERSECTION
        [row, col] = size(stick.ptos.horrC2);
        Ind = [];
        for i = 1:col-1
          if (stick.ptos.vert(2,1) > stick.ptos.horrC2(2,i) && stick.ptos.vert(2,1) < stick.ptos.horrC2(2,i+1))
          % VT is within a segment in HT
            Ind = i;
          end
        end
        if (isempty(Ind))
          % verify if y-coord for HT coincide with one for VT
          IndD = find(stick.ptos.vert(2,1) == stick.ptos.horrC2(2,:));
          % check the vertival position of HT
          if abs(stick.ptos.horrC2(2,end)-stick.ptos.vert(2,1))<=0.001*geo.htail.b
            IndD = 1;
            stick.ptos.vert(2,:) = ones(1,length(stick.ptos.vert(2,:)))*stick.ptos.horrC2(2,end) ;
            stick.ptospanel.vert(2,:) = ones(1,length(stick.ptospanel.vert(2,:)))*stick.ptos.horrC2(2,end) ;
          end
          if isempty(IndD)
            stick.ptos.vert(2,:) = ones(1,length(stick.ptos.vert(2,:)))*stick.ptos.horrC2(2,end) ;
            stick.ptospanel.vert(2,:) = ones(1,length(stick.ptospanel.vert(2,:)))*stick.ptos.horrC2(2,end) ;
          end
            ptoHorr = stick.ptos.horrC2(:,end);
          else
          dy = stick.ptos.horrC2(2,Ind+1)-stick.ptos.horrC2(2,Ind);
          % add new point for HT
          xHorVer = stick.ptos.horrC2(1,Ind) + (stick.ptos.horrC2(1,Ind+1)-stick.ptos.horrC2(1,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
          zHorVer = stick.ptos.horrC2(3,Ind) + (stick.ptos.horrC2(3,Ind+1)-stick.ptos.horrC2(3,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
          ptoHorr = [xHorVer;stick.ptos.vert(2,1);zHorVer];
          if isempty(find(abs(stick.ptos.horrC2(2,:)-ptoHorr(2))<=0.02*geo.htail.b/sum(geo.htail.CAERO1.n_coarse),1))
          chord = geo.htail.CAERO1.chord(Ind) + (geo.htail.CAERO1.chord(Ind+1)-geo.htail.CAERO1.chord(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          geo.htail.CAERO1.chord = [geo.htail.CAERO1.chord(1:Ind) ; chord; geo.htail.CAERO1.chord(Ind+1:end)];

          dihedral = geo.htail.CAERO1.dihedral(Ind);
          geo.htail.CAERO1.dihedral = [geo.htail.CAERO1.dihedral(1:Ind); dihedral; geo.htail.CAERO1.dihedral(Ind+1:end)];

          geo.htail.CAERO1.span(Ind) = (ptoHorr(2)-stick.ptos.horrC2(2,Ind))/cos(deg2rad(geo.htail.CAERO1.dihedral(Ind)));
          span = (stick.ptos.horrC2(2,Ind+1)-ptoHorr(2))/cos(deg2rad(geo.htail.CAERO1.dihedral(Ind)));
          geo.htail.CAERO1.span = [geo.htail.CAERO1.span(1:Ind);span;geo.htail.CAERO1.span(Ind+1:end)];

          geo.htail.CAERO1.taper = [geo.htail.CAERO1.taper(1:Ind-1);chord/geo.htail.CAERO1.chord(Ind); geo.htail.CAERO1.chord(Ind+2)/chord ;geo.htail.CAERO1.taper(Ind+1:end)];

          geo.htail.CAERO1.sweepLE = [geo.htail.CAERO1.sweepLE(1:Ind-1); geo.htail.CAERO1.sweepLE(Ind);geo.htail.CAERO1.sweepLE(Ind:end)];

          geo.htail.CAERO1.sweepQC = [geo.htail.CAERO1.sweepQC(1:Ind-1);geo.htail.CAERO1.sweepQC(Ind);geo.htail.CAERO1.sweepQC(Ind:end)];

          incidence = geo.htail.CAERO1.incidence(Ind) + (geo.htail.CAERO1.incidence(Ind+1)-geo.htail.CAERO1.incidence(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          geo.htail.CAERO1.incidence = [geo.htail.CAERO1.incidence(1:Ind) ; incidence; geo.htail.CAERO1.incidence(Ind+1:end)];


          geo.htail.CAERO1.airfoil = [geo.htail.CAERO1.airfoil(1:(Ind-1)*2+1);...
          geo.htail.CAERO1.airfoil((Ind-1)*2+1);...
          geo.htail.CAERO1.airfoil((Ind-1)*2+1);...
          geo.htail.CAERO1.airfoil(Ind*2:end)];
          if aircraft.Horizontal_tail.Elevator.present == 1
            sup_control_frc = geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) + (geo.htail.CAERO1.sup_control.frc((Ind-1)*2+2)-geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) )/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
            geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc(1:(Ind-1)*2);sup_control_frc;sup_control_frc;...
            geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1);...
            geo.htail.CAERO1.sup_control.frc((Ind)*2:end)];
            LIND = length(geo.htail.CAERO1.span)-1;
            if geo.htail.twc >0
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'  elev3r';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'  elev3l';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            else
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'  elev3r';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'  elev3l';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            end
            geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ;-1];
          else
            sup_control_frc = 0;
            geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc(1:(Ind-1)*2);sup_control_frc;sup_control_frc;...
            geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1);...
            geo.htail.CAERO1.sup_control.frc((Ind)*2:end)];
            LIND = length(geo.htail.CAERO1.span)-1;
            if geo.htail.twc >0
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            else
              geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
            end
            geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ;0];
          end
          stick.IDCAERO1.horr = [stick.IDCAERO1.horr; stick.IDCAERO1.horr(end)+1];
          stick.part.horr = [stick.part.horr(1:Ind), Ind, stick.part.horr(Ind+1:end)]; 
          ny_hori =  round(stick.ny.hori(Ind)*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind))/dy);
          if isequal(ny_hori, stick.ny.hori(Ind))
            ny_hori= ny_hori - 1;
          elseif ny_hori==0
            ny_hori = 1;
          end
          stick.ny.hori = [stick.ny.hori(1:Ind-1); ny_hori; stick.ny.hori(Ind)-ny_hori; stick.ny.hori(Ind+1:end) ];
          stick.nx.hori = [stick.nx.hori(1:Ind); stick.nx.hori(Ind);stick.nx.hori(Ind+1:end)];
          stick.nx.sup_control.hori = [stick.nx.sup_control.hori(1:Ind);stick.nx.sup_control.hori(Ind:end)];
          if geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
            nTOTInd = (stick.nx.hori(Ind) + stick.nx.sup_control.hori(Ind))*stick.ny.hori(Ind);
          else
            nTOTInd = (stick.nx.hori(Ind))*stick.ny.hori(Ind);
          end
          if geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
            nTOTInd2 = (stick.nx.hori(Ind+1) + stick.nx.sup_control.hori(Ind+1))*stick.ny.hori(Ind+1);
          else
            nTOTInd2 = (stick.nx.hori(Ind+1))*stick.ny.hori(Ind+1);
          end
          stick.nTOT.hori = [stick.nTOT.hori(1:Ind-1); nTOTInd; nTOTInd2; stick.nTOT.hori(Ind+1:end) ];
          T = geo.htail.CAERO1.chord(Ind+2)/chord;
          TW(1,1) = incidence*pi/180 ;
          TW(2,1) = geo.htail.CAERO1.incidence(Ind+2)*pi/180;
          SW = geo.htail.CAERO1.sweepQC(Ind)*pi/180;
          dihed = dihedral *pi/180;
          alpha(1,1) = (geo.htail.csi_root + (geo.htail.ni_root - geo.htail.csi_root)/2)/geo.htail.CR;
          alpha(2,1) = alpha(1,1);
          alpha(3,1) = (geo.htail.csi_kink + (geo.htail.ni_kink - geo.htail.csi_kink)/2)/geo.htail.CR_kink;
          alpha(4,1) = (geo.htail.csi_tip + (geo.htail.ni_tip - geo.htail.csi_tip)/2)/geo.htail.CT;

          A(1,1) = alpha(Ind) + (alpha(Ind+1)-alpha(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          A(2,1) = alpha(Ind+1);

          ptos_horr(1) = stick.ptos.horr(1,Ind) + (stick.ptos.horr(1,Ind+1)-stick.ptos.horr(1,Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          ptos_horr(3) = stick.ptos.horr(3,Ind) + (stick.ptos.horr(3,Ind+1)-stick.ptos.horr(3,Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          ptos_horr(2) = ptoHorr(2);

          stick.ptos.horr = [stick.ptos.horr(:,1:Ind), ptos_horr' , stick.ptos.horr(:,Ind+1:end)];

          REFx = ptoHorr(1);
          REFy = ptoHorr(2);
          REFz = ptoHorr(3);

          ox = ptos_horr(1);
          oy = ptos_horr(2);
          oz = ptos_horr(3);

          [dummy, qc, dummy, pane] = caero_geo(chord, span, T, TW, SW, dihed, ox, oy, oz, A, REFx, REFy, REFz);
          stick.ptospanel.horr(:,(Ind-1)*4+2:(Ind-1)*4+3) = pane(:,[1,4]);
          stick.ptospanel.horr = [stick.ptospanel.horr(:,1:(Ind-1)*4+4),pane,stick.ptospanel.horr(:,(Ind)*4+1:end)];
          stick.ptos.horrC2 = [stick.ptos.horrC2(:,1:Ind), ptoHorr , stick.ptos.horrC2(:,Ind+1:end)];
          n = geo.htail.CAERO1.n(Ind)*(ptoHorr(2)-stick.ptos.horrC2(2,Ind))/dy;
          n_coarse = geo.htail.CAERO1.n_coarse(Ind)*(ptoHorr(2)-stick.ptos.horrC2(2,Ind))/dy;
          nfore = round(n);
          nfore_coarse = round(n_coarse);
          if isequal(nfore, geo.htail.CAERO1.n(Ind))
            nfore = nfore - 1;
          elseif nfore==0
            nfore = 1;
          end
          if isequal(nfore_coarse, geo.htail.CAERO1.n_coarse(Ind))
            nfore_coarse = nfore_coarse - 1;
          elseif nfore_coarse==0
            nfore_coarse = 1;
          end
          naft = geo.htail.CAERO1.n(Ind) - nfore;
          nadd = [nfore; naft];
          geo.htail.CAERO1.n = [geo.htail.CAERO1.n(1:Ind-1); nadd; geo.htail.CAERO1.n(Ind+1:end)];

          naft_coarse = geo.htail.CAERO1.n_coarse(Ind) - nfore_coarse;
          nadd_coarse = [nfore_coarse; naft_coarse];
          geo.htail.CAERO1.n_coarse = [geo.htail.CAERO1.n_coarse(1:Ind-1); nadd_coarse; geo.htail.CAERO1.n_coarse(Ind+1:end)];
          geo.htail.index = zeros(length(geo.htail.CAERO1.n)+1,1);
          geo.htail.index(1) = 1;
          for i = 1 : length(geo.htail.CAERO1.n)
            geo.htail.index(i+1) = geo.htail.index(i)+geo.htail.CAERO1.n(i);
          end
        else
          Dy = stick.ptos.horrC2(2,abs(stick.ptos.horrC2(2,:)-ptoHorr(2))<=0.02*geo.htail.b/sum(geo.htail.CAERO1.n_coarse)) - ptoHorr(2);
          stick.ptos.vert(2,:) = stick.ptos.vert(3,:)+Dy;
          stick.ptospanel.vert(3,:) = stick.ptospanel.vert(3,:) + Dy;
        end
        %         end
        [Y, J] = sort(stick.ptos.horrC2(2,:), 'ascend');
        % Sort points
        stick.ptos.horrC2(1,:) = stick.ptos.horrC2(1,J);
        stick.ptos.horrC2(2,:) = Y;
        stick.ptos.horrC2(3,:) = stick.ptos.horrC2(3,J);
      end
      % Adding the HorVer point of horizontal tail to vertical tail
      [row, col] = size(stick.ptos.vert);
      Ind = [];
      for i = 1:col-1
        if (ptoHorr(3) > stick.ptos.vert(3,i) && ptoHorr(3) < stick.ptos.vert(3,i+1))
        % HT is within a segment in VT
          Ind = i;
        end
      end
      if (isempty(Ind))
        % verify if z-coord for HT coincide with one for VT
        IndD = find(ptoHorr(3) == stick.ptos.vert(3,:));
        % check the vertival position of HT
        if abs(ptoHorr(3)-stick.ptos.vert(3,1))<=0.001*geo.vtail.b
          IndD = 1;
          stick.ptos.vert(3,1) = ptoHorr(3);
        end
        if isempty(IndD)
          stick.ptos.vert(3,:) = stick.ptos.vert(3,:)-(stick.ptos.vert(3,1)- ptoHorr(3));
          stick.ptospanel.vert(3,:) = stick.ptospanel.vert(3,:)-(stick.ptospanel.vert(3,1)- ptoHorr(3));
        end
      else
        dz = stick.ptos.vert(3,Ind+1)-stick.ptos.vert(3,Ind);
        % add new point for VT
        xHorVer = stick.ptos.vert(1,Ind) + (stick.ptos.vert(1,Ind+1)-stick.ptos.vert(1,Ind))/dz*(-stick.ptos.vert(3,Ind)+zHorVer);
        ptoHorr = [xHorVer;stick.ptos.vert(2,1);zHorVer];
        if isempty(find(abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse),1))
          chord = geo.vtail.CAERO1.chord(Ind) + (geo.vtail.CAERO1.chord(Ind+1)-geo.vtail.CAERO1.chord(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
          geo.vtail.CAERO1.chord = [geo.vtail.CAERO1.chord(1:Ind) ; chord; geo.vtail.CAERO1.chord(Ind+1:end)];

          dihedral = geo.vtail.CAERO1.dihedral(Ind);
          geo.vtail.CAERO1.dihedral = [geo.vtail.CAERO1.dihedral(1:Ind); dihedral; geo.vtail.CAERO1.dihedral(Ind+1:end)];

          geo.vtail.CAERO1.span(Ind) = ptoHorr(3)-stick.ptos.vert(3,Ind);
          span = stick.ptos.vert(3,Ind+1)-ptoHorr(3);
          geo.vtail.CAERO1.span = [geo.vtail.CAERO1.span(1:Ind);span;geo.vtail.CAERO1.span(Ind+1:end)];

          geo.vtail.CAERO1.taper = [geo.vtail.CAERO1.taper(1:Ind-1);chord/geo.vtail.CAERO1.chord(Ind); geo.vtail.CAERO1.chord(Ind+2)/chord ;geo.vtail.CAERO1.taper(Ind+1:end)];

          geo.vtail.CAERO1.sweepLE = [geo.vtail.CAERO1.sweepLE(1:Ind); geo.vtail.CAERO1.sweepLE(Ind);geo.vtail.CAERO1.sweepLE(Ind+1:end)];

          geo.vtail.CAERO1.sweepQC = [geo.vtail.CAERO1.sweepQC(1:Ind);geo.vtail.CAERO1.sweepQC(Ind);geo.vtail.CAERO1.sweepQC(Ind+1:end)];

          incidence = geo.vtail.CAERO1.incidence(Ind) + (geo.vtail.CAERO1.incidence(Ind+1)-geo.vtail.CAERO1.incidence(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
          geo.vtail.CAERO1.incidence = [geo.vtail.CAERO1.incidence(1:Ind) ; incidence; geo.vtail.CAERO1.incidence(Ind+1:end)];


          geo.vtail.CAERO1.airfoil = [geo.vtail.CAERO1.airfoil(1:(Ind-1)*2+2);...
          geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
          geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
          geo.vtail.CAERO1.airfoil(Ind*2+1:end)];
          if aircraft.Vertical_tail.Rudder.present == 1
            sup_control_frc = geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) + (geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2)-geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) )/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
            geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
            geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
            geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
            LIND = length(geo.vtail.CAERO1.span)-1;
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);' rudder3';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
            geo.vtail2.CAERO1.sup_control.nme = [geo.vtail2.CAERO1.sup_control.nme(1:Ind,:);' rudder3';geo.vtail2.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'rudder3l';geo.vtail2.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];

            geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;1];
          else
            sup_control_frc = 0;
            geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
            geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
            geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
            LIND = length(geo.vtail.CAERO1.span)-1;
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
            geo.vtail2.CAERO1.sup_control.nme = [geo.vtail2.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.vtail2.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.vtail2.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];

            geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;0];
          end
          stick.IDCAERO1.vert = [stick.IDCAERO1.vert; stick.IDCAERO1.vert(end)+1];
          stick.part.vert = [stick.part.vert(1:Ind), Ind, stick.part.vert(Ind+1:end)]; 
          ny_vert =  round(stick.ny.vert(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz);
          if isequal(ny_vert, stick.ny.vert(Ind))
            ny_vert= ny_vert - 1;
          elseif ny_vert==0
            ny_vert = 1;
          end
          stick.ny.vert = [stick.ny.vert(1:Ind-1); ny_vert; stick.ny.vert(Ind)-ny_vert; stick.ny.vert(Ind+1:end) ];
          stick.nx.vert = [stick.nx.vert(1:Ind); stick.nx.vert(Ind);stick.nx.vert(Ind+1:end)];
          stick.nx.sup_control.vert = [stick.nx.sup_control.vert(1:Ind);stick.nx.sup_control.vert(Ind:end)];
          if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
            nTOTInd = (stick.nx.vert(Ind) + stick.nx.sup_control.vert(Ind))*stick.ny.vert(Ind);
          else
            nTOTInd = (stick.nx.vert(Ind))*stick.ny.vert(Ind);
          end
          if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2) > 0
            nTOTInd2 = (stick.nx.vert(Ind+1) + stick.nx.sup_control.vert(Ind+1))*stick.ny.vert(Ind+1);
          else
            nTOTInd2 = (stick.nx.vert(Ind+1))*stick.ny.vert(Ind+1);
          end
          stick.nTOT.vert = [stick.nTOT.vert(1:Ind-1); nTOTInd; nTOTInd2; stick.nTOT.vert(Ind+1:end) ];
          T = geo.vtail.CAERO1.chord(Ind+2)/chord;
          TW(1,1) = incidence ;
          TW(2,1) = geo.vtail.CAERO1.incidence(Ind+2);
          SW = geo.vtail.CAERO1.sweepQC(Ind)*pi/180;
          dihed = dihedral *pi/180;
          ox = ptoHorr(1);
          oy = ptoHorr(2);
          oz = ptoHorr(3);

          alpha(1,1) = (geo.vtail.csi_root + (geo.vtail.ni_root - geo.vtail.csi_root)/2)/geo.vtail.CR;
          alpha(2,1) = (geo.vtail.csi_kink + (geo.vtail.ni_kink - geo.vtail.csi_kink)/2)/geo.vtail.CR_kink;
          alpha(3,1) = (geo.vtail.csi_tip + (geo.vtail.ni_tip - geo.vtail.csi_tip)/2)/geo.vtail.CT;

          A(1,1) = alpha(Ind) + (alpha(Ind+1)-alpha(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
          A(2,1) = alpha(Ind+1);

          REFx = ptoHorr(1);
          REFy = ptoHorr(2);
          REFz = ptoHorr(3);
          [dummy, dummy, dummy, pane] = caero_geo(chord, span, T, TW, SW, dihed, ox, oy, oz, A, REFx, REFy, REFz);
          stick.ptospanel.vert(:,(Ind-1)*4+2:(Ind-1)*4+3) = pane(:,[1,4]);
          stick.ptospanel.vert = [stick.ptospanel.vert(:,1:(Ind-1)*4+4),pane,stick.ptospanel.vert(:,(Ind)*4+1:end)];
          stick.ptos.vert = [stick.ptos.vert(:,1:Ind), ptoHorr , stick.ptos.vert(:,Ind+1:end)];
          n = geo.vtail.CAERO1.n(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz;
          n_coarse = geo.vtail.CAERO1.n_coarse(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz;
          nfore = round(n);
          nfore_coarse = round(n_coarse);
          if isequal(nfore, geo.vtail.CAERO1.n(Ind))
            nfore = nfore - 1;
          elseif nfore==0
            nfore = 1;
          end
          if isequal(nfore_coarse, geo.vtail.CAERO1.n_coarse(Ind))
            nfore_coarse = nfore_coarse - 1;
          elseif nfore_coarse==0
            nfore_coarse = 1;
          end
          naft = geo.vtail.CAERO1.n(Ind) - nfore;
          nadd = [nfore; naft];
          geo.vtail.CAERO1.n = [geo.vtail.CAERO1.n(1:Ind-1); nadd; geo.vtail.CAERO1.n(Ind+1:end)];

          naft_coarse = geo.vtail.CAERO1.n_coarse(Ind) - nfore_coarse;
          nadd_coarse = [nfore_coarse; naft_coarse];
          geo.vtail.CAERO1.n_coarse = [geo.vtail.CAERO1.n_coarse(1:Ind-1); nadd_coarse; geo.vtail.CAERO1.n_coarse(Ind+1:end)];
        else
          Dz = stick.ptos.vert(3,abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse)) - ptoHorr(3);
          stick.ptos.vert(3,:) = stick.ptos.vert(3,:)-Dz;
          stick.ptospanel.stick.ptos.vert(3,:) = stick.ptospanel.stick.ptos.vert(3,:) - Dz;
        end
        %         end
        [Y, J] = sort(stick.ptos.vert(3,:), 'ascend');
        % Sort points
        stick.ptos.vert(1,:) = stick.ptos.vert(1,J);
        stick.ptos.vert(2,:) = stick.ptos.vert(2,J);
        stick.ptos.vert(3,:) = Y;
        stick.link.vert = 'hori';
      end
    end
  else % no twin present. htail linked to tboom
%
    if (max(stick.ptos.horrC2(2,:)) < geo.tbooms.y)
      Xhtail = stick.ptos.horrC2(1,end);
      point2add = [Xhtail; stick.ptos.tbooms(2:3,1)];
      pos = find(Xhtail < stick.ptos.tbooms(1,:));
      if ~isempty(pos)
        stick.ptos.tbooms = [stick.ptos.tbooms(:,1:pos), point2add, stick.ptos.tbooms(:,pos+1:end)];
        stick.htail.X = Xhtail;
      else
        stick.htail.X = stick.ptos.tbooms(1,end);
      end
    else
      Xhtail = interp1(stick.ptos.horrC2(2,:),stick.ptos.horrC2(1,:),geo.tbooms.y,'linear');
      point2add = [Xhtail; stick.ptos.tbooms(2:3,1)];
      pos = find(Xhtail < stick.ptos.tbooms(1,:));
      if ~isempty(pos)
        stick.ptos.tbooms = [stick.ptos.tbooms(:,1:pos), point2add, stick.ptos.tbooms(:,pos+1:end)];
      end
      stick.htail.X = Xhtail;
    end
    stick.link.hori = 'tbooms';
%
  end
%--------------------------------------------------------------------------------------------------
% 3 CASE: a) no vertical tail 
%         b) TWIN tail without booms
%
else
  ptoz = interp1(stick.ptos.fuse(1,:), stick.ptos.fuse(3,:), stick.ptos.horrC2(1,1));
  ptoHorrFuse = [stick.ptos.horrC2(1,1); 0; ptoz];
  % fuselage/HT connection point
  if isempty( find( ptoHorrFuse(1)==stick.ptos.fuse(1,:) ,1) )
  %         check the distance between ptofw and other points, if it is to
  %         near to another one it will sobstitute by ptos
    if isempty(find(abs(stick.ptos.fuse(1,:)-ptoHorrFuse(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse,1))
      stick.ptos.fuse = [stick.ptos.fuse, ptoHorrFuse];
    else
      stick.ptos.fuse(:,abs(stick.ptos.fuse(1,:)-ptoHorrFuse(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse) = ptoHorrFuse;
    end
  end
  [Y, J] = sort(stick.ptos.fuse(1,:), 'ascend');
  % Sort points
  stick.ptos.fuse(1,:) = Y;
  stick.ptos.fuse(2,:) = stick.ptos.fuse(2,J);
  stick.ptos.fuse(3,:) = stick.ptos.fuse(3,J);
  stick.link.hori = 'fuse';
% FIND INTERSECTION
  if (aircraft.Vertical_tail.present && strcmp(geo.htail.intersect,'vert'))
    [row, col] = size(stick.ptos.horrC2);
    %
    Ind = [];
    for i = 1:col-1
      if (stick.ptos.vert(2,1) > stick.ptos.horrC2(2,i) && stick.ptos.vert(2,1) < stick.ptos.horrC2(2,i+1))
      % VT is within a segment in HT
        Ind = i;
      end
    end
    if (isempty(Ind))
    % verify if y-coord for HT coincide with one for VT
      IndD = find(stick.ptos.vert(2,1) == stick.ptos.horrC2(2,:));
      % check the vertival position of HT
      if abs(stick.ptos.horrC2(2,end)-stick.ptos.vert(2,1))<=0.001*geo.htail.b
        IndD = 1;
        stick.ptos.vert(2,:) = ones(1,length(stick.ptos.vert(2,:)))*stick.ptos.horrC2(2,end) ;
        stick.ptospanel.vert(2,:) = ones(1,length(stick.ptospanel.vert(2,:)))*stick.ptos.horrC2(2,end) ;
      end
      if isempty(IndD)
        stick.ptos.vert(2,:) = ones(1,length(stick.ptos.vert(2,:)))*stick.ptos.horrC2(2,end) ;
        stick.ptospanel.vert(2,:) = ones(1,length(stick.ptospanel.vert(2,:)))*stick.ptos.horrC2(2,end) ;
      end
      ptoHorr = stick.ptos.horrC2(:,end);
    else
      dy = stick.ptos.horrC2(2,Ind+1)-stick.ptos.horrC2(2,Ind);
      % add new point for HT
      xHorVer = stick.ptos.horrC2(1,Ind) + (stick.ptos.horrC2(1,Ind+1)-stick.ptos.horrC2(1,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
      zHorVer = stick.ptos.horrC2(3,Ind) + (stick.ptos.horrC2(3,Ind+1)-stick.ptos.horrC2(3,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
      ptoHorr = [xHorVer;stick.ptos.vert(2,1);zHorVer];
      if isempty(find(abs(stick.ptos.horrC2(2,:)-ptoHorr(2))<=0.02*geo.htail.b/sum(geo.htail.CAERO1.n_coarse),1))
        chord = geo.htail.CAERO1.chord(Ind) + (geo.htail.CAERO1.chord(Ind+1)-geo.htail.CAERO1.chord(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
        geo.htail.CAERO1.chord = [geo.htail.CAERO1.chord(1:Ind) ; chord; geo.htail.CAERO1.chord(Ind+1:end)];

        dihedral = geo.htail.CAERO1.dihedral(Ind);
        geo.htail.CAERO1.dihedral = [geo.htail.CAERO1.dihedral(1:Ind); dihedral; geo.htail.CAERO1.dihedral(Ind+1:end)];

        geo.htail.CAERO1.span(Ind) = ptoHorr(2)-stick.ptos.horrC2(2,Ind);
        span = stick.ptos.horrC2(2,Ind+1)-ptoHorr(2);
        geo.htail.CAERO1.span = [geo.htail.CAERO1.span(1:Ind);span;geo.htail.CAERO1.span(Ind+1:end)];

        geo.htail.CAERO1.taper = [geo.htail.CAERO1.taper(1:Ind-1);chord/geo.htail.CAERO1.chord(Ind); geo.htail.CAERO1.chord(Ind+2)/chord ;geo.htail.CAERO1.taper(Ind+1:end)];

        geo.htail.CAERO1.sweepLE = [geo.htail.CAERO1.sweepLE(1:Ind-1); geo.htail.CAERO1.sweepLE(Ind);geo.htail.CAERO1.sweepLE(Ind:end)];

        geo.htail.CAERO1.sweepQC = [geo.htail.CAERO1.sweepQC(1:Ind-1);geo.htail.CAERO1.sweepQC(Ind);geo.htail.CAERO1.sweepQC(Ind:end)];

        incidence = geo.htail.CAERO1.incidence(Ind) + (geo.htail.CAERO1.incidence(Ind+1)-geo.htail.CAERO1.incidence(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
        geo.htail.CAERO1.incidence = [geo.htail.CAERO1.incidence(1:Ind) ; incidence; geo.htail.CAERO1.incidence(Ind+1:end)];


        geo.htail.CAERO1.airfoil = [geo.htail.CAERO1.airfoil(1:(Ind-1)*2+1);...
        geo.htail.CAERO1.airfoil((Ind-1)*2+1);...
        geo.htail.CAERO1.airfoil((Ind-1)*2+1);...
        geo.htail.CAERO1.airfoil(Ind*2:end)];
        if aircraft.Horizontal_tail.Elevator.present == 1
          sup_control_frc = geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) + (geo.htail.CAERO1.sup_control.frc((Ind-1)*2+2)-geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) )/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
          geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc(1:(Ind-1)*2);sup_control_frc;sup_control_frc;...
          geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1);...
          geo.htail.CAERO1.sup_control.frc((Ind)*2:end)];
          LIND = length(geo.htail.CAERO1.span)-1;
          if geo.htail.twc >0
            geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'  elev3r';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'  elev3l';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
          else
            geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'  elev3r';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'  elev3l';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
          end
            geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ;-1];
        else
          sup_control_frc =0;
          geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc(1:(Ind-1)*2);sup_control_frc;sup_control_frc;...
          geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1);...
          geo.htail.CAERO1.sup_control.frc((Ind)*2:end)];
          LIND = length(geo.htail.CAERO1.span)-1;
          if geo.htail.twc >0
            geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
          else
            geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.htail.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];
          end
          geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ;0];
        end
        stick.IDCAERO1.horr = [stick.IDCAERO1.horr; stick.IDCAERO1.horr(end)+1];
        stick.part.horr = [stick.part.horr(1:Ind), Ind, stick.part.horr(Ind+1:end)]; 
        ny_hori =  round(stick.ny.hori(Ind)*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind))/dy);
        if isequal(ny_hori, stick.ny.hori(Ind))
          ny_hori= ny_hori - 1;
        elseif ny_hori==0
          ny_hori = 1;
        end
        stick.ny.hori = [stick.ny.hori(1:Ind-1); ny_hori; stick.ny.hori(Ind)-ny_hori; stick.ny.hori(Ind+1:end) ];
        stick.nx.hori = [stick.nx.hori(1:Ind); stick.nx.hori(Ind);stick.nx.hori(Ind+1:end)];
        stick.nx.sup_control.hori = [stick.nx.sup_control.hori(1:Ind);stick.nx.sup_control.hori(Ind:end)];
        if geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
          nTOTInd = (stick.nx.hori(Ind) + stick.nx.sup_control.hori(Ind))*stick.ny.hori(Ind);
        else
          nTOTInd = (stick.nx.hori(Ind))*stick.ny.hori(Ind);
        end
        if geo.htail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
          nTOTInd2 = (stick.nx.hori(Ind+1) + stick.nx.sup_control.hori(Ind+1))*stick.ny.hori(Ind+1);
        else
          nTOTInd2 = (stick.nx.hori(Ind+1))*stick.ny.hori(Ind+1);
        end
        stick.nTOT.hori = [stick.nTOT.hori(1:Ind-1); nTOTInd; nTOTInd2; stick.nTOT.hori(Ind+1:end) ];
        T = geo.htail.CAERO1.chord(Ind+2)/chord;
        TW(1,1) = incidence*pi/180 ;
        TW(2,1) = geo.htail.CAERO1.incidence(Ind+2)*pi/180;
        SW = geo.htail.CAERO1.sweepQC(Ind)*pi/180;
        dihed = dihedral *pi/180;
        alpha(1,1) = (geo.htail.csi_root + (geo.htail.ni_root - geo.htail.csi_root)/2)/geo.htail.CR;
        alpha(2,1) = alpha(1,1);
        alpha(3,1) = (geo.htail.csi_kink + (geo.htail.ni_kink - geo.htail.csi_kink)/2)/geo.htail.CR_kink;
        alpha(4,1) = (geo.htail.csi_tip + (geo.htail.ni_tip - geo.htail.csi_tip)/2)/geo.htail.CT;

        A(1,1) = alpha(Ind) + (alpha(Ind+1)-alpha(Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
        A(2,1) = alpha(Ind+1);

        ptos_horr(1) = stick.ptos.horr(1,Ind) + (stick.ptos.horr(1,Ind+1)-stick.ptos.horr(1,Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
        ptos_horr(3) = stick.ptos.horr(3,Ind) + (stick.ptos.horr(3,Ind+1)-stick.ptos.horr(3,Ind))/dy*(ptoHorr(2)-stick.ptos.horrC2(2,Ind));
        ptos_horr(2) = ptoHorr(2);

        stick.ptos.horr = [stick.ptos.horr(:,1:Ind), ptos_horr' , stick.ptos.horr(:,Ind+1:end)];

        REFx = ptoHorr(1);
        REFy = ptoHorr(2);
        REFz = ptoHorr(3);

        ox = ptos_horr(1);
        oy = ptos_horr(2);
        oz = ptos_horr(3);

        [dummy, qc, dummy, pane] = caero_geo(chord, span, T, TW, SW, dihed, ox, oy, oz, A, REFx, REFy, REFz);
        stick.ptospanel.horr(:,(Ind-1)*4+2:(Ind-1)*4+3) = pane(:,[1,4]);
        stick.ptospanel.horr = [stick.ptospanel.horr(:,1:(Ind-1)*4+4),pane,stick.ptospanel.horr(:,(Ind)*4+1:end)];
        stick.ptos.horrC2 = [stick.ptos.horrC2(:,1:Ind), ptoHorr , stick.ptos.horrC2(:,Ind+1:end)];
        n = geo.htail.CAERO1.n(Ind)*(ptoHorr(2)-stick.ptos.horrC2(2,Ind))/dy;
        n_coarse = geo.htail.CAERO1.n_coarse(Ind)*(ptoHorr(2)-stick.ptos.horrC2(2,Ind))/dy;
        nfore = round(n);
        nfore_coarse = round(n_coarse);
        if isequal(nfore, geo.htail.CAERO1.n(Ind))
          nfore = nfore - 1;
        elseif nfore==0
          nfore = 1;
        end
        if isequal(nfore_coarse, geo.htail.CAERO1.n_coarse(Ind))
          nfore_coarse = nfore_coarse - 1;
        elseif nfore_coarse==0
          nfore_coarse = 1;
        end
        naft = geo.htail.CAERO1.n(Ind) - nfore;
        nadd = [nfore; naft];
        geo.htail.CAERO1.n = [geo.htail.CAERO1.n(1:Ind-1); nadd; geo.htail.CAERO1.n(Ind+1:end)];

        naft_coarse = geo.htail.CAERO1.n_coarse(Ind) - nfore_coarse;
        nadd_coarse = [nfore_coarse; naft_coarse];
        geo.htail.CAERO1.n_coarse = [geo.htail.CAERO1.n_coarse(1:Ind-1); nadd_coarse; geo.htail.CAERO1.n_coarse(Ind+1:end)];
        geo.htail.index = zeros(length(geo.htail.CAERO1.n)+1,1);
        geo.htail.index(1) = 1;
        for i = 1 : length(geo.htail.CAERO1.n)
          geo.htail.index(i+1) = geo.htail.index(i)+geo.htail.CAERO1.n(i);
        end
      else
        Dy = stick.ptos.horrC2(2,abs(stick.ptos.horrC2(2,:)-ptoHorr(2))<=0.02*geo.htail.b/sum(geo.htail.CAERO1.n_coarse)) - ptoHorr(2);
        stick.ptos.vert(2,:) = stick.ptos.vert(3,:)+Dy;
        stick.ptospanel.vert(3,:) = stick.ptospanel.vert(3,:) + Dy;
        end
        %         end
        [Y, J] = sort(stick.ptos.horrC2(2,:), 'ascend');
        % Sort points
        stick.ptos.horrC2(1,:) = stick.ptos.horrC2(1,J);
        stick.ptos.horrC2(2,:) = Y;
        stick.ptos.horrC2(3,:) = stick.ptos.horrC2(3,J);
      end
      % Adding the HorVer point of horizontal tail to vertical tail
      [row, col] = size(stick.ptos.vert);
      Ind = [];
      for i = 1:col-1
        if (ptoHorr(3) > stick.ptos.vert(3,i) && ptoHorr(3) < stick.ptos.vert(3,i+1))
        % HT is within a segment in VT
          Ind = i;
        end
      end
      if (isempty(Ind))
        % verify if z-coord for HT coincide with one for VT
        IndD = find(ptoHorr(3) == stick.ptos.vert(3,:));
        % check the vertival position of HT
        if abs(ptoHorr(3)-stick.ptos.vert(3,1))<=0.001*geo.vtail.b
          IndD = 1;
          stick.ptos.vert(3,1) = ptoHorr(3);
        end
        if isempty(IndD)
          stick.ptos.vert(3,:) = stick.ptos.vert(3,:)-(stick.ptos.vert(3,1)- ptoHorr(3));
          stick.ptospanel.vert(3,:) = stick.ptospanel.vert(3,:)-(stick.ptospanel.vert(3,1)- ptoHorr(3));
        end
        stick.link.vert = 'hori';
      else
        dz = stick.ptos.vert(3,Ind+1)-stick.ptos.vert(3,Ind);
        % add new point for VT
        xHorVer = stick.ptos.vert(1,Ind) + (stick.ptos.vert(1,Ind+1)-stick.ptos.vert(1,Ind))/dz*(-stick.ptos.vert(3,Ind)+zHorVer);
        %             zHorVer = stick.ptos.horrC2(3,Ind) + (stick.ptos.horrC2(3,Ind+1)-stick.ptos.horrC2(3,Ind))/dy*(stick.ptos.vert(2,1)-stick.ptos.horrC2(2,Ind));
        ptoHorr = [xHorVer;stick.ptos.vert(2,1);zHorVer];
        if isempty(find(abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse),1))
          chord = geo.vtail.CAERO1.chord(Ind) + (geo.vtail.CAERO1.chord(Ind+1)-geo.vtail.CAERO1.chord(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
          geo.vtail.CAERO1.chord = [geo.vtail.CAERO1.chord(1:Ind) ; chord; geo.vtail.CAERO1.chord(Ind+1:end)];

          dihedral = geo.vtail.CAERO1.dihedral(Ind);
          geo.vtail.CAERO1.dihedral = [geo.vtail.CAERO1.dihedral(1:Ind); dihedral; geo.vtail.CAERO1.dihedral(Ind+1:end)];

          geo.vtail.CAERO1.span(Ind) = ptoHorr(3)-stick.ptos.vert(3,Ind);
          span = stick.ptos.vert(3,Ind+1)-ptoHorr(3);
          geo.vtail.CAERO1.span = [geo.vtail.CAERO1.span(1:Ind);span;geo.vtail.CAERO1.span(Ind+1:end)];

          geo.vtail.CAERO1.taper = [geo.vtail.CAERO1.taper(1:Ind-1);chord/geo.vtail.CAERO1.chord(Ind); geo.vtail.CAERO1.chord(Ind+2)/chord ;geo.vtail.CAERO1.taper(Ind+1:end)];

          geo.vtail.CAERO1.sweepLE = [geo.vtail.CAERO1.sweepLE(1:Ind); geo.vtail.CAERO1.sweepLE(Ind);geo.vtail.CAERO1.sweepLE(Ind+1:end)];

          geo.vtail.CAERO1.sweepQC = [geo.vtail.CAERO1.sweepQC(1:Ind);geo.vtail.CAERO1.sweepQC(Ind);geo.vtail.CAERO1.sweepQC(Ind+1:end)];

          incidence = geo.vtail.CAERO1.incidence(Ind) + (geo.vtail.CAERO1.incidence(Ind+1)-geo.vtail.CAERO1.incidence(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
          geo.vtail.CAERO1.incidence = [geo.vtail.CAERO1.incidence(1:Ind) ; incidence; geo.vtail.CAERO1.incidence(Ind+1:end)];
          geo.vtail.CAERO1.airfoil = [geo.vtail.CAERO1.airfoil(1:(Ind-1)*2+2);...
          geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
          geo.vtail.CAERO1.airfoil((Ind-1)*2+2);...
          geo.vtail.CAERO1.airfoil(Ind*2+1:end)];
          if aircraft.Vertical_tail.Rudder.present == 1
            sup_control_frc = geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) + (geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2)-geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) )/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
            geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
            geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
            geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
            LIND = length(geo.vtail.CAERO1.span)-1;
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);' rudder3';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
            geo.vtail2.CAERO1.sup_control.nme = [geo.vtail2.CAERO1.sup_control.nme(1:Ind,:);' rudder3';geo.vtail2.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'rudder3l';geo.vtail2.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];

            geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;1];
          else
            sup_control_frc =0;
            geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc(1:(Ind-1)*2+1);sup_control_frc;sup_control_frc;...
            geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2);...
            geo.vtail.CAERO1.sup_control.frc((Ind)*2+1:end)];
            LIND = length(geo.vtail.CAERO1.span)-1;
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.vtail.CAERO1.sup_control.nme(Ind+1:end,:)];
            geo.vtail2.CAERO1.sup_control.nme = [geo.vtail2.CAERO1.sup_control.nme(1:Ind,:);'    none';geo.vtail2.CAERO1.sup_control.nme(Ind+1:Ind+LIND,:);'    none';geo.vtail2.CAERO1.sup_control.nme(Ind+LIND+1:end,:)];

            geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ;0];
          end
          stick.IDCAERO1.vert = [stick.IDCAERO1.vert; stick.IDCAERO1.vert(end)+1];
          stick.part.vert = [stick.part.vert(1:Ind), Ind, stick.part.vert(Ind+1:end)]; 
          ny_vert =  round(stick.ny.vert(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz);
          if isequal(ny_vert, stick.ny.vert(Ind))
            ny_vert= ny_vert - 1;
          elseif ny_vert==0
            ny_vert = 1;
          end
          stick.ny.vert = [stick.ny.vert(1:Ind-1); ny_vert; stick.ny.vert(Ind)-ny_vert; stick.ny.vert(Ind+1:end) ];
          stick.nx.vert = [stick.nx.vert(1:Ind); stick.nx.vert(Ind);stick.nx.vert(Ind+1:end)];
          stick.nx.sup_control.vert = [stick.nx.sup_control.vert(1:Ind);stick.nx.sup_control.vert(Ind:end)];
          if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+1) > 0
            nTOTInd = (stick.nx.vert(Ind) + stick.nx.sup_control.vert(Ind))*stick.ny.vert(Ind);
          else
            nTOTInd = (stick.nx.vert(Ind))*stick.ny.vert(Ind);
          end
          if geo.vtail.CAERO1.sup_control.frc((Ind-1)*2+2) > 0
           nTOTInd2 = (stick.nx.vert(Ind+1) + stick.nx.sup_control.vert(Ind+1))*stick.ny.vert(Ind+1);
          else
            nTOTInd2 = (stick.nx.vert(Ind+1))*stick.ny.vert(Ind+1);
          end
          stick.nTOT.vert = [stick.nTOT.vert(1:Ind-1); nTOTInd; nTOTInd2; stick.nTOT.vert(Ind+1:end) ];
          T = geo.vtail.CAERO1.chord(Ind+2)/chord;
          TW(1,1) = incidence ;
          TW(2,1) = geo.vtail.CAERO1.incidence(Ind+2);
          SW = geo.vtail.CAERO1.sweepQC(Ind)*pi/180;
          dihed = dihedral *pi/180;
          ox = ptoHorr(1);
          oy = ptoHorr(2);
          oz = ptoHorr(3);

          alpha(1,1) = (geo.vtail.csi_root + (geo.vtail.ni_root - geo.vtail.csi_root)/2)/geo.vtail.CR;
          alpha(2,1) = (geo.vtail.csi_kink + (geo.vtail.ni_kink - geo.vtail.csi_kink)/2)/geo.vtail.CR_kink;
          alpha(3,1) = (geo.vtail.csi_tip + (geo.vtail.ni_tip - geo.vtail.csi_tip)/2)/geo.vtail.CT;

          A(1,1) = alpha(Ind) + (alpha(Ind+1)-alpha(Ind))/dz*(ptoHorr(3)-stick.ptos.vert(3,Ind));
          A(2,1) = alpha(Ind+1);

          REFx = ptoHorr(1);
          REFy = ptoHorr(2);
          REFz = ptoHorr(3);
          [dummy, dummy, dummy, pane] = caero_geo(chord, span, T, TW, SW, dihed, ox, oy, oz, A, REFx, REFy, REFz);
          stick.ptospanel.vert(:,(Ind-1)*4+2:(Ind-1)*4+3) = pane(:,[1,4]);
          stick.ptospanel.vert = [stick.ptospanel.vert(:,1:(Ind-1)*4+4),pane,stick.ptospanel.vert(:,(Ind)*4+1:end)];
          stick.ptos.vert = [stick.ptos.vert(:,1:Ind), ptoHorr , stick.ptos.vert(:,Ind+1:end)];
          n = geo.vtail.CAERO1.n(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz;
          n_coarse = geo.vtail.CAERO1.n_coarse(Ind)*(ptoHorr(3)-stick.ptos.vert(3,Ind))/dz;
          nfore = round(n);
          nfore_coarse = round(n_coarse);
          if isequal(nfore, geo.vtail.CAERO1.n(Ind))
            nfore = nfore - 1;
          elseif nfore==0
            nfore = 1;
          end
          if isequal(nfore_coarse, geo.vtail.CAERO1.n_coarse(Ind))
            nfore_coarse = nfore_coarse - 1;
          elseif nfore_coarse==0
            nfore_coarse = 1;
          end
          naft = geo.vtail.CAERO1.n(Ind) - nfore;
          nadd = [nfore; naft];
          geo.vtail.CAERO1.n = [geo.vtail.CAERO1.n(1:Ind-1); nadd; geo.vtail.CAERO1.n(Ind+1:end)];
          naft_coarse = geo.vtail.CAERO1.n_coarse(Ind) - nfore_coarse;
          nadd_coarse = [nfore_coarse; naft_coarse];
          geo.vtail.CAERO1.n_coarse = [geo.vtail.CAERO1.n_coarse(1:Ind-1); nadd_coarse; geo.vtail.CAERO1.n_coarse(Ind+1:end)];
        else
          Dz = stick.ptos.vert(3,abs(stick.ptos.vert(3,:)-ptoHorr(3))<=0.02*geo.vtail.b/sum(geo.vtail.CAERO1.n_coarse)) - ptoHorr(3);
          stick.ptos.vert(3,:) = stick.ptos.vert(3,:)-Dz;
          stick.ptospanel.stick.ptos.vert(3,:) = stick.ptospanel.stick.ptos.vert(3,:) - Dz;
        end
        [Y, J] = sort(stick.ptos.vert(3,:), 'ascend');
        % Sort points
        stick.ptos.vert(1,:) = stick.ptos.vert(1,J);
        stick.ptos.vert(2,:) = stick.ptos.vert(2,J);
        stick.ptos.vert(3,:) = Y;
        stick.link.vert = 'hori';
      end
    end
  end
end
%
function [wing, qc, c2, pane] = caero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz)

% -->c: cord at wing-fuselge link
% -->b: distance of kink1 from fuselage
% -->T: taper ration CK1/c
% -->TW: root_incidence & kink1_incidence (rad)
% -->SW: sweep angle at QC
% -->dihed: dihedral angle
% --> ox, oy, oz LE point at wing-fuselge link
% --> alpha: fore_spar & aft_spar (adimensional)
% --> REFx, REFy, REFz middle point (structural) at wing-fuselge link
%
lem(1) =  0.25*c;
lem(2) =  0.25*T*c;
lem(3) = -0.75*T*c;
lem(4) = -0.75*c;
%
DX = [(1-cos(TW(1,1)))*cos(SW) (1-cos(TW(2,1)))*cos(SW)...
      (1-cos(TW(2,1)))*cos(SW) (1-cos(TW(1,1)))*cos(SW)].*lem;
%
DY = -[sin(TW(1,1))*sin(dihed)*cos(SW) sin(TW(2,1))*sin(dihed)*cos(SW)...
       sin(TW(2,1))*sin(dihed)*cos(SW) sin(TW(1,1))*sin(dihed)*cos(SW)].*lem;
%
DZ = [sin(TW(1,1))*cos(dihed) sin(TW(2,1))*cos(dihed)...
      sin(TW(2,1))*cos(dihed) sin(TW(1,1))*cos(dihed)].*lem;

% Panel corners
wingx = [0 0.25*c+b*tan(SW)-0.25*T*c 0.25*c+b*tan(SW)+0.75*T*c c] + ox + DX;
wingy = [0 b*cos(dihed) b*cos(dihed) 0] + oy + DY;
wingz = [0 b*sin(dihed) b*sin(dihed) 0] + oz + DZ;


% QC line
qcx = [wingx(1,1) + (wingx(1,4)-wingx(1,1))/4, wingx(1,2) + (wingx(1,3)-wingx(1,2))/4];
qcy = [wingy(1,1) + (wingy(1,4)-wingy(1,1))/4, wingy(1,2) + (wingy(1,3)-wingy(1,2))/4];
qcz = [wingz(1,1) + (wingz(1,4)-wingz(1,1))/4, wingz(1,2) + (wingz(1,3)-wingz(1,2))/4];

% Given line
c2x = [wingx(1,1) + (wingx(1,4)-wingx(1,1))*alpha(1,1), wingx(1,2) + (wingx(1,3)-wingx(1,2))*alpha(2,1)];
c2y = [wingy(1,1) + (wingy(1,4)-wingy(1,1))*alpha(1,1), wingy(1,2) + (wingy(1,3)-wingy(1,2))*alpha(2,1)];
c2z = [wingz(1,1) + (wingz(1,4)-wingz(1,1))*alpha(1,1), wingz(1,2) + (wingz(1,3)-wingz(1,2))*alpha(2,1)];

%**************************************************************************
% Calculate panel corners without incidence angle
DX = [(1-cos(0))*cos(SW) (1-cos(0))*cos(SW)...
      (1-cos(0))*cos(SW) (1-cos(0))*cos(SW)].*lem;
%
DY = -[sin(0)*sin(dihed)*cos(SW) sin(0)*sin(dihed)*cos(SW)...
       sin(0)*sin(dihed)*cos(SW) sin(0)*sin(dihed)*cos(SW)].*lem;
%
DZ = [sin(0)*cos(dihed) sin(0)*cos(dihed)...
      sin(0)*cos(dihed) sin(0)*cos(dihed)].*lem;
% Panel corners
panex = [0 0.25*c+b*tan(SW)-0.25*T*c 0.25*c+b*tan(SW)+0.75*T*c c] + ox + DX;
paney = [0 b*cos(dihed) b*cos(dihed) 0] + oy + DY;
panez = [0 b*sin(dihed) b*sin(dihed) 0] + oz + DZ;
%**************************************************************************

% Move points
dx = REFx - c2x(1);
dy = REFy - c2y(1);
dz = REFz - c2z(1);
%
wingx = wingx + dx;
wingy = wingy + dy;
wingz = wingz + dz;
%
qcx = qcx + dx;
qcy = qcy + dy;
qcz = qcz + dz;
%
c2x = c2x + dx;
c2y = c2y + dy;
c2z = c2z + dz;
%
panex = panex + dx;
paney = paney + dy;
panez = panez + dz;

% Save output
wing = [wingx; wingy; wingz];
qc = [qcx; qcy; qcz];
c2 = [c2x; c2y; c2z];
pane = [panex; paney; panez];

end