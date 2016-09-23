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
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna DIAPM
%***********************************************************************************************************************
% Inputs:
% guess_model.loads struct
% guess_model geo struct
% ITEM: index of the component 1=fuse, 2=wing, 3=vt, 4=ht, 5=canard
% SecPerc: array with spanwise percentage 
% for example [0.0 0.5 0.8] will plot diagrams for sections at 0, 50% and 80%
% MConf mass configuration index (1 only for guess standard)
%
function plot_sizing_man(loads, geo, ITEM, SecPerc, Mconf)
nfig = 0;

switch ITEM
%
case 1
if isfield(loads.fus,'man')
  nfig = plot_man_loads(nfig, loads.fus.man, 'Fuselage', Mconf);
  nfig = plot_potato_sec_loads(nfig, loads.fus.man, geo.fus.x, SecPerc, 'Fuselage', Mconf);
end
%
case 2
if isfield(loads.wing,'man')
  nfig = plot_man_loads(nfig, loads.wing.man, 'Wing', Mconf);
  nfig = plot_potato_sec_loads(nfig, loads.wing.man, geo.wing.y, SecPerc, 'Wing', Mconf);
end
%
case 3
if isfield(loads.vtail,'man')
  nfig = plot_man_loads(nfig, loads.vtail.man, 'Vtail', Mconf);
  nfig = plot_potato_sec_loads(nfig, loads.vtail.man, geo.vtail.y, SecPerc, 'Vtail', Mconf);
end
%
case 4
if isfield(loads.htail,'man')
  nfig = plot_man_loads(nfig, loads.htail.man, 'Vtail', Mconf);
  nfig = plot_potato_sec_loads(nfig, loads.htail.man, geo.htail.y, SecPerc, 'Htail', Mconf);
end
%
case 5
if isfield(loads.canard,'man')
  nfig = plot_man_loads(nfig, loads.canard.man, 'Canard', Mconf);
  nfig = plot_potato_sec_loads(nfig, loads.canard.man, geo.canard.y, SecPerc, 'Canard', Mconf);
end
%
end % switch
%
end

function nfig = plot_man_loads(nfig,man, label, mconf)
%
  NCONFTOT = size(man.FS_i,2);
  NTRIM = size(man.FS,2);
  if (mconf>NCONFTOT)
    fprintf(1,'\n### Warning: mass configuration %d not available.', mconf);
    return
  end
  FSI = man.FS_i(:,mconf);
  MTI = man.Mt_i(:,mconf);
  MI = man.M_i(:,mconf);
%
  nfig = nfig+1; figure(nfig); close;figure(nfig); 
  plot(FSI, [1:length(FSI)],'.');
  grid on;
  xlabel('Maneuver ID');  ylabel('Section index'); title([label,' maximum absolute shear, configuration ', num2str(mconf)]);
  set(gca,'Xlim',[1,NTRIM]'); set(gca,'xTick',[1:NTRIM]')
  nfig = nfig+1; figure(nfig); close;figure(nfig);
  plot(MI, [1:length(MI)],'.');
  set(gca,'Xlim',[1,NTRIM]'); set(gca,'xTick',[1:NTRIM]')
  grid on;
  xlabel('Maneuver ID');  ylabel('Section index'); title([label,' maximum absolute bending, configuration ', num2str(mconf)]);
  nfig = nfig+1; figure(nfig); close;figure(nfig);
  plot(MTI, [1:length(MTI)],'.');
  grid on;
  xlabel('Maneuver ID');  ylabel('Section index'); title([label,' maximum absolute torque, configuration ', num2str(mconf)]);
  set(gca,'Xlim',[1,NTRIM]'); set(gca,'xTick',[1:NTRIM]')
%
  if (NCONFTOT>1)
    FSK = man.FS_j;
    MTK = man.Mt_j;
    MK = man.M_j;
    NS = length(FSK);
    nfig = nfig+1; figure(nfig); close;figure(nfig); 
    plot(FSK, [1:length(FSK)],'.');
    grid on;
    xlabel('Mass configuration');  ylabel('Section index'); title([label,' critical mass configuration for shear']);
    set(gca,'Xlim',[1 NCONFTOT]);
    set(gca,'xTick',[1:NCONFTOT]'); 
    nfig = nfig+1; figure(nfig); close;figure(nfig);
    plot(MK, [1:length(MK)],'.');
    set(gca,'Xlim',[1 NCONFTOT]);
    set(gca,'xTick',[1:NCONFTOT]'); 
    grid on;
    xlabel('Mass configuration');  ylabel('Section index'); title([label,' critical mass configuration for bending']);
    nfig = nfig+1; figure(nfig); close;figure(nfig);
    plot(MTK, [1:length(MTK)],'.');
    grid on;
    xlabel('Mass configuration');  ylabel('Section index'); title([label,' critical mass configuration for torque']);
    set(gca,'Xlim',[1 NCONFTOT]);
    set(gca,'xTick',[1:NCONFTOT]'); 
  end
%
end

function nfig = plot_potato_sec_loads(nfig, man, geosec, SecPerc, label, mconf)
%
if (mconf>size(man.FS,3))
  fprintf(1,'\n### Warning: mass configuration %d not available.', mconf);
  return
end
KX = 1.04;
KY = 1.04;
%
Y = geosec ./ (geosec(end)-geosec(1));
for n=1:length(SecPerc)
  [v,sec] = min(abs(Y-SecPerc(n)));
sec
  nfig = nfig+1; figure(nfig); close;figure(nfig); hold on;
  px = [];
  py = [];
  for k=1:size(man.FS,2)
    plot(man.FS(sec,k,mconf)./1000, man.M(sec,k,mconf)./1000,'ks');
    text(man.FS(sec,k,mconf)./1000*KX, man.M(sec,k,mconf)./1000*KY,num2str(k));
    px = [px; man.FS(sec,k,mconf)./1000];
    py = [py; man.M(sec,k,mconf)./1000];
  end
  maxFS = max(man.FS(sec,:,mconf))./1000
  maxM = max(man.M(sec,:,mconf))./1000
  minFS = min(man.FS(sec,:,mconf))./1000
  minM = min(man.M(sec,:,mconf))./1000
  plot([minFS, maxFS], [minM, maxM], 'ro'); 
  grid on;
  xlabel('Shear [KN]');  ylabel('Bending [KNm]'); title([label, ' shear Vs bending at ', num2str(SecPerc(n)*100),'%, configuration ', num2str(mconf)]);
  if (length(unique(px))>2)
    k = convhull(px, py);
    plot(px(k), py(k), '-k'),
    tr = 0.4;
    patch(px(k), py(k), 'g', 'facealpha', tr, 'edgecolor', 'g', 'edgealpha', tr);
  end 
  nfig = nfig+1; figure(nfig); close;figure(nfig); hold on;
  px = [];
  py = [];
  for k=1:size(man.FS,2)
    plot(man.FS(sec,k,mconf)./1000, man.Mt(sec,k,mconf)./1000,'ks');
    text(man.FS(sec,k,mconf)./1000*KX, man.Mt(sec,k,mconf)./1000*KY,num2str(k));
    px = [px; man.FS(sec,k,mconf)./1000];
    py = [py; man.Mt(sec,k,mconf)./1000];
  end
  maxFS = max(man.FS(sec,:,mconf))./1000;
  maxMt = max(man.Mt(sec,:,mconf))./1000;
  minFS = min(man.FS(sec,:,mconf))./1000;
  minMt = min(man.Mt(sec,:,mconf))./1000;
  plot([minFS, maxFS], [minMt , maxMt], 'ro'); 
  grid on;
  xlabel('Shear [KN]');  ylabel('Torque [KNm]'); title([label, ' shear Vs torque at ', num2str(SecPerc(n)*100),'%, configuration ', num2str(mconf)]);
  if (length(unique(px))>2)
    k = convhull(px, py);
    plot(px(k), py(k), '-k'),
    tr = 0.4;
    patch(px(k), py(k), 'g', 'facealpha', tr, 'edgecolor', 'g', 'edgealpha', tr);
  end
% 
  nfig = nfig+1; figure(nfig); close;figure(nfig); hold on;
  px = [];
  py = [];
  for k=1:size(man.FS,2)
    plot(man.M(sec,k,mconf)./1000, man.Mt(sec,k,mconf)./1000,'ks');
    text(man.M(sec,k,mconf)./1000*KX, man.Mt(sec,k,mconf)./1000*KY,num2str(k));
    px = [px; man.M(sec,k,mconf)./1000];
    py = [py; man.Mt(sec,k,mconf)./1000];
  end
  maxM = max(man.M(sec,:,mconf))./1000;
  maxMt = max(man.Mt(sec,:,mconf))./1000;
  minM = min(man.M(sec,:,mconf))./1000;
  minMt = min(man.Mt(sec,:,mconf))./1000;
  plot([minM, maxM], [minMt , maxMt], 'ro'); 
  grid on;
  xlabel('Bending [KNm]');  ylabel('Torque [KNm]'); title([label, ' bending Vs torque at ', num2str(SecPerc(n)*100),'%, configuration ', num2str(mconf)]);
  if (length(unique(px))>2)
    k = convhull(px, py);
    plot(px(k), py(k), '-k'),
    tr = 0.4;
    patch(px(k), py(k), 'g', 'facealpha', tr, 'edgecolor', 'g', 'edgealpha', tr); 
  end
end
%
end