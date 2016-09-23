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
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
function aeroelastic_interface_ext(MasterSurf)

global beam_model;

fid = beam_model.Param.FID;
%SPLINE_TYPE = beam_model.Info.spline_type;
% fix spline type to BEAM spline
SPLINE_TYPE = 1;
%
next = beam_model.Info.ntrimext;
nderext = beam_model.Info.nderext;
EXT = beam_model.Aero.Trim.Ext;
% Reference data
[rho, p, T, a, mu] = ISA_h(beam_model.Aero.state.ALT);
Extra.QREF = 0.5 * rho * beam_model.Aero.state.AS^2;
Extra.PREF = p;
DOFX = beam_model.Node.DOF(beam_model.Node.DOF(:,1)>0,1);
DOFY = beam_model.Node.DOF(beam_model.Node.DOF(:,2)>0,2);
DOFZ = beam_model.Node.DOF(beam_model.Node.DOF(:,3)>0,3);
FX = 0;
FY = 0;
FZ = 0;
%
beam_model.Aero.Trim.DExt.Index = [];
counter = zeros(5+length(MasterSurf),1);
for n=1:nderext
  for k=1:length(beam_model.Aero.Trim.DExt.DOF(n).data)
    name = cell2mat(beam_model.Aero.Trim.DExt.DOF(n).data(k));
    index = 0;
    switch name
      case {'ALPHA', 'alpha'}
        index = 1;
      case {'BETA', 'beta'}
        index = 2;
      case {'p', 'P'}
        index = 3;
      case {'q', 'Q'}
        index = 4;
      case {'q', 'R'}
        index = 5;
    end
    if index == 0
      for j=1:length(MasterSurf)
        if strcmp(MasterSurf(j),name)
          index = 5+j;
          break;
        end
      end
    end
    if index == 0
      error(['Unable to fix DOF ', name, ' given in DEREXT card ', num2str(beam_model.Aero.Trim.DExt.ID(n)),'.']);
    end
    beam_model.Aero.Trim.DExt.Index(n).data(k) = index;
    counter(index) = counter(index) +1;
  end
end
%
if nderext
  counter = counter(counter>0);
  index = find(counter ~= nderext);
  if ~isempty(index)
    error('DEREXT cards must refer to the same derivatives.');
  end
end
%
plot_beam_model(1);
%
switch (beam_model.Param.SOL)
%
  case {144,644,150}
%
	  fprintf(fid, '\n- Building aero-structural interpolation matrices for external aero coupling...');
    fprintf(fid, '\n\t - Method: %d.', SPLINE_TYPE);
    fprintf(fid, '\n\t - Reference load:\n');
    for i=1:next

      Ref.ORIGIN = [0 0 0];
      Ref.Rmat = eye(3);
      Ref.Coord = EXT.Coord(i);
      Symm = beam_model.Aero.Trim.Ext.Symmetry(i);
      if EXT.Coord(i)
        Ref.ORIGIN = beam_model.Coord.Origin(EXT.Coord(i),:);
        Ref.Rmat =  beam_model.Coord.R(:,:,EXT.Coord(i),:);
      end
%
      switch(EXT.Type(i))
%
%       PATRAN file
        case {1}
%
          [C, F] = load_patran(EXT.File{i}.data, Extra, Symm, Ref);

%       FLUENT ASCII file
        case {2}
%
          [C, F] = load_fluent_ascii(EXT.File{i}.data(1), Extra, Symm, Ref);
%
        otherwise
          error(['Unknown file format ', num2str(EXT.Type(i)),'.']);
      end

      body = find(EXT.CAERO(i).data < 100);
      if (isempty(body)) % caero
        beam_model.Aero.Trim.Ext.I{i} = interf_vlm_model1_ext(i, beam_model.Node, beam_model.Aero, C);
      else % body
        if length(EXT.CAERO(i).data)>1
          error(['TRIMEXT card ,' num2str(EXT.ID(n)), ' refer to both body and aero panels or to multiple bodies.']);
        end
        beam_model.Aero.Trim.Ext.I{i} = interf_body_model1_ext(i, beam_model.Node, beam_model.Aero, C);
      end
%     transfer Fa0
      beam_model.Aero.Trim.Ext.F0{i} = beam_model.Aero.Trim.Ext.I{i} * F;
      fprintf(fid, 'Forces resultants along structural mesh: ');
      fx = sum(beam_model.Aero.Trim.Ext.F0{i}(DOFX));
      fy = sum(beam_model.Aero.Trim.Ext.F0{i}(DOFY));
      fz = sum(beam_model.Aero.Trim.Ext.F0{i}(DOFZ));
      fprintf(fid, '\n\tFx: %g.', fx);
      fprintf(fid, '\n\tFy: %g.', fy);
      fprintf(fid, '\n\tFz: %g.', fz);
      fprintf(fid, '\n');
      FX = FX + fx;
      FY = FY + fy;
      FZ = FZ + fz;
%    
      if (beam_model.Param.DER_TYPE == 2)
        fprintf(fid,'External derivative:\n');
        drext = beam_model.Aero.Trim.Ext.DExt(i);
        if drext
          npert = length(beam_model.Aero.Trim.DExt.DOF(drext).data);
          pert = beam_model.Aero.Trim.DExt.Pert(drext).data;
          beam_model.Aero.Trim.Ext.DF{i}.data = zeros(length(beam_model.Aero.Trim.Ext.F0{i}),npert);
          for kpert = 1:npert
            fprintf(fid,' - %s:', cell2mat(beam_model.Aero.Trim.DExt.DOF(drext).data(kpert)));
            switch(EXT.Type(i))
              case {1}
                [C, F] = load_patran(beam_model.Aero.Trim.DExt.File{drext}.data(kpert,:), Extra, Symm, Ref);
              case {2}
                [C, F] = load_fluent_ascii(beam_model.Aero.Trim.DExt.File{drext}.data(kpert,1), Extra, Symm, Ref);
              otherwise
                error(['Unknown file format ', num2str(EXT.Type(i)),'.']);
            end
            %     transfer Fa
            fprintf(fid, 'Forces resultants along structural mesh: ');
            DFA = beam_model.Aero.Trim.Ext.I{i} * F;
            fxd = sum(DFA(DOFX));
            fyd = sum(DFA(DOFY));
            fzd = sum(DFA(DOFZ));
            fprintf(fid, '\n\tFx: %g.', fxd);
            fprintf(fid, '\n\tFy: %g.', fyd);
            fprintf(fid, '\n\tFz: %g.', fzd);
            fprintf(fid, '\n');
            beam_model.Aero.Trim.Ext.DF{i}.data(:,kpert) = (DFA - beam_model.Aero.Trim.Ext.F0{i})./pert(kpert);
          end
        end
      end % der
%
    end % loop
%
    fprintf(fid, '\nFinal resultants along structural mesh for reference load: ');
    fprintf(fid, '\n\tFx: %g.', FX);
    fprintf(fid, '\n\tFy: %g.', FY);
    fprintf(fid, '\n\tFz: %g.', FZ);
    fprintf(fid, '\n');
%
end
%
end
%
function I = interf_vlm_model1_ext(n, NODE, AERO, aer_data)
%
I = [];
ri = []; ci = []; vi = [];
%
Ext = AERO.Trim.Ext;
ndof = max(max(NODE.DOF));
[naer, nv, dim] = size(aer_data);
%
nset = Ext.Set(n);
str_data = NODE.Coord(AERO.Set.Node(nset).data,1:3); % get stuctural coords
%
nstr = size(str_data,1);
i1 = Ext.Index(n, 1);
i2 = Ext.Index(n, 2);
toll = Ext.Toll(n);
[Imv, Rmat] = beam_interface(str_data, aer_data, i1, i2, toll); % interface points in local frame
%------------------------------------------------------------------------------------------------------------------
% R rotates forces in the local frame: pick only z component
for k=1:naer
  ri = [ri, k.*ones(1, 3)];
  ci = [ci, (k-1)*3+1:k*3];
  vi = [vi, Rmat(3,:)];
end
ri = double(ri); ci = double(ci); vi = double(vi);
% add 2naer zeros in rows for bending and torque 
nrows = double(3*naer); ncols = double(3*naer); 
R = sparse(ri, ci, vi, nrows, ncols);
%
%------------------------------------------------------------------------------------------------------------------
% Rt rotates aero forces in the absolute frame and send them to nodal DOF
ri = [];
ci = [];
vi = [];
RmatT = Rmat';
for k=1:nstr
  index = find(NODE.DOF(AERO.Set.Node(nset).data(k),1:3)); % check translation DOF are not fixed
  if (~isempty(index))
    ri = [ri, NODE.DOF(AERO.Set.Node(nset).data(k),index)];  
    ci = [ci, k.*ones(1,length(index))];
    vi = [vi, RmatT(index,3)'];
  end
  index = find(NODE.DOF(AERO.Set.Node(nset).data(k),4:6)); % check rotation DOF are not fixed
  if (~isempty(index))
    ri = [ri, repmat(NODE.DOF(AERO.Set.Node(nset).data(k),3+index), 1, 2)];  
    ci = [ci, repmat((nstr+k), 1, length(index)), repmat((2*nstr+k), 1, length(index))];
    vi = [vi, RmatT(index,1)', RmatT(index,2)'];
  end
end
ri = double(ri); ci = double(ci); vi = double(vi);
nrows = double(ndof); ncols = double(3*nstr);
Rt = sparse(ri, ci, vi, nrows, ncols);
% Assemble interpolation matrix
I = Rt * (Imv') * R; 
%
end
%
function I = interf_body_model1_ext(n, NODE, AERO, aero_data)
%
I = [];
toll = 0;
ndof = max(max(NODE.DOF));
%
Ext = AERO.Trim.Ext;
nset = Ext.Set(n);
% use body reference frame for spline
str_data = NODE.Coord(AERO.Set.Node(nset).data,1:3); % get stuctural coords   
nstr = size(str_data,1);
naer = size(aero_data,1);
i1 = Ext.Index(n, 1); i2 = Ext.Index(n, 2);
ORIG = str_data(i1,:); % origin
P3 = (str_data(i2,:)-ORIG)'; % x axis
P2 = [0; 1; 0]; % y axis
Py = P2;
P2 = (crossm(P3)* P2); % z axis
Pz = P2 ./norm(P2);
Px = P3 ./norm(P3);
Rbody = [Px, Py, Pz];
% rotate frame to have y along body axis
Rmatz = (expm(crossm([0 0 -pi/2])) * Rbody)'; 
% rotate frame to have y along body axis
Rmaty = (expm(crossm([pi/2 0 0])) * expm(crossm([0 0 -pi/2])) * Rbody)'; 
%
Hz = beam_interface2(str_data, aero_data, toll, ORIG, Rmatz);
Hy = beam_interface2(str_data, aero_data, toll, ORIG, Rmaty);
ri = []; ci = []; viz = []; viy = [];
% R rotates forces in the local frame: pick only z component
for k=1:naer
  ri = [ri, k.*ones(1, 3)];
  ci = [ci, (k-1)*3+1:k*3];
  viz = [viz, Rmatz(3,:)];
  viy = [viy, Rmaty(3,:)];
end
ri = double(ri); ci = double(ci); viz = double(viz); viy = double(viy);
% add 2naer zeros in rows for bending and torque 
nrows = double(3*naer); ncols = double(3*naer); 
Rz = sparse(ri, ci, viz, nrows, ncols);
Ry = sparse(ri, ci, viy, nrows, ncols);
% Rt rotates aero forces in the absolute frame and send them to nodal DOF
ri = []; ci = []; viz = []; viy = [];
RmatTz = Rmatz';
RmatTy = Rmaty';
for k=1:nstr
  index = find(NODE.DOF(AERO.Set.Node(nset).data(k),1:3)); % check translation DOF are not fixed
  if (~isempty(index))
    ri = [ri, NODE.DOF(AERO.Set.Node(nset).data(k),index)];  
    ci = [ci, k.*ones(1,length(index))];
    viz = [viz, RmatTz(index,3)'];
    viy = [viy, RmatTy(index,3)'];
  end
  index = find(NODE.DOF(AERO.Set.Node(nset).data(k),4:6)); % check rotation DOF are not fixed
  if (~isempty(index))
    ri = [ri, repmat(NODE.DOF(AERO.Set.Node(nset).data(k),3+index), 1, 2)];  
    ci = [ci, repmat((nstr+k), 1, length(index)), repmat((2*nstr+k), 1, length(index))];
    viz = [viz, RmatTz(index,1)', RmatTz(index,2)'];
    viy = [viy, RmatTy(index,1)', RmatTy(index,2)'];
  end
end
ri = double(ri); ci = double(ci); viz = double(viz); viy = double(viy);
nrows = double(ndof); ncols = double(3*nstr);
Rtz = sparse(ri, ci, viz, nrows, ncols);
Rty = sparse(ri, ci, viy, nrows, ncols);
%------------------------------------------------------------------------------------------------------------------
%   Assemble interpolation matrix
I = Rtz * (Hz') * Rz + Rty * (Hy') * Ry; 
end
 