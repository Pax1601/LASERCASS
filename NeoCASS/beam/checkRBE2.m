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

function [RBE2,Node,nrbe,ndof2] = checkRBE2(fid,RBE2,Node,PARAM,Bar,Beam,Celas,ngrid,SPC,ndofIn)

IDS = [];
IDM = [];
NGDL = [];
IDRBE2 = [];
Conn = [Bar.Conn;Beam.Conn];
cont = 1;
nrbe = length(RBE2.ID);
ndof2 = ndofIn;
% DOFS = [];
% DOFM = [];
if ~isempty(find(diff(RBE2.ID)==0,1))
  error(fid, 'Duplicated RBE2 ID.');
else
%  check master and slave consistency
  for i = 1 : length(RBE2.ID)
    IDM = [IDM,RBE2.IDM(i)];
    IDS = [IDS,RBE2.IDS(i).data];
    v = intersect(IDM,IDS);
    if ~isempty(v)
      errmsg = ['Node ', num2str(v(1)),' must be either master or slave. Check RBE2 n. id ', num2str(RBE2.ID(i)),'.'];
      error(errmsg);
    end
%
    ns   = length(RBE2.IDS(i).data);
    nGDL = length(RBE2.GDL(i).data);
%
    GDL = zeros(1,nGDL);
    for j = 1 : nGDL
      GDL(j) = str2double(RBE2.GDL(i).data(j));
    end
%  
    RBE2.GDL(i).data = GDL;
    master = find(Node.ID == RBE2.IDM(i));
    RBE2.DOFM(i).data = Node.DOF(master,:); % master DOFs
    RBE2.DOFS(i).data = [];                 % slave DOFs
    RBE2.DOFL(i).data = [];                 % connected nodes DOFs
%        
    for j = 1 : ns
      NGDL = [NGDL;nGDL];
      IDRBE2= [IDRBE2;cont];
      slave = find(Node.ID == RBE2.IDS(i).data(j));
      first  = find(slave == Conn(:,1) );
      second = find(slave == Conn(:,3) );
%
%     Get connected nodes DOFs
%
      if (isempty(first) && isempty(second)) % no stiffness associated, mechanisms will appear
%       check for CELAS
        if ~isempty(Celas.Node)
          first  = find(RBE2.IDS(i).data(j) == Celas.Node(:,1) );
          second = find(RBE2.IDS(i).data(j) == Celas.Node(:,2) );
          DOFcelas = [];
          for c1 = 1 : length(first)
            DOFcelas = [DOFcelas;Celas.DOF(first(c1),2).data'];
          end

          for c2 = 1 : length(second)
            DOFcelas = [DOFcelas;Celas.DOF(second(c2),1).data'];
          end
          if ~(isempty(DOFcelas))
            DOFcelas = sort(DOFcelas(ismember(DOFcelas,DOFlinked)==0));
            DOFcelas = [DOFcelas((diff(DOFcelas)~=0)),DOFcelas(end)];
            RBE2.DOFL(i).data = [RBE2.DOFL(i).data; DOFcelas];
          end
        else % pure rigid element
          if (nGDL<6)
            errmsg = ['RBE ', num2str(RBE2.ID(i)), ' results in no stiffness for DOF ',num2str(setdiff([1:6],GDL)),'.'];
            error(errmsg);
          end        
        end
%
      else % stiffness associated
%
        linked = sort([reshape(Conn(first,[2,3]),1,length(first)*2),reshape(Conn(second,[1,2]),1,length(second)*2)]);
        linked = [linked((diff(linked)~=0)),linked(end)];
        DOFlinked = sort([Node.DOF(slave,setdiff(1:6,GDL))'; reshape(Node.DOF(linked,:)',length(linked)*6,1)]);
        RBE2.DOFL(i).data = [RBE2.DOFL(i).data; DOFlinked];
        RBE2.DOFL(i).data = RBE2.DOFL(i).data(RBE2.DOFL(i).data>0);
      end 
%
      RBE2.DOFS(i).data = [RBE2.DOFS(i).data; Node.DOF(slave,GDL)'];
      r = Node.Coord(slave,:) - Node.Coord(master,:);
      ir = [1 2 3 4 5 6 1 1 2 2 3 3 ];
      jr = [1 2 3 4 5 6 5 6 4 6 4 5];
      vr = [1 1 1 1 1 1 r(3), -r(2), -r(3), r(1), r(2), -r(1)];
      RP = sparse(ir,jr,vr,6,6);
      RBE2.R(i).data( (j-1)*nGDL+1 : j*nGDL,: ) = RP(GDL,:);
    end % slave loop
    %
    cont = cont+1;
  end % RBE loop
end % if
%
if PARAM.SPC
%
  count = 0;
  fprintf(fid, '\n\tSetting Nodal contraints...');
  spcindex = find( SPC.ID == PARAM.SPC);
  if isempty(spcindex)
    error('Unable to find SPC set %d as required.', PARAM.SPC);
  else
    set = 1:6;
    ndof = 0;
    ndof2 = 0;
% count dof
    dofCompare = zeros(ndofIn,1);
    for n=1:ngrid
      if Node.ID(n)  && isempty(find(Node.ID(n)==IDS,1)) && ~isempty(find(Node.DOF(n, :),1))% structural node
        index = find(Node.DOF(n, :));
        index = ismember(set, index);
        Node.DOF2(n, :) = int32(zeros(1,6));
        Node.DOF2(n, index) = int32(ndof+1 : ndof+length(find(index==1)));
        ndof = ndof + length(find(index==1));
        dofIn =  Node.DOF(n,index);
        index = find(Node.DOF2(n, :));
        Node.DOF(n,:) = int32(zeros(1,6));
        Node.DOF(n, index) = int32(ndof2+1 : ndof2+length(index));
        ndof2 = ndof2 + length(index);
        dofCompare(dofIn) = Node.DOF(n,index);
      elseif Node.ID(n) && ~isempty(find(Node.DOF(n, :),1))
        ids = find(Node.ID(n)==IDS);
        index = find(Node.DOF(n, :));
        index = ismember(set, index);
        Node.DOF2(n, :) = int32(zeros(1,6));
        Node.DOF2(n,RBE2.GDL(IDRBE2(ids)).data) = RBE2.DOFM(IDRBE2(ids)).data(RBE2.GDL(IDRBE2(ids)).data);
        Node.DOF2(n,setdiff(1:6,RBE2.GDL(IDRBE2(ids)).data)) = int32(ndof+1 : ndof+6-NGDL(ids));
        ndof = ndof + length(find(index==1))-NGDL(ids);
        dofIn =  Node.DOF(n,index);
        index1 = index;
        index = find(Node.DOF2(n, :));
        Node.DOF(n,:) = int32(zeros(1,6));
        Node.DOF(n, index) = int32(ndof2+1 : ndof2+length(index));
        ndof2 = ndof2 + length(index);
        dofCompare(dofIn) = Node.DOF(n,index1);
      else
        Node.DOF2(n, :) = int32(zeros(1,6));
        Node.DOF(n,:) = int32(zeros(1,6));
      end
    end
    deleteRBE2 = [];
    for i = 1 : nrbe
      indDOFM = find((RBE2.DOFM(i).data)~=0);
      RBE2.DOFM(i).data = dofCompare(RBE2.DOFM(i).data(indDOFM));
      RBE2.DOFS(i).data = dofCompare(RBE2.DOFS(i).data);
      indDOFS = find(RBE2.DOFS(i).data~=0);
      RBE2.DOFS(i).data = RBE2.DOFS(i).data(indDOFS);
      RBE2.R(i).data = RBE2.R(i).data(indDOFS,indDOFM);
      RBE2.DOFL(i).data = dofCompare(RBE2.DOFL(i).data);
      RBE2.DOFL(i).data = RBE2.DOFL(i).data(RBE2.DOFL(i).data~=0);
      for ii = 1 : length(spcindex)
        spcset = spcindex(ii);
        if ismember(RBE2.IDM(i),Node.ID(SPC.Nodes(spcset).list))
          if any(RBE2.DOFM(i).data) % no SPC on master DOF
            RBE2.DOFM(i).data = RBE2.DOFM(i).data(RBE2.DOFM(i).data~=0);
          else
            deleteRBE2 = [deleteRBE2,i];
          end
        elseif any(ismember(RBE2.IDS(i).data,Node.ID(SPC.Nodes(spcset).list)))
          error('SPC node cannot be a slave!!!');
        end
      end
    end
    index = setdiff(1:nrbe,deleteRBE2);
    RBE2.ID = RBE2.ID(index);
    RBE2.IDM = RBE2.IDM(index);
    RBE2.IDS = RBE2.IDS(index);
    RBE2.DOFM = RBE2.DOFM(index);
    RBE2.DOFS = RBE2.DOFS(index);
    RBE2.DOFL = RBE2.DOFL(index);
    RBE2.R = RBE2.R(index);
    nrbe = length(index);
  end %end check
  fprintf(fid,'done.');
%
else
%   
  set = 1:6;
  ndof = 0;
% count dof
  for n=1:ngrid
    if Node.ID(n) && isempty(find(Node.ID(n)==IDS,1)) && ~isempty(find(Node.DOF(n, :),1))% structural master node
      Node.DOF2(n, :) = int32(zeros(1,6));
      Node.DOF2(n, :) = int32(ndof+1 : ndof+6);
      ndof = ndof + 6;
    elseif Node.ID(n) && ~isempty(find(Node.DOF(n, :),1))
      ids = find(Node.ID(n)==IDS);
      Node.DOF2(n, :) = int32(zeros(1,6));
      Node.DOF2(n,RBE2.GDL(IDRBE2(ids)).data) = RBE2.DOFM(IDRBE2(ids)).data(RBE2.GDL(IDRBE2(ids)).data);
      Node.DOF2(n,setdiff(1:6,RBE2.GDL(IDRBE2(ids)).data)) = int32(ndof+1 : ndof+6-NGDL(ids));
      ndof = ndof + 6-NGDL(ids);
    else
      Node.DOF2(n, :) = int32(zeros(1,6));
    end
  end
%
end % end PARAM