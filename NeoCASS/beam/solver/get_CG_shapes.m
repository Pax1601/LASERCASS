function [RMODE, RMODEN, RINDEX, EPS] = get_CG_shapes(NODE, RBE2, CG, SUPORT)
%
% 01-08-2016 Federico Fonte: partial check on RBE2 DOFs (line 26)
%
  nnodes = size(SUPORT,1);
  index = [];
  DOF = NODE.DOF;
  DOF2 = NODE.DOF2;
  for i=1:nnodes
    m = find(SUPORT(i,1) == [NODE.ID]);
    dof = num2str(SUPORT(i,2));
    for k=1:length(dof)
      index = [index, DOF2(m, str2num(dof(k)))];
      RINDEX(k) = str2num(dof(k));
    end
  end
  RMODE = zeros(max(max(DOF2)),6);
  RMODEN = zeros(max(max(DOF)),6);
  nnodes = length(NODE.ID);
  %
  if ~isempty(RBE2)
    for i=1:length(RBE2.IDS)
      for j=1:length(RBE2.IDS(i).data)
        nodeindex = find(NODE.ID == RBE2.IDS(i).data(j));
        dofindex  = RBE2.GDL(i).data;
        DOF2(nodeindex, dofindex) = 0;
      end
    end
  end
%
  for j=1:nnodes
    indext = find(DOF2(j,1:3));
    indexr = find(DOF2(j,4:6));
    arm = crossm(CG-NODE.Coord(j,:));
    for k=1:length(indext)
      RMODE(DOF2(j,indext(k)), indext(k)) = 1; % translation x,y,z
    end
    RMODE(DOF2(j,indext), 4:6) = arm(indext,:);
    for k=1:length(indexr)
      RMODE(DOF2(j,indexr(k)+3),indexr(k)+3) = 1;
    end
  end
  if ~isempty(RBE2) 
    for j=1:nnodes
      indext = find(DOF(j,1:3));
      indexr = find(DOF(j,4:6));
      arm = crossm(CG-NODE.Coord(j,:));
      for k=1:length(indext)
        RMODEN(DOF(j,indext(k)), indext(k)) = 1; % translation x,y,z
      end
      RMODEN(DOF(j,indext), 4:6) = arm(indext,:);
      for k=1:length(indexr)
        RMODEN(DOF(j,indexr(k)+3),k+3) = 1;
      end
     end 
     RMODEN = RMODEN(:,RINDEX);
  else
    RMODEN = RMODE(:,RINDEX);
  end
%
  RMODE = RMODE(:,RINDEX);
  EPS = zeros(length(RINDEX),1);
end
