function[start_val, NODEY] = starting_point_interp(stretch,nrib_span,nodey,nelem_rib_tra,coord_nodes,size_rib_nodes,size_sect_nodes, ttorq, twbs, As)
count=1;
idx_cg=1;
span_pos(idx_cg)=0;
%
X(idx_cg)=0;
Y(idx_cg)=0;
Z(idx_cg)=0;
%
idx_cg=idx_cg+1;

for i=1:stretch
    for j=1:nrib_span(i)
        span_pos(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,2)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_rib_nodes{i}{j}(1),2)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2),2)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3),2));

        X(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,1)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_rib_nodes{i}{j}(1),1)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2),1)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3),1));
%
        Y(idx_cg) = span_pos(idx_cg);
%
        Z(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,3)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_rib_nodes{i}{j}(1),3)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2),3)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3),3));

        idx_cg=idx_cg+1;
    end
    j=nrib_span(i)+1;

    span_pos(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,2)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_sect_nodes{i+1}(1),2)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(3+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2),2)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(4+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3),2));
%
    X(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,1)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_sect_nodes{i+1}(1),1)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(3+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2),1)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(4+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3),1));
%
    Y(idx_cg) = span_pos(idx_cg);
%
    Z(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,3)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_sect_nodes{i+1}(1),3)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(3+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2),3)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(4+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3),3));
%
    idx_cg=idx_cg+1;
end
%
X(1)=X(2);
Z(1)=Z(2);
%
%
% beam length
%
nelem = length(X);
Lbeam = zeros(nelem-1,1);
for j = 1:nelem-1
  P2 = [X(j+1), Y(j+1), Z(j+1)];
  P1 = [X(j), Y(j), Z(j)];
  Lbeam(j) = norm(P2-P1);
end
NODEY  = [0; cumsum(Lbeam)]';
%
ns = length(span_pos);
start_val = zeros(ns,3);
start_val(:,1) = interp1(nodey, ttorq, NODEY,'linear', 'extrap');
start_val(:,2) = interp1(nodey, twbs, NODEY,'linear', 'extrap');
start_val(:,3) = interp1(nodey, As, NODEY,'linear', 'extrap');
end
