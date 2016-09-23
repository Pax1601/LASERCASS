function[n_rib,rib_id,spc_nodes,fuse_constr,spar_v,rib_centre]=rib_generator(grid_coord,size_sect_nodes,size_rib_nodes,stretch,nrib_span,...
    nelem_tra,nelem_rib_tra,nelem_tot,nnodes_tot,nelem_rib,material,spc_nodes,rib_thick)
%Funzione per la generazione delle centine nel modello
OFFSET = 1999999;
fuse_constr = [];
spar_v = [];
n_baie=(sum(nrib_span)+length(nrib_span))*nelem_rib_tra;
idx_size=1;
dim(idx_size,:)=size_sect_nodes{1};idx_size=idx_size+1;
for i=1:stretch
    for j=1:nrib_span(i)
        for p=1:nelem_rib_tra
            dim(idx_size,:)=size_rib_nodes{i}{j};
            idx_size=idx_size+1;
        end
    end
    for p=1:nelem_rib_tra
        dim(idx_size,:)=size_sect_nodes{i+1};
        idx_size=idx_size+1;
    end
end



n_nodi=sum(dim')'+4;                                                            %Numero dei nodi in ogni sezioni
cum_n_nodi=cumsum(n_nodi);                                                      %Cumulativo dei nodi in ogni sezioni
n_rib=((length(dim)-1)/nelem_rib_tra)+1;
pshell_id=20000;

idx_node=1;  
stat=1;
pshell_id=pshell_id+1;
v1=grid_coord(1,:);                                                             %Coordinate vertice 1 della sezione
v3=grid_coord(2+dim(stat,1),:);                                                 %Coordinate vertice 3 della sezione
v4=grid_coord(3+dim(stat,1)+dim(stat,2),:);                                     %Coordinate vertice 4 della sezione
v2=grid_coord(4+dim(stat,1)+dim(stat,2)+dim(stat,3),:);                         %Coordinate vertice 2 della sezione
count=1;
vert1(count,:)=v1;vert2(count,:)=v2;vert3(count,:)=v3;vert4(count,:)=v4;
count=count+1;
centre=(v1+v2+v3+v4)/4;                                                         %Coordinate del centro del box
for k=nelem_rib-1:-1:1
    for w=1:sum(dim(stat,:))+4
        fraz=k/nelem_rib;
        node(idx_node,:)=fraz*[grid_coord(w,1)-centre(1) grid_coord(w,2)-centre(2) grid_coord(w,3)-centre(3)]+centre;
        idx_node=idx_node+1;
    end
end


fout=fopen('rib.dat','w');


for k=1:idx_node-1
    fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8d\n','GRID    ',OFFSET+nnodes_tot+k,0,node(k,1),node(k,2),node(k,3),0);    %Scrittura dei grid delle centine
    spc_nodes(end+1)=nnodes_tot+k;
end

fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8d\n','GRID    ',OFFSET+nnodes_tot+idx_node,0,centre(1),centre(2),centre(3),0);    %Scrittura del grid centrale
spc_nodes(end+1)=nnodes_tot+idx_node;
rib_centre(1).grid=nnodes_tot+idx_node;
rib_centre(1).coord=[centre(1) centre(2) centre(3)];
id_centre=nnodes_tot+idx_node;
nnodes_tot_start=nnodes_tot;
nnodes_tot=nnodes_tot+idx_node;
delta_node=(idx_node-1)/(nelem_rib-1);
fclose(fout);

id_seq=zeros(nelem_rib,delta_node);
id_seq(1,:)=[1:3+dim(stat,1)+dim(stat,2) ...
    fliplr(4+dim(stat,1)+dim(stat,2):3+dim(stat,1)+dim(stat,2)+dim(stat,3)) ...
    4+dim(stat,1)+dim(stat,2)+dim(stat,3):4+dim(stat,1)+dim(stat,2)+dim(stat,3)+dim(stat,4)];
for k=2:nelem_rib
    id_seq(k,:)=[1+nnodes_tot_start+(k-2)*delta_node:3+dim(stat,1)+dim(stat,2)+nnodes_tot_start+(k-2)*delta_node ...
        fliplr(4+dim(stat,1)+dim(stat,2)+nnodes_tot_start+(k-2)*delta_node:3+dim(stat,1)+dim(stat,2)+dim(stat,3)+nnodes_tot_start+(k-2)*delta_node) ...
        4+dim(stat,1)+dim(stat,2)+dim(stat,3)+nnodes_tot_start+(k-2)*delta_node:4+dim(stat,1)+dim(stat,2)+dim(stat,3)+dim(stat,4)+nnodes_tot_start+(k-2)*delta_node];
end
%Scrittura dei quad4 della centina
fout=fopen('rib.dat','a');
for k=1:nelem_rib-1
    for w=1:delta_node-1
        fprintf(fout,'%8s%8d%8d%8d%8d%8d%8d\n','CQUAD4  ',nelem_tot+w+delta_node*(k-1),pshell_id,OFFSET+id_seq(k,w),...
          OFFSET+id_seq(k,w+1),OFFSET+id_seq(k+1,w+1),OFFSET+id_seq(k+1,w));
    end
    w=delta_node;
    fprintf(fout,'%8s%8d%8d%8d%8d%8d%8d\n','CQUAD4  ',nelem_tot+w+delta_node*(k-1),pshell_id,OFFSET+id_seq(k,w),OFFSET+id_seq(k,1),...
      OFFSET+id_seq(k+1,1),OFFSET+id_seq(k+1,w));        
end

nelem_tot=nelem_tot+(nelem_rib-1)*delta_node;
%Scrittura dei tria3 della centina
for w=1:delta_node-1
    fprintf(fout,'%8s%8d%8d%8d%8d%8d\n','CTRIA3  ',nelem_tot+w,pshell_id,OFFSET+id_seq(nelem_rib,w),OFFSET+id_seq(nelem_rib,w+1),OFFSET+id_centre);
end
w=delta_node;
fprintf(fout,'%8s%8d%8d%8d%8d%8d\n','CTRIA3  ',nelem_tot+w,pshell_id,OFFSET+id_seq(nelem_rib,w),OFFSET+id_seq(nelem_rib,1),OFFSET+id_centre);
fclose(fout);
nelem_tot=nelem_tot+delta_node;

fuse_sect = nrib_span(1) +1;                                                                           
for i=1:n_rib-1                                                                 %Da questo ciclo manca la centina di root
    idx_node=1;  
    stat=1+i*nelem_rib_tra;
    pshell_id=pshell_id+1;
    v1=grid_coord(1+cum_n_nodi(stat-1),:);                                      %Coordinate vertice 1 della sezione
    v3=grid_coord(2+dim(stat,1)+cum_n_nodi(stat-1),:);                          %Coordinate vertice 3 della sezione
    v4=grid_coord(3+dim(stat,1)+dim(stat,2)+cum_n_nodi(stat-1),:);              %Coordinate vertice 4 della sezione
    v2=grid_coord(4+dim(stat,1)+dim(stat,2)+dim(stat,3)+cum_n_nodi(stat-1),:);  %Coordinate vertice 2 della sezione
    vert1(count,:)=v1;vert2(count,:)=v2;vert3(count,:)=v3;vert4(count,:)=v4;
    count=count+1;
    centre=(v1+v2+v3+v4)/4;                                              %Coordinate del centro del box
    for k=nelem_rib-1:-1:1
        for w=1:sum(dim(stat,:))+4
            fraz=k/nelem_rib;
            node(idx_node,:)=fraz*[grid_coord(w+cum_n_nodi(stat-1),1)-centre(1) grid_coord(w+cum_n_nodi(stat-1),2)-centre(2) grid_coord(w+cum_n_nodi(stat-1),3)-centre(3)]+centre;
            idx_node=idx_node+1;
        end
    end
    if (i==fuse_sect)
      fuse_ID = [OFFSET+nnodes_tot+1:OFFSET+nnodes_tot+idx_node-1];
      i1 = 1; i2 = 2+dim(i,1); i3 = 2+dim(i,1)+dim(i,2)+1; i4 = 2+dim(i,1)+dim(i,2)+1+dim(i,3)+1;
      fuse_constr{1} = [fuse_ID(1), fuse_ID(i4+dim(i,4):-1:i4)];  
      fuse_constr{2} = fuse_ID(i2:i3);  
      spar_v(1,:) = node(i1,:);
      spar_v(2,:) = node(i4,:);
      spar_v(3,:) = node(i2,:);
      spar_v(4,:) = node(i3,:);
    end
    fout=fopen('rib.dat','a');
    for k=1:idx_node-1
        fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8d\n','GRID    ',OFFSET+nnodes_tot+k,0,node(k,1),node(k,2),node(k,3),0);    %Scrittura dei grid delle centine
    end
    fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8d\n','GRID    ',OFFSET+nnodes_tot+idx_node,0,centre(1),centre(2),centre(3),0);    %Scrittura del grid centrale
    rib_centre(end+1).grid=nnodes_tot+idx_node;
    rib_centre(end).coord=[centre(1) centre(2) centre(3)];
    id_centre=nnodes_tot+idx_node;
    nnodes_tot_start=nnodes_tot;
    nnodes_tot=nnodes_tot+idx_node;
    delta_node=(idx_node-1)/(nelem_rib-1);
    fclose(fout);

    id_seq=zeros(nelem_rib,delta_node);
    id_seq(1,:)=[1+cum_n_nodi(stat-1):3+dim(stat,1)+dim(stat,2)+cum_n_nodi(stat-1) ...
        fliplr(4+dim(stat,1)+dim(stat,2)+cum_n_nodi(stat-1):3+dim(stat,1)+dim(stat,2)+dim(stat,3)+cum_n_nodi(stat-1)) ...
        4+dim(stat,1)+dim(stat,2)+dim(stat,3)+cum_n_nodi(stat-1):4+dim(stat,1)+dim(stat,2)+dim(stat,3)+dim(stat,4)+cum_n_nodi(stat-1)];
    for k=2:nelem_rib
        id_seq(k,:)=[1+nnodes_tot_start+(k-2)*delta_node:3+dim(stat,1)+dim(stat,2)+nnodes_tot_start+(k-2)*delta_node ...
            fliplr(4+dim(stat,1)+dim(stat,2)+nnodes_tot_start+(k-2)*delta_node:3+dim(stat,1)+dim(stat,2)+dim(stat,3)+nnodes_tot_start+(k-2)*delta_node) ...
            4+dim(stat,1)+dim(stat,2)+dim(stat,3)+nnodes_tot_start+(k-2)*delta_node:4+dim(stat,1)+dim(stat,2)+dim(stat,3)+dim(stat,4)+nnodes_tot_start+(k-2)*delta_node];
    end
    %Scrittura dei quad4 della centina
    fout=fopen('rib.dat','a');
    for k=1:nelem_rib-1
        for w=1:delta_node-1
            fprintf(fout,'%8s%8d%8d%8d%8d%8d%8d\n','CQUAD4  ',nelem_tot+w+delta_node*(k-1),pshell_id,OFFSET+id_seq(k,w),OFFSET+id_seq(k,w+1),OFFSET+id_seq(k+1,w+1),OFFSET+id_seq(k+1,w));
        end
        w=delta_node;
        fprintf(fout,'%8s%8d%8d%8d%8d%8d%8d\n','CQUAD4  ',nelem_tot+w+delta_node*(k-1),pshell_id,OFFSET+id_seq(k,w),OFFSET+id_seq(k,1),OFFSET+id_seq(k+1,1),OFFSET+id_seq(k+1,w));        
    end

    nelem_tot=nelem_tot+(nelem_rib-1)*delta_node;
    %Scrittura dei tria3 della centina
    for w=1:delta_node-1
        fprintf(fout,'%8s%8d%8d%8d%8d%8d\n','CTRIA3  ',nelem_tot+w,pshell_id,OFFSET+id_seq(nelem_rib,w),OFFSET+id_seq(nelem_rib,w+1),OFFSET+id_centre);
    end
    w=delta_node;
    fprintf(fout,'%8s%8d%8d%8d%8d%8d\n','CTRIA3  ',nelem_tot+w,pshell_id,OFFSET+id_seq(nelem_rib,w),OFFSET+id_seq(nelem_rib,1),OFFSET+id_centre);
    fclose(fout);
    nelem_tot=nelem_tot+delta_node;
end

fout=fopen('rib.dat','a');
if material==1
    mat=2;
elseif material==2
    mat=3;
end
for i=20001:pshell_id
    fprintf(fout,'%8s%8d%8d%8.5f%8d%8s%8d%8s%8f\n','PSHELL  ',i,mat,rib_thick,1,'',1,'',0.0);
end
rib_id=20001:pshell_id;
n_rib=pshell_id-20000;
fclose(fout);



dtable_generator(vert1,vert2,vert3,vert4);