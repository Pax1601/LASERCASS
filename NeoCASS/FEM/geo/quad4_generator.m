function[n_baie,nelem_tot]=quad4_generator(grid_coord,size_sect_nodes,size_rib_nodes,nelem_rib_tra,stretch,nrib_span)
%Funzione che genera gli elementi quad4 di nastran
OFFSET = 1999999;
n_baie=(sum(nrib_span)+length(nrib_span))*nelem_rib_tra;
idx_size=1;
dim(idx_size,:)=size_sect_nodes{1};
idx_size=idx_size+1;
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

idx_el=1;
for i=1:n_baie
    %Pannello superiore
    if dim(i+1,1)==dim(i,1)
        for j=1:dim(i+1,1)+1
            if i==1
                elem(idx_el,:)=[j j+cum_n_nodi(i) j+1+cum_n_nodi(i) j+1];
                prop(idx_el)=1+(ceil(i/nelem_rib_tra)-1)*10;
                tipo(idx_el)=1;                                                 %QUAD4
                idx_el=idx_el+1;
            else
                elem(idx_el,:)=[j+cum_n_nodi(i-1) j+cum_n_nodi(i) j+1+cum_n_nodi(i) j+1+cum_n_nodi(i-1)];
                prop(idx_el)=1+(ceil(i/nelem_rib_tra)-1)*10;
                tipo(idx_el)=1;
                idx_el=idx_el+1;
            end
        end
    else
        for j=1:dim(i+1,1)+1
            if i==1
                elem(idx_el,:)=[j j+cum_n_nodi(i) j+1+cum_n_nodi(i) j+1];
                prop(idx_el)=1+(ceil(i/nelem_rib_tra)-1)*10;
                tipo(idx_el)=1;
                idx_el=idx_el+1;
            else
                elem(idx_el,:)=[j+cum_n_nodi(i-1) j+cum_n_nodi(i) j+1+cum_n_nodi(i) j+1+cum_n_nodi(i-1)];
                prop(idx_el)=1+(ceil(i/nelem_rib_tra)-1)*10;
                tipo(idx_el)=1;
                idx_el=idx_el+1;
            end
        end
        for k=1:(dim(i,1)-dim(i+1,1))
            j=k+dim(i+1,1)+1;
            if i==1
                elem(idx_el,:)=[j dim(i+1,1)+2+cum_n_nodi(i) j+1 0];
                prop(idx_el)=1+(ceil(i/nelem_rib_tra)-1)*10;
                tipo(idx_el)=2;                                                      %TRIA3
                idx_el=idx_el+1;
            else
                elem(idx_el,:)=[j+cum_n_nodi(i-1) dim(i+1,1)+2+cum_n_nodi(i) j+1+cum_n_nodi(i-1) 0];
                prop(idx_el)=1+(ceil(i/nelem_rib_tra)-1)*10;
                tipo(idx_el)=2;
                idx_el=idx_el+1;
            end 
        end
    end
    %Longherone posteriore
    for j=1:dim(i,2)+1
        k=1+dim(i,1)+j;                                                         %Sezione i
        w=1+dim(i+1,1)+j;                                                       %Sezione i+1
        if i==1
            elem(idx_el,:)=[k w+cum_n_nodi(i) w+1+cum_n_nodi(i) k+1];
            prop(idx_el)=2+(ceil(i/nelem_rib_tra)-1)*10;
            tipo(idx_el)=1;                                                     %QUAD4
            idx_el=idx_el+1;
        else
            elem(idx_el,:)=[k+cum_n_nodi(i-1) w+cum_n_nodi(i) w+1+cum_n_nodi(i) k+1+cum_n_nodi(i-1)];
            prop(idx_el)=2+(ceil(i/nelem_rib_tra)-1)*10;
            tipo(idx_el)=1;
            idx_el=idx_el+1;
        end
    end
    %Pannello inferiore
    if i==1
        ID_a=[3+dim(i,1)+dim(i,2) (3+dim(i,1)+dim(i,2)+dim(i,3):-1:4+dim(i,1)+dim(i,2)) 4+dim(i,1)+dim(i,2)+dim(i,3)];
        ID_b=[3+dim(i+1,1)+dim(i+1,2)+cum_n_nodi(i) (3+dim(i+1,1)+dim(i+1,2)+dim(i+1,3):-1:4+dim(i+1,1)+dim(i+1,2))+cum_n_nodi(i) 4+dim(i+1,1)+dim(i+1,2)+dim(i+1,3)+cum_n_nodi(i)];
    else
        ID_a=[3+dim(i,1)+dim(i,2)+cum_n_nodi(i-1) (3+dim(i,1)+dim(i,2)+dim(i,3):-1:4+dim(i,1)+dim(i,2))+cum_n_nodi(i-1) 4+dim(i,1)+dim(i,2)+dim(i,3)+cum_n_nodi(i-1)];
        ID_b=[3+dim(i+1,1)+dim(i+1,2)+cum_n_nodi(i) (3+dim(i+1,1)+dim(i+1,2)+dim(i+1,3):-1:4+dim(i+1,1)+dim(i+1,2))+cum_n_nodi(i) 4+dim(i+1,1)+dim(i+1,2)+dim(i+1,3)+cum_n_nodi(i)];
    end
    if dim(i+1,3)==dim(i,3)
        for j=1:dim(i,3)+1
            elem(idx_el,:)=[ID_a(j) ID_b(j) ID_b(j+1) ID_a(j+1)];
            prop(idx_el)=3+(ceil(i/nelem_rib_tra)-1)*10;
            tipo(idx_el)=1;
            idx_el=idx_el+1;
        end
    else
        for k=1:(dim(i,3)-dim(i+1,3))
            elem(idx_el,:)=[ID_a(k) ID_b(1) ID_a(k+1) 0];
            prop(idx_el)=3+(ceil(i/nelem_rib_tra)-1)*10;
            tipo(idx_el)=2;
            idx_el=idx_el+1;  
        end
        difference=(dim(i,3)-dim(i+1,3));
        for j=1:dim(i+1,3)+1
            elem(idx_el,:)=[ID_a(j+difference) ID_b(j) ID_b(j+1) ID_a(j+difference+1)];
            prop(idx_el)=3+(ceil(i/nelem_rib_tra)-1)*10;
            tipo(idx_el)=1;
            idx_el=idx_el+1;
        end 

    end
    %Longherone anteriore
    for j=1:dim(i,4)
        k=3+dim(i,1)+dim(i,2)+dim(i,3)+j;                                       %Sezione i
        w=3+dim(i+1,1)+dim(i+1,2)+dim(i+1,3)+j;                                 %Sezione i+1
        if i==1
            elem(idx_el,:)=[k w+cum_n_nodi(i) w+1+cum_n_nodi(i) k+1];
            prop(idx_el)=4+(ceil(i/nelem_rib_tra)-1)*10;
            tipo(idx_el)=1;                                                     %QUAD4
            idx_el=idx_el+1;
        else
            elem(idx_el,:)=[k+cum_n_nodi(i-1) w+cum_n_nodi(i) w+1+cum_n_nodi(i) k+1+cum_n_nodi(i-1)];
            prop(idx_el)=4+(ceil(i/nelem_rib_tra)-1)*10;
            tipo(idx_el)=1;
            idx_el=idx_el+1;
        end
    end
    k=k+1;
    w=w+1;
    if i==1
        elem(idx_el,:)=[k w+cum_n_nodi(i) 1+cum_n_nodi(i) 1];
        prop(idx_el)=4+(ceil(i/nelem_rib_tra)-1)*10;
        tipo(idx_el)=1;                                                     %QUAD4
        idx_el=idx_el+1;        
    else
        elem(idx_el,:)=[k+cum_n_nodi(i-1) w+cum_n_nodi(i) 1+cum_n_nodi(i) 1+cum_n_nodi(i-1)];
        prop(idx_el)=4+(ceil(i/nelem_rib_tra)-1)*10;
        tipo(idx_el)=1;                                                     %QUAD4
        idx_el=idx_el+1;          
    end
end




fout=fopen('element.dat','w');
for i=1:size(elem,1)
    if tipo(i)==1
        fprintf(fout,'%8s%8d%8d%8d%8d%8d%8d\n','CQUAD4  ',i,prop(i),OFFSET + elem(i,1),OFFSET + elem(i,2),OFFSET + elem(i,3),OFFSET + elem(i,4));
    elseif tipo(i)==2
        fprintf(fout,'%8s%8d%8d%8d%8d%8d\n','CTRIA3  ',i,prop(i),OFFSET + elem(i,1),OFFSET + elem(i,2),OFFSET + elem(i,3));
    end
end
nelem_tot=size(elem,1);
fclose(fout);
