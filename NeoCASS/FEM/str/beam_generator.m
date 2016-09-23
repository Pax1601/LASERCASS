function[n_tot_elem]=beam_generator(coord_sect_nodes,coord_rib_nodes,size_sect_nodes,size_rib_nodes,stretch,nrib_span,nelem_tra,n_tot_elem,nelem_rib_tra,Area)
%Funzione che genera gli elementi cbeam di nastran
OFFSET = 1999999;
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

idx_el=1;
for i=1:n_baie
    if i==1
        beam(idx_el,:)=[1 1+cum_n_nodi(i) 2];                                   %Soletta 1                                
        prop(idx_el)=1000+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=1;
        idx_el=idx_el+1;
        for j=nelem_tra:nelem_tra:dim(i+1,1)
            k=j+1;                                                              %Sezione i
            w=j+cum_n_nodi(i)+1;                                                %Sezione i+1
            beam(idx_el,:)=[k w k+1];                    
            prop(idx_el)=1001+(ceil(i/nelem_rib_tra)-1)*10;
            A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
            tipo(idx_el)=2;
            idx_el=idx_el+1;           
        end 
        beam(idx_el,:)=[2+dim(i,1) 2+dim(i+1,1)+cum_n_nodi(i) 1+dim(i,1)];      %Soletta 3
        prop(idx_el)=1002+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=3;
        idx_el=idx_el+1;
        beam(idx_el,:)=[3+dim(i,1)+dim(i,2) 3+dim(i+1,1)+dim(i+1,2)+cum_n_nodi(i) 3+dim(i,1)+dim(i,2)+dim(i,3)]; %Soletta 4
        prop(idx_el)=1003+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=4;
        idx_el=idx_el+1;
        vett=nelem_tra:nelem_tra:dim(i+1,3);
        for j=vett
            k=j+3+dim(i,1)+dim(i,2);                                            %Sezione i
            w=j+cum_n_nodi(i)+3+dim(i+1,1)+dim(i+1,2);                          %Sezione i+1
            beam(idx_el,:)=[k w k+1];
            if j==vett(end)
                beam(idx_el,:)=[k w 3+dim(i,1)+dim(i,2)];
            end
            prop(idx_el)=1004+(ceil(i/nelem_rib_tra)-1)*10;
            A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
            tipo(idx_el)=5;
            idx_el=idx_el+1;           
        end
        beam(idx_el,:)=[4+dim(i,1)+dim(i,2)+dim(i,3) 4+dim(i+1,1)+dim(i+1,2)+dim(i+1,3)+cum_n_nodi(i) 4+dim(i,1)+dim(i,2)]; %Soletta 2
        prop(idx_el)=1005+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=6;
        idx_el=idx_el+1;
    else
        beam(idx_el,:)=[1+cum_n_nodi(i-1) 1+cum_n_nodi(i) 2+cum_n_nodi(i-1)];   %Soletta 1
        prop(idx_el)=1000+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=1;
        idx_el=idx_el+1;
        for j=nelem_tra:nelem_tra:dim(i+1,1)
            k=j+cum_n_nodi(i-1)+1;                                              %Sezione i
            w=j+cum_n_nodi(i)+1;                                                %Sezione i+1
            beam(idx_el,:)=[k w k+1];                    
            prop(idx_el)=1001+(ceil(i/nelem_rib_tra)-1)*10;
            A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
            tipo(idx_el)=2;
            idx_el=idx_el+1;           
        end
        beam(idx_el,:)=[2+dim(i,1)+cum_n_nodi(i-1) 2+dim(i+1,1)+cum_n_nodi(i) 1+dim(i,1)+cum_n_nodi(i-1)]; %Soletta 3
        prop(idx_el)=1002+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=3;
        idx_el=idx_el+1;
        beam(idx_el,:)=[3+dim(i,1)+dim(i,2)+cum_n_nodi(i-1) 3+dim(i+1,1)+dim(i+1,2)+cum_n_nodi(i) 3+dim(i,1)+dim(i,2)+dim(i,3)+cum_n_nodi(i-1)]; %Soletta 4
        prop(idx_el)=1003+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=4;
        idx_el=idx_el+1;
        vett=nelem_tra:nelem_tra:dim(i+1,3);
        for j=vett
            k=j+cum_n_nodi(i-1)+3+dim(i,1)+dim(i,2);                            %Sezione i
            w=j+cum_n_nodi(i)+3+dim(i+1,1)+dim(i+1,2);                          %Sezione i+1
            beam(idx_el,:)=[k w k+1];
            if j==vett(end)
                beam(idx_el,:)=[k w 3+dim(i,1)+dim(i,2)+cum_n_nodi(i-1)];
            end
            prop(idx_el)=1004+(ceil(i/nelem_rib_tra)-1)*10;
            A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
            tipo(idx_el)=5;
            idx_el=idx_el+1;           
        end
        beam(idx_el,:)=[4+dim(i,1)+dim(i,2)+dim(i,3)+cum_n_nodi(i-1) 4+dim(i+1,1)+dim(i+1,2)+dim(i+1,3)+cum_n_nodi(i) 4+dim(i,1)+dim(i,2)+cum_n_nodi(i-1)]; %Soletta 2
        prop(idx_el)=1005+(ceil(i/nelem_rib_tra)-1)*10;
        A_pbeam(idx_el)=Area(ceil(i/nelem_rib_tra));
        tipo(idx_el)=6;
        idx_el=idx_el+1;
    end   
end

fout=fopen('beam.dat','w');
for i=1:length(beam)
    fprintf(fout,'%8s%8d%8d%8d%8d%8d\n','CBEAM   ',n_tot_elem+i,prop(i),OFFSET+beam(i,1),OFFSET+beam(i,2),OFFSET+beam(i,3));
%     if tipo(i)==1
%         fprintf(fout,'%8s%8s%8s%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n','','','',0.294*sqrt(A_pbeam(i)/0.2256),0.0,-0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),0.0,-0.294*sqrt(A_pbeam(i)/0.2256));
%     elseif tipo(i)==2
%         fprintf(fout,'%8s%8s%8s%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n','','','',0.0,0.0,-0.62*sqrt(A_pbeam(i)/0.36),0.0,0.0,-0.62*sqrt(A_pbeam(i)/0.36));  
%     elseif tipo(i)==3
%         fprintf(fout,'%8s%8s%8s%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n','','','',-0.294*sqrt(A_pbeam(i)/0.2256),0.0,-0.294*sqrt(A_pbeam(i)/0.2256),-0.294*sqrt(A_pbeam(i)/0.2256),0.0,-0.294*sqrt(A_pbeam(i)/0.2256));        
%     elseif tipo(i)==4
%         fprintf(fout,'%8s%8s%8s%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n','','','',-0.294*sqrt(A_pbeam(i)/0.2256),0.0,0.294*sqrt(A_pbeam(i)/0.2256),-0.294*sqrt(A_pbeam(i)/0.2256),0.0,0.294*sqrt(A_pbeam(i)/0.2256));        
%     elseif tipo(i)==5
%         fprintf(fout,'%8s%8s%8s%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n','','','',0.0,0.0,0.62*sqrt(A_pbeam(i)/0.36),0.0,0.0,0.62*sqrt(A_pbeam(i)/0.36));        
%     elseif tipo(i)==6
%         fprintf(fout,'%8s%8s%8s%8.4f%8.4f%8.4f%8.4f%8.4f%8.4f\n','','','',0.294*sqrt(A_pbeam(i)/0.2256),0.0,0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),0.0,0.294*sqrt(A_pbeam(i)/0.2256));        
%     end
end
n_tot_elem=n_tot_elem+length(beam);
fclose(fout);



