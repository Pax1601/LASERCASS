function[grid_coord,spc_nodes,vert1,vert2,vert3,vert4]=grid_generator(coord_sect_nodes,coord_nodes,nelem_rib_tra,stretch,nrib_span,size_sect_nodes,size_rib_nodes,vert1,vert2,vert3,vert4)
%Funzione che scrive le coordinate dei nodi con la formattazione Nastran
OFFSET = 1999999;

%grid_coord,n_tot_station,n_nodes

fout=fopen('grid.dat','w');
count=1;
count2_1=1;count2_2=1;count2_3=1;count2_4=1;
for i=1:sum(size_sect_nodes{1})+4
    fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8d\n','GRID    ',OFFSET+count,0,coord_sect_nodes{1}(i,1),coord_sect_nodes{1}(i,2),coord_sect_nodes{1}(i,3),0);
    
    if (coord_sect_nodes{1}(i,1)==vert1(count2_1).coord(1))&&(coord_sect_nodes{1}(i,2)==vert1(count2_1).coord(2))&&(coord_sect_nodes{1}(i,3)==vert1(count2_1).coord(3))
        vert1(count2_1).grid=count;
        count2_1=count2_1+1;
    end
    if (coord_sect_nodes{1}(i,1)==vert2(count2_2).coord(1))&&(coord_sect_nodes{1}(i,2)==vert2(count2_2).coord(2))&&(coord_sect_nodes{1}(i,3)==vert2(count2_2).coord(3))
        vert2(count2_2).grid=count;
        count2_2=count2_2+1;
    end    
    if (coord_sect_nodes{1}(i,1)==vert3(count2_3).coord(1))&&(coord_sect_nodes{1}(i,2)==vert3(count2_3).coord(2))&&(coord_sect_nodes{1}(i,3)==vert3(count2_3).coord(3))
        vert3(count2_3).grid=count;
        count2_3=count2_3+1;
    end   
    if (coord_sect_nodes{1}(i,1)==vert4(count2_4).coord(1))&&(coord_sect_nodes{1}(i,2)==vert4(count2_4).coord(2))&&(coord_sect_nodes{1}(i,3)==vert4(count2_4).coord(3))
        vert4(count2_4).grid=count;
        count2_4=count2_4+1;
    end  
    
    grid_coord(count,:)=[coord_sect_nodes{1}(i,1),coord_sect_nodes{1}(i,2),coord_sect_nodes{1}(i,3)];
    count=count+1;
end
spc_nodes=1:(count-1);
for i=1:stretch
    for j=1:nrib_span(i)+1
        for p=1:nelem_rib_tra
            a=coord_nodes{i}{j}{p}(:,1);b=coord_nodes{i}{j}{p}(:,2);c=coord_nodes{i}{j}{p}(:,3);
            for z=1:length(a)
                fprintf(fout,'%8s%8d%8d%8.4f%8.4f%8.4f%8d\n','GRID    ',OFFSET + count,0,a(z),b(z),c(z),0);
                                
                if (a(z)==vert1(count2_1).coord(1))&&(b(z)==vert1(count2_1).coord(2))&&(c(z)==vert1(count2_1).coord(3))
                    vert1(count2_1).grid=count;
                    if count2_1<length(vert1)
                        count2_1=count2_1+1;
                    end
                end
                if (a(z)==vert2(count2_2).coord(1))&&(b(z)==vert2(count2_2).coord(2))&&(c(z)==vert2(count2_2).coord(3))
                    vert2(count2_2).grid=count;
                    if count2_2<length(vert1)
                        count2_2=count2_2+1;
                    end
                end    
                if (a(z)==vert3(count2_3).coord(1))&&(b(z)==vert3(count2_3).coord(2))&&(c(z)==vert3(count2_3).coord(3))
                    vert3(count2_3).grid=count;
                    if count2_3<length(vert1)
                        count2_3=count2_3+1;
                    end
                end   
                if (a(z)==vert4(count2_4).coord(1))&&(b(z)==vert4(count2_4).coord(2))&&(c(z)==vert4(count2_4).coord(3))
                    vert4(count2_4).grid=count;
                    if count2_4<length(vert1)
                        count2_4=count2_4+1;
                    end
                end 
                
                grid_coord(count,:)=[a(z),b(z),c(z)];
                count=count+1; 
            end
        end
    end
end
fclose(fout);
