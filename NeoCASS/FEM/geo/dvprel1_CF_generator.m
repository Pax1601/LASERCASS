function[]=dvprel1_CF_generator(section,n_rib_station)
%Funzione che scrive le dvprel1 del problema per i compositi

n_bay_prop=n_rib_station-1;
fout=fopen('dvprel1.dat','w');
sect=1;
for i=1:n_bay_prop                                                         %Scrittura delle dvprel1 per i pannelli ed i longheroni
    for j=1:4
        id2=j+(i-1)*10;
        dvid=20*section(sect).prop(1)+5*(j-1)+1;
        for k=1:4
            id=20*(i-1)+5*(j-1)+k;
            fprintf(fout,'%8s%8d%8s%8d%8s\n','DVPREL1 ',id,'PCOMP  ',id2,['T' num2str(k)]); 
            fprintf(fout,'%8s%8d%8.5f\n',' ',dvid,1.0); 
        end
    end
    if i==section(sect).prop(end)+1
        sect=sect+1;
    end  
end
sect=1;
for i=1:n_bay_prop                                                          %Scrittura delle dvprel1 per i correnti
    for j=1:6
        id=1000+(j-1)+(i-1)*10;
        fprintf(fout,'%8s%8d%8s%8d%8s\n','DVPREL1 ',id,'PBEAM   ',id,'A'); 
        fprintf(fout,'%8s%8d%8.5f\n',' ',999+j+section(sect).prop(1)*10,1.0); 
    end
    if i==section(sect).prop(end)+1
        sect=sect+1;
    end  
end
% for i=10001:n_rib+10000                                                     %Scrittura delle dvprel1 per le centine
%     id=i;
%     fprintf(fout,'%8s%8d%8s%8d%8s\n','DVPREL1 ',id,'PSHELL  ',id,'T'); 
%     fprintf(fout,'%8s%8d%8.5f\n',' ',id,1.0); 
% end
fclose(fout);