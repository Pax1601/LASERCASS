function[]=desvar_generator(section,start_val)
%Funzione che scrive le desvar del problema

fout=fopen('desvar.dat','w');
bound=[5e-4 25e-3                                                           %Boundaries del pannello superiore
       5e-4 25e-3                                                           %Boundaries del longherone posteriore
       5e-4 25e-3                                                           %Boundaries del pannello inferiore
       5e-4 25e-3                                                           %Boundaries del longherone anteriore
       1e-5 1e-2                                                            %Boundaries soletta 1
       1e-5 1e-2                                                            %Boundaries correnti superiori
       1e-5 1e-2                                                            %Boundaries soletta 3
       1e-5 1e-2                                                            %Boundaries soletta 4
       1e-5 1e-2                                                            %Boundaries correnti inferiori
       1e-5 1e-2                                                            %Boundaries soletta 2
       5e-4 25e-3];                                                         %Boundaries centine
for i=1:length(section)                                                     %Scrittura delle desvar per i pannelli ed i longheroni
    for j=1:4
        id=j+section(i).prop(1)*10;
        if id<10
            lab=['shell00' num2str(id)];
        elseif id<100
            lab=['shell0' num2str(id)];
        else
            lab=['shell' num2str(id)];
        end
        if j==1 || j==3
            fprintf(fout,'%8s%8d%8s%8.5f%8.5f%8.5f\n','DESVAR  ',id,lab,max(start_val(section(i).prop+1,1)),bound(j,1),bound(j,2)); 
        elseif j==2 || j==4
            fprintf(fout,'%8s%8d%8s%8.5f%8.5f%8.5f\n','DESVAR  ',id,lab,max(start_val(section(i).prop+1,2)),bound(j,1),bound(j,2)); 
        end
    end
end
for i=1:length(section)                                                     %Scrittura delle desvar per i correnti
    for j=1:6
        id=1000+(j-1)+section(i).prop(1)*10;
        lab=['beam' num2str(id)];
        fprintf(fout,'%8s%8d%8s%8.5f%8.5f%8.5f\n','DESVAR  ',id,lab,max(start_val(section(i).prop+1,3)),bound(4+j,1),bound(4+j,2));
    end
end
% for i=10001:n_rib+10000                                                     %Scrittura delle desvar per le centine
%     id=i;
%     lab=['rib' num2str(id)];
%     fprintf(fout,'%8s%8d%8s%8.5f%8.5f%8.5f\n','DESVAR  ',id,lab,2e-3,bound(11,1),bound(11,2)); 
% end
fclose(fout);