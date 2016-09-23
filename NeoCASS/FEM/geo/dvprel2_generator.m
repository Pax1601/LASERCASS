function[]=dvprel2_generator(section,n_rib_station)
%Funzione che scrive le dvprel2 del problema
%Vengono utilizzate solo per le pbeam

n_bay_prop=n_rib_station-1;
fout=fopen('dvprel2.dat','w');
%Scrittura delle equazioni per le inerzie delle varie forme di correnti
fprintf(fout,'%8s%8dIs(A)=0.413*(A**2)\n','DEQATN',1);                         %L'equazione 1 descrive Iyy e Izz delle solette ad L
fprintf(fout,'%8s%8dIyzs(A)=-0.24285*(A**2)\n','DEQATN',2);                    %L'equazione 2 descrive Iyz delle solette ad L
fprintf(fout,'%8s%8dJs(A)=0.020866*(A**2)\n','DEQATN',3);                      %L'equazione 3 descrive J delle solette ad L
fprintf(fout,'%8s%8dOFFs(A)=0.294*((A/0.2256)**0.5)\n','DEQATN',4);            %L'equazione 4 descrive l'offset delle solette ad L
fprintf(fout,'%8s%8dOFFs(A)=-0.294*((A/0.2256)**0.5)\n','DEQATN',11);            %L'equazione 4 descrive l'offset delle solette ad L

fprintf(fout,'%8s%8dIyyc(A)=0.5140*(A**2)\n','DEQATN',5);                       %L'equazione 5 descrive Iyy e Izz delle solette ad Z
fprintf(fout,'%8s%8dIzzc(A)=0.6601*(A**2)\n','DEQATN',6);                     %L'equazione 6 descrive Iyy e Izz delle solette ad Z
fprintf(fout,'%8s%8dIyzc(A)=-0.4563*(A**2)\n','DEQATN',7);                    %L'equazione 7 descrive Iyz delle solette ad Z
fprintf(fout,'%8s%8dJc(A)=0.0133*(A**2)\n','DEQATN',8);                      %L'equazione 8 descrive J delle solette ad Z
fprintf(fout,'%8s%8dOFFc(A)=0.62*((A/0.36)**0.5)\n','DEQATN',9);              %L'equazione 9 descrive l'offset delle solette ad Z
fprintf(fout,'%8s%8dOFFc(A)=-0.62*((A/0.36)**0.5)\n','DEQATN',10);              %L'equazione 9 descrive l'offset delle solette ad Z

id_desvar=zeros(1,6);idIyy=zeros(1,6);idIzz=zeros(1,6);idIyz=zeros(1,6);idJ=zeros(1,6);idOff=zeros(1,6);
sect=1;
for i=1:n_bay_prop                                                          %Scrittura delle desvar per i correnti
    for j=1:6
        id_desvar(j)=1000+(j-1)+(i-1)*10;
        idIyy(j)=2000+(j-1)+(i-1)*10;
        idIzz(j)=3000+(j-1)+(i-1)*10;
        idIyz(j)=4000+(j-1)+(i-1)*10;
        idJ(j)=5000+(j-1)+(i-1)*10;
        idOffy(j)=6000+(j-1)+(i-1)*10;
        idOffz(j)=7000+(j-1)+(i-1)*10;
    end
    for j=[1 3 4 6]
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idIyy(j),'PBEAM',id_desvar(j),5,'','',1);    %I1
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idIzz(j),'PBEAM',id_desvar(j),6,'','',1);    %I2
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idIyz(j),'PBEAM',id_desvar(j),7,'','',2);    %I12
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idJ(j),'PBEAM',id_desvar(j),8,'','',3);      %j
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        if j==1 || j==4
            fprintf(fout,'%8s%8d%8s%8d%8s%8s%8s%8d\n','DVPREL2',idOffy(j),'PBEAM',id_desvar(j),'N1(A)','','',4);      
            fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1)); 
            fprintf(fout,'%8s%8d%8s%8d%8s%8s%8s%8d\n','DVPREL2',idOffz(j),'PBEAM',id_desvar(j),'N2(A)','','',4);      
            fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        elseif j==3 || j==6
            fprintf(fout,'%8s%8d%8s%8d%8s%8s%8s%8d\n','DVPREL2',idOffy(j),'PBEAM',id_desvar(j),'N1(A)','','',4);      
            fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1)); 
            fprintf(fout,'%8s%8d%8s%8d%8s%8s%8s%8d\n','DVPREL2',idOffz(j),'PBEAM',id_desvar(j),'N2(A)','','',11);      
            fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));    
        end
    end
    for j=[2 5]
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idIyy(j),'PBEAM',id_desvar(j),5,'','',5);    %I1
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idIzz(j),'PBEAM',id_desvar(j),6,'','',6);    %I2
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idIyz(j),'PBEAM',id_desvar(j),7,'','',7);    %I12
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));
        fprintf(fout,'%8s%8d%8s%8d%8d%8s%8s%8d\n','DVPREL2',idJ(j),'PBEAM',id_desvar(j),8,'','',8);      %j
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1));   
        if j==2
            fprintf(fout,'%8s%8d%8s%8d%8s%8s%8s%8d\n','DVPREL2',idOffz(j),'PBEAM',id_desvar(j),'N2(A)','','',9);     
            fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1)); 
        else
            fprintf(fout,'%8s%8d%8s%8d%8s%8s%8s%8d\n','DVPREL2',idOffz(j),'PBEAM',id_desvar(j),'N2(A)','','',10);     
            fprintf(fout,'%8s%8s%8d\n','','DESVAR',1000+(j-1)+10*section(sect).prop(1)); 
        end
    end
    if i==section(sect).prop(end)+1
        sect=sect+1;
    end  
end
fclose(fout);