function[pshell_id]=pshell_generator(n_rib_station,start_val)
%Funzione per la scrittura delle PSHELL

n_bay_prop=n_rib_station-1;
fout=fopen('pshell.dat','w');
pshell_id=zeros(n_bay_prop*4,1);
count=1;
for i=1:n_bay_prop
    fprintf(fout,'%8s%8d%8d%8.5f%8d%8s%8d%8s%8f\n','PSHELL  ',1+(i-1)*10,1,start_val(i,1),1,'',1,'',0.0); 
    pshell_id(count)=1+(i-1)*10;count=count+1;
    fprintf(fout,'%8s%8d%8d%8.5f%8d%8s%8d%8s%8f\n','PSHELL  ',2+(i-1)*10,1,start_val(i,2),1,'',1,'',0.0); 
    pshell_id(count)=2+(i-1)*10;count=count+1;
    fprintf(fout,'%8s%8d%8d%8.5f%8d%8s%8d%8s%8f\n','PSHELL  ',3+(i-1)*10,1,start_val(i,1),1,'',1,'',0.0);
    pshell_id(count)=3+(i-1)*10;count=count+1;
    fprintf(fout,'%8s%8d%8d%8.5f%8d%8s%8d%8s%8f\n','PSHELL  ',4+(i-1)*10,1,start_val(i,2),1,'',1,'',0.0);
    pshell_id(count)=4+(i-1)*10;count=count+1;
end
fclose(fout);