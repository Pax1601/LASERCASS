function[pcomp_id]=pcomp_generator(n_rib_station)
%Funzione per la scrittura delle PCOMP

n_bay_prop=n_rib_station-1;
fout=fopen('pcomp.dat','w');                                                    %Apertura del file con le proprietà del laminato 
pcomp_id=zeros(n_bay_prop*4,1);
th_lamina=0.13e-3;
gradi=[0 45 -45 90];
idx=1;
for i=1:n_bay_prop
    fprintf(fout,'PCOMP   %8d              0.   1000.    HILL                     SYM+\n',1+(i-1)*10);

    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(1),th_lamina,gradi(2));
    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(3),th_lamina,gradi(4));
    pcomp_id(idx)=1+(i-1)*10;idx=idx+1;
    fprintf(fout,'PCOMP   %8d              0.   1000.    HILL                     SYM+\n',2+(i-1)*10);
    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(1),th_lamina,gradi(2));
    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(3),th_lamina,gradi(4));
    pcomp_id(idx)=2+(i-1)*10;idx=idx+1;
    fprintf(fout,'PCOMP   %8d              0.   1000.    HILL                     SYM+\n',3+(i-1)*10);
    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(1),th_lamina,gradi(2));
    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(3),th_lamina,gradi(4));
    pcomp_id(idx)=3+(i-1)*10;idx=idx+1;
    fprintf(fout,'PCOMP   %8d              0.   1000.    HILL                     SYM+\n',4+(i-1)*10);
    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(1),th_lamina,gradi(2));
    fprintf(fout,'               2%8f%7d.     YES       2%8f%7d.     YES\n',th_lamina,gradi(3),th_lamina,gradi(4));
    pcomp_id(idx)=4+(i-1)*10;idx=idx+1;
end
fclose(fout);



