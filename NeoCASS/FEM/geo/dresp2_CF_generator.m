function[]=dresp2_CF_generator(section)
%Funzione per la generazione delle dresp1 di nastran

fout=fopen('dresp2.dat','w');
fprintf(fout,'DEQATN       200th(a)=8.*a\n');
%Generazione delle dresp2
for i=1:length(section)
    for j=1:4
        id=6000+10*section(i).prop(1)+j;
        fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',id,'a',200);
        fprintf(fout,'%8s%8s%8d\n','','DESVAR',1+(j-1)*5+20*section(i).prop(1));
    end
end
fclose(fout);
