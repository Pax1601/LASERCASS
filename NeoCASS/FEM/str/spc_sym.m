function[]=spc_sym(spc_nodes)
%Funzione per la scittura automatica dei vincoli di simmetria sulla prima centina
OFFSET = 1999999;
fout=fopen('spc_sym.dat','w');
for i=1:length(spc_nodes)
    fprintf(fout,'%8s%8d%8d%8d%8.1f\n','SPC     ',1,spc_nodes(i)+OFFSET,246,0.);
end
fclose(fout);