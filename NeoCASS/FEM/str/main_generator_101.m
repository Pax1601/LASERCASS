function[]=main_generator_101(outname, material)
%Funzione che scrive il main dell'analisi nastran
fout=fopen(outname,'w');
fprintf(fout,'SOL    101\n');
fprintf(fout,'TIME   1000\n');
fprintf(fout,'CEND\n');
fprintf(fout,'LINE = 999999\n');
fprintf(fout,'SPCFORCES=ALL\n');
fprintf(fout,'ELFORCE=ALL\n');
fprintf(fout,'TITLE = MSC/NASTRAN : 3D wing box static analysis\n');
fprintf(fout,'ECHO=PUNCH(NEWBULK)\n');
fprintf(fout,'SPC = 1\n');
fprintf(fout,'LOAD = 1\n');
fprintf(fout,'ELSTRESS(CENTER)=ALL\n');
if material==2
    fprintf(fout,'STRAIN(CENTER)=ALL\n');
end
fprintf(fout,'DISPLACEMENT=ALL\n');
fprintf(fout,'BEGIN BULK\n');
fprintf(fout,'PARAM,GRDPNT,0\n');
fprintf(fout,'INCLUDE ''grid.dat''\n');
fprintf(fout,'INCLUDE ''element.dat''\n');
fprintf(fout,'INCLUDE ''beam.dat''\n');
if material==1
    fprintf(fout,'INCLUDE ''pshell.dat''\n');
elseif material==2
    fprintf(fout,'INCLUDE ''pcomp.dat''\n');
end
fprintf(fout,'INCLUDE ''mat.dat''\n');
fprintf(fout,'INCLUDE ''rib.dat''\n');
fprintf(fout,'INCLUDE ''RBE3.dat''\n');
fprintf(fout,'INCLUDE ''spc_fuse.dat''\n');
fprintf(fout,'INCLUDE ''spc_sym.dat''\n');
fprintf(fout,'INCLUDE ''pbeam.dat''\n');
fprintf(fout,'ENDDATA');
fclose(fout);