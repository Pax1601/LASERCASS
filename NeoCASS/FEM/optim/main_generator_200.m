function[]=main_generator_200(outname, material)
%Funzione che scrive il main dell'analisi nastran
fout=fopen(outname,'w');
fprintf(fout,'SOL    200\n');
fprintf(fout,'TIME   1000\n');
fprintf(fout,'CEND\n');
fprintf(fout,'LINE = 999999\n');
fprintf(fout,'SPCFORCES=ALL\n');
fprintf(fout,'ELFORCE=ALL\n');
fprintf(fout,'TITLE = MSC/NASTRAN : 3D wing box optimization\n');
fprintf(fout,'ECHO=PUNCH(NEWBULK)\n');
fprintf(fout,'DESOBJ(MIN) = 1000000\n');
fprintf(fout,'ANALYSIS = STATICS\n');
fprintf(fout,'SUBCASE 1\n');
fprintf(fout,'SPC = 1\n');
fprintf(fout,'LOAD = 1\n');
fprintf(fout,'DESSUB = 1\n');
fprintf(fout,'ELSTRESS(CENTER)=ALL\n');
if material==2
    fprintf(fout,'STRAIN(CENTER)=ALL\n');
end
fprintf(fout,'DISPLACEMENT=ALL\n');
%fprintf(fout,'SUBCASE 2\n');
%fprintf(fout,'SPC = 1\n');
%fprintf(fout,'LOAD = 5\n');
%fprintf(fout,'DESSUB = 1\n');
%fprintf(fout,'ELSTRESS(CENTER)=ALL\n');
%if material==2
%    fprintf(fout,'STRAIN(CENTER)=ALL\n');
%end
%fprintf(fout,'DISPLACEMENT=ALL\n');
fprintf(fout,'BEGIN BULK\n');
fprintf(fout,'PARAM,GRDPNT,0\n');
fprintf(fout,'DSCREEN STRESS  -0.5    3\n');
fprintf(fout,'DSCREEN EQUA    -0.5    3\n');
fprintf(fout,'DOPTPRM DESMAX  50      DELP    5.0e-2  DPMIN   5.0e-4  DELX    2.5e-2\n');
fprintf(fout,'        DXMIN   2.5e-2  P1      1       P2      13      P2CALL  150\n');
fprintf(fout,'        APRCOD  1\n');
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
fprintf(fout,'INCLUDE ''desvar.dat''\n');
fprintf(fout,'INCLUDE ''dvprel1.dat''\n');
fprintf(fout,'INCLUDE ''dvprel2.dat''\n');
fprintf(fout,'INCLUDE ''dresp1.dat''\n');
if material==2
    fprintf(fout,'INCLUDE ''dresp2.dat''\n');    
end
fprintf(fout,'INCLUDE ''dconstr.dat''\n');
fprintf(fout,'INCLUDE ''analytical_buckling.dat''\n');
fprintf(fout,'INCLUDE ''pbeam.dat''\n');
fprintf(fout,'INCLUDE ''dtable.dat''\n');
fprintf(fout,'ENDDATA');
fclose(fout);