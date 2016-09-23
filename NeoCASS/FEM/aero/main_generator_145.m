function[]=main_generator_145(outname, material)
%Funzione che scrive il main dell'analisi nastran
fout=fopen(outname,'w');
fprintf(fout,'SOL    145\n');
fprintf(fout,'TIME   1000\n');
fprintf(fout,'CEND\n');
fprintf(fout,'LINE = 999999\n');
fprintf(fout,'SPCFORCES=ALL\n');
fprintf(fout,'ELFORCE=ALL\n');
fprintf(fout,'APRES=ALL\n');
fprintf(fout,'AEROF=ALL\n');
fprintf(fout,'TITLE = MSC/NASTRAN : 3D wing box static aeroelasticity\n');
fprintf(fout,'ECHO=PUNCH(NEWBULK)\n');
fprintf(fout,'SPC = 1\n');
fprintf(fout,'METHOD = 1\n');
fprintf(fout,'SVEC = ALL\n');
fprintf(fout,'FMETHOD = 10\n');
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
fprintf(fout,'INCLUDE ''caero.dat''\n');
%
fprintf(fout,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fout,'$ MODIFY THE FOLLOWING CARDS for flutter solution\n$\n');
fprintf(fout,'$ Update vibration modes parameters\n');
fprintf(fout,'EIGRL   1                       10\n');
fprintf(fout,'$ Update number of modes to use\n');
fprintf(fout,'PARAM,LMODES,10\n');
fprintf(fout,'$ Update Mach number and reduced frequencies\n');
fprintf(fout,'MKAERO1 0.70\n');   
fprintf(fout,'        0.001   0.01    0.05    0.1     0.2     0.5     0.7     1.0 \n');
fprintf(fout,'FLUTTER 10      PK      10      20      30\n');
fprintf(fout,'$ Update density ratio\n');
fprintf(fout,'FLFACT  10      1.0\n');
fprintf(fout,'$ Update analysis Mach number\n');
fprintf(fout,'FLFACT  20      0.7\n');
fprintf(fout,'$ Update field velocity\n');
fprintf(fout,'FLFACT  30      10.0    THRU    300.0   9\n');
fprintf(fout,'$ Update reference chord and density\n');
fprintf(fout,'AERO    0               1.0     1.225   1\n');
fprintf(fout,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
%
fprintf(fout,'ENDDATA');
fclose(fout);