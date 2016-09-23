function[]=mat_generator(material)
%Funzione per la generazione del materiale

fout=fopen('mat.dat','w');
if material==1
    fprintf(fout,'MAT1           17.0E+0102.7E+010   0.3302.8E+003   1.000   1.000   1.000+MT    1\n');
    fprintf(fout,'+MT    1   1.000   1.000   1.000\n');
    fprintf(fout,'MAT1           2  1.E+133.76E+12    0.33   279.6      0.      0.  \n');
elseif material==2
    fprintf(fout,'MAT1           163.4E+0921.9e+09           1500.      1.      1.      1.+MT    1\n');
    fprintf(fout,'+MT    1      1.      1.      1.\n');
    fprintf(fout,'MAT8           21.50E+11 9.08E+9    0.32 5.29E+9      0.      0.   1500.+MT    2\n');
    fprintf(fout,'+MT    2      0.      0.      0. 1.45E+9  7.5E+8  6.3E+7 1.25E+8  6.7E+7+MA    2\n');
    fprintf(fout,'+MA    2                        \n');
    fprintf(fout,'MAT1           3  1.E+133.76E+12    0.33   279.6      0.      0. \n');
end
fclose(fout);
   

  