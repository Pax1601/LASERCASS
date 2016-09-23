function[]=dconstr_generator(section)
%Funzione che genera le DCONSTR di matlab

last=section(end).prop(end);

fout=fopen('dconstr.dat','w');
fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000001,1000001,'-2.33e+8','2.33e+8');
fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000002,1000002,'-2.33e+8','2.33e+8');
fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000003,1000003,'-2.33e+8','2.33e+8');
fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000004,1000004,'-2.33e+8','2.33e+8');
fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000005,1000005,'-2.33e+8','2.33e+8');
fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000006,1000006,'-2.33e+8','2.33e+8');
fprintf(fout,'%8s%8d%8d%8d%8d%8d%8d%8d','DCONADD',1,1000001,1000002,1000003,1000004,1000005,1000006);
k=7;
for i=0:last
    k=k+1;
    fprintf(fout,'%8d',1+10*i);
    if k==8
        fprintf(fout,'\n        ');
        k=0;
    end
end
for i=0:last
    k=k+1;
    fprintf(fout,'%8d',3+10*i);
    if k==8
        fprintf(fout,'\n        ');
        k=0;
    end
end
for i=0:last
    k=k+1;
    fprintf(fout,'%8d',1001+10*i);
    if k==8
        fprintf(fout,'\n        ');
        k=0;
    end
end
for i=0:last
    k=k+1;
    fprintf(fout,'%8d',1002+10*i);
    if k==8
        fprintf(fout,'\n        ');
        k=0;
    end
end
for i=0:last
    k=k+1;
    fprintf(fout,'%8d',1003+10*i);
    if k==8
        fprintf(fout,'\n        ');
        k=0;
    end
end
for i=0:last
    k=k+1;
    fprintf(fout,'%8d',1004+10*i);
    if k==8
        fprintf(fout,'\n        ');
        k=0;
    end
end
for j=[1 4]
    for i=0:last
        k=k+1;
        fprintf(fout,'%8d',2000+j+10*i);
        if k==8
            fprintf(fout,'\n        ');
            k=0;
        end
    end    
end

fclose(fout);