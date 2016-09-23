function[]=dconstr_CF_generator(section)
%Funzione che genera le DCONSTR di matlab

last=section(end).prop(end);

fout=fopen('dconstr.dat','w');
for i=1:4
    fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000000+i,1000000+i,'-6.90e-3','6.90e-3');
end
for i=1:8
    fprintf(fout,'%8s%8d%8d%8s%8s\n','DCONSTR',1000004+i,1000004+i,'','1.0');
end
fprintf(fout,'%8s%8d%8d%8d%8d%8d%8d%8d%8d\n','DCONADD',1,1000001,1000002,1000003,1000004,1000005,1000006,1000007);
fprintf(fout,'%8s%8d%8d%8d%8d%8d','',1000008,1000009,1000010,1000011,1000012);
k=5;
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