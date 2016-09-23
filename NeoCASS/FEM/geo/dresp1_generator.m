function[]=dresp1_generator(pbeam_id,pshell_id,rib_id)
%Funzione per la generazione delle dresp1 di nastran

fout=fopen('dresp1.dat','w');
fprintf(fout,'%8s%8d%8s%8s\n','DRESP1',2000000,'WEIGHT','WEIGHT');                  %Response che è l'obiettivo
fprintf(fout,'  DEQATN     100W(X)=X-40000.\n');
fprintf(fout,'  DRESP2 1000000    PESO     100\n');
fprintf(fout,'          DRESP1 2000000\n');

%Generazione delle dresp1 per i correnti e le solette 
n_prop_beam=length(pbeam_id);
fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000001,'STREMAX','STRESS','PBEAM','',8,'',pbeam_id(1)); %Stress max endA
row=0;
for i=2:n_prop_beam
    if floor(i-1)/8>row
        fprintf(fout,'\n%8s','');
        row=row+1;
    end
    fprintf(fout,'%8d',pbeam_id(i));
end
fprintf(fout,'\n');
fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000002,'STREMIN','STRESS','PBEAM','',9,'',pbeam_id(1)); %Stress in endA
row=0;
for i=2:n_prop_beam
    if floor(i-1)/8>row
        fprintf(fout,'\n%8s','');
        row=row+1;
    end
    fprintf(fout,'%8d',pbeam_id(i));
end
fprintf(fout,'\n');
fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000003,'STREMAX','STRESS','PBEAM','',108,'',pbeam_id(1)); %Stress max endB
row=0;
for i=2:n_prop_beam
    if floor(i-1)/8>row
        fprintf(fout,'\n%8s','');
        row=row+1;
    end
    fprintf(fout,'%8d',pbeam_id(i));
end
fprintf(fout,'\n');
fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000004,'STREMIN','STRESS','PBEAM','',109,'',pbeam_id(1)); %Stress min endB
row=0;
for i=2:n_prop_beam
    if floor(i-1)/8>row
        fprintf(fout,'\n%8s','');
        row=row+1;
    end
    fprintf(fout,'%8d',pbeam_id(i));
end
fprintf(fout,'\n');
%Generazione delle dresp1 per il rivestimento ed i longheroni
n_prop_shell=length(pshell_id);
fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000005,'STREMAX','STRESS','PSHELL','',9,'',pshell_id(1)); %Von Mises stress at Z1
row=0;
for i=2:n_prop_shell
    if floor(i-1)/8>row
        fprintf(fout,'\n%8s','');
        row=row+1;
    end
    fprintf(fout,'%8d',pshell_id(i));
end
fprintf(fout,'\n');
fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000006,'STREMIN','STRESS','PSHELL','',17,'',pshell_id(1)); %Von Mises stress at Z2
row=0;
for i=2:n_prop_shell
    if floor(i-1)/8>row
        fprintf(fout,'\n%8s','');
        row=row+1;
    end
    fprintf(fout,'%8d',pshell_id(i));
end
fprintf(fout,'\n');
%Generazione delle dresp1 per le centine
% n_prop_rib=length(rib_id);
% fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000007,'STREMAX','STRESS','PSHELL','',9,'',rib_id(1)); %Von Mises stress at Z1
% row=0;
% for i=2:n_prop_rib
%     if floor(i-1)/8>row
%         fprintf(fout,'\n%8s','');
%         row=row+1;
%     end
%     fprintf(fout,'%8d',rib_id(i));
% end
% fprintf(fout,'\n');
% fprintf(fout,'%8s%8d%8s%8s%8s%8s%8d%8s%8d','DRESP1',1000008,'STREMIN','STRESS','PSHELL','',17,'',rib_id(1)); %Von Mises stress at Z2
% row=0;
% for i=2:n_prop_rib
%     if floor(i-1)/8>row
%         fprintf(fout,'\n%8s','');
%         row=row+1;
%     end
%     fprintf(fout,'%8d',rib_id(i));
% end
% fprintf(fout,'\n');
fclose(fout);
