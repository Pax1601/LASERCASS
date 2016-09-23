function[A_pbeam,pbeam_id]=pbeam_generator(n_rib_station,start_val)
%Funzione per la scrittura delle PBEAM

n_bay_prop=n_rib_station-1;
fout=fopen('pbeam.dat','w');
I1=1.0e-5;
I2=1.0e-5;
I12=0.0;
J=1.0e-5;
NSM=0.;
count=1;
pbeam_id=zeros(6*n_bay_prop,1);
A_pbeam=start_val(:,3);
for i=1:n_bay_prop
    for j=1:6
        fprintf(fout,'%8s%8d%8d%8.1e%8.1e%8.1e%8.1e%8.1e%8.1e\n','PBEAM   ',1000+(j-1)+(i-1)*10,1,start_val(i,3),I1,I2,I12,J,NSM); 
        fprintf(fout,'%8s%8.5f%8.5f%8.5f%8.5f%8.5f%8.5f%8.5f%8.5f\n','',0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0); 
        fprintf(fout,'%8s%8s%8.5f\n','','YESA    ',1.); 
        fprintf(fout,'%8s%8.5f%8.5f\n','',1.,1.); 
        if j==1
            fprintf(fout,'%40s%8.5f%8.5f%8.5f%8.5f\n','',0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256));
        elseif j==2
            fprintf(fout,'%40s%8.5f%8.5f%8.5f%8.5f\n','',0.,0.62*sqrt(A_pbeam(i)/0.36),0.,0.62*sqrt(A_pbeam(i)/0.36));
        elseif j==3
            fprintf(fout,'%40s%8.5f%8.5f%8.5f%8.5f\n','',0.294*sqrt(A_pbeam(i)/0.2256),-0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),-0.294*sqrt(A_pbeam(i)/0.2256));
        elseif j==4
            fprintf(fout,'%40s%8.5f%8.5f%8.5f%8.5f\n','',0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256));
        elseif j==5
            fprintf(fout,'%40s%8.5f%8.5f%8.5f%8.5f\n','',0,-0.62*sqrt(A_pbeam(i)/0.36),0,-0.62*sqrt(A_pbeam(i)/0.36));
        elseif j==6
            fprintf(fout,'%40s%8.5f%8.5f%8.5f%8.5f\n','',0.294*sqrt(A_pbeam(i)/0.2256),-0.294*sqrt(A_pbeam(i)/0.2256),0.294*sqrt(A_pbeam(i)/0.2256),-0.294*sqrt(A_pbeam(i)/0.2256));
        end
        A_pbeam(1000+(j-1)+(i-1)*10)=start_val(i,3);
        pbeam_id(count)=1000+(j-1)+(i-1)*10;
        count=count+1;
    end
end
fclose(fout);


