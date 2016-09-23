function[]=dtable_generator(v1,v2,v3,v4)
%Funzione per la generazione della dtable contenente l'alteza dei
%longheroni lungo la span.

nbaie=length(v1)-1;
l_post=zeros(nbaie,1);l_ant=zeros(nbaie,1);
for i=1:nbaie
    l_ant(i)=norm(v1(i,:)-v2(i,:));
    l_post(i)=norm(v3(i,:)-v4(i,:));
end
fout=fopen('dtable.dat','w');
fprintf(fout,'%8s','DTABLE');
k=1;
for i=1:nbaie
    id=2+(i-1)*10;
    if id<10
        lab=['h00' num2str(id)];
    elseif id<100
        lab=['h0' num2str(id)];
    else
        lab=['h' num2str(id)];
    end
    fprintf(fout,'%8s%8.4f',lab,l_post(i));
    k=k+1;
    if k==5
        k=1;
        fprintf(fout,'\n%8s',' ');
    end
end
for i=1:nbaie
    id=4+(i-1)*10;
    if id<10
        lab=['h00' num2str(id)];
    elseif id<100
        lab=['h0' num2str(id)];
    else
        lab=['h' num2str(id)];
    end
    fprintf(fout,'%8s%8.4f',lab,l_ant(i));
    k=k+1;
    if k==5
        k=1;
        fprintf(fout,'\n%8s',' ');
    end
end
fclose(fout);
