function[coord_nodes]=nodes_position(coord_sect_nodes,coord_rib_nodes,size_sect_nodes,size_rib_nodes,nelem_rib_tra,stretch,nrib_span)
%Funzione che calcola la posizione dei nodi in apertura


for i=1:stretch
    for j=1:nrib_span(i)
        for p=1:nelem_rib_tra
            count=1;
            if j==1
                %Vertice1
                k=1;                                                                %Indice relativo alla centina (j)
                w=1;                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(w,:);
                count=count+1;
                %Pannello superiore
                for k=1:size_rib_nodes{i}{j}(1)
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k+1,1)-coord_sect_nodes{i}(k+1,1))^2+(coord_rib_nodes{i}{j}(k+1,2)-coord_sect_nodes{i}(k+1,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k+1,3)-coord_sect_nodes{i}(k+1,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k+1,1)-coord_sect_nodes{i}(k+1,1) coord_rib_nodes{i}{j}(k+1,2)-coord_sect_nodes{i}(k+1,2) ...
                        coord_rib_nodes{i}{j}(k+1,3)-coord_sect_nodes{i}(k+1,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(k+1,:);
                    count=count+1;
                end
                %Vertice3
                k=2+size_rib_nodes{i}{j}(1);                                        %Indice relativo alla centina (j)
                w=2+size_sect_nodes{i}(1);                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(w,:);
                count=count+1;
                %Longherone posteriore
                for m=1:size_rib_nodes{i}{j}(2)
                    k=m+size_rib_nodes{i}{j}(1)+2;
                    w=m+size_sect_nodes{i}(1)+2; 
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                        coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(w,:);
                    count=count+1;
                end
                %Vertice4
                k=3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2);                                        %Indice relativo alla centina (j)
                w=3+size_sect_nodes{i}(1)+size_sect_nodes{i}(2);                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(w,:);
                count=count+1;
                %Pannello inferiore
                for m=1:size_rib_nodes{i}{j}(3)
                    k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+3;
                    w=m+size_sect_nodes{i}(1)+size_sect_nodes{i}(2)+3; 
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                        coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(w,:);
                    count=count+1;
                end
                %Vertice2
                k=4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3);                                        %Indice relativo alla centina (j)
                w=4+size_sect_nodes{i}(1)+size_sect_nodes{i}(2)+size_sect_nodes{i}(3);                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(w,:);
                count=count+1;   
                %Longherone anteriore
                for m=1:size_rib_nodes{i}{j}(4)
                    k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3)+4;
                    w=m+size_sect_nodes{i}(1)+size_sect_nodes{i}(2)+size_sect_nodes{i}(3)+4; 
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                        coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_sect_nodes{i}(w,:);
                    count=count+1;
                end
            else
                %Vertice1
                k=1;                                                                %Indice relativo alla centina (j)
                w=1;                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(w,:);
                count=count+1;
                %Pannello superiore
                for k=1:size_rib_nodes{i}{j}(1)
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k+1,1)-coord_rib_nodes{i}{j-1}(k+1,1))^2+(coord_rib_nodes{i}{j}(k+1,2)-coord_rib_nodes{i}{j-1}(k+1,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k+1,3)-coord_rib_nodes{i}{j-1}(k+1,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k+1,1)-coord_rib_nodes{i}{j-1}(k+1,1) coord_rib_nodes{i}{j}(k+1,2)-coord_rib_nodes{i}{j-1}(k+1,2) ...
                        coord_rib_nodes{i}{j}(k+1,3)-coord_rib_nodes{i}{j-1}(k+1,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(k+1,:);
                    count=count+1;
                end
                %Vertice3
                k=2+size_rib_nodes{i}{j}(1);                                        %Indice relativo alla centina (j)
                w=2+size_rib_nodes{i}{j-1}(1);                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(w,:);
                count=count+1;
                %Longherone posteriore
                for m=1:size_rib_nodes{i}{j}(2)
                    k=m+size_rib_nodes{i}{j}(1)+2;
                    w=m+size_rib_nodes{i}{j-1}(1)+2; 
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                        coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(w,:);
                    count=count+1;
                end
                %Vertice4
                k=3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2);                                        %Indice relativo alla centina (j)
                w=3+size_rib_nodes{i}{j-1}(1)+size_rib_nodes{i}{j-1}(2);                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(w,:);
                count=count+1;
                %Pannello inferiore
                for m=1:size_rib_nodes{i}{j}(3)
                    k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+3;
                    w=m+size_rib_nodes{i}{j-1}(1)+size_rib_nodes{i}{j-1}(2)+3; 
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                        coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(w,:);
                    count=count+1;
                end
                %Vertice2
                k=4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3);                                        %Indice relativo alla centina (j)
                w=4+size_rib_nodes{i}{j-1}(1)+size_rib_nodes{i}{j-1}(2)+size_rib_nodes{i}{j-1}(3);                                                                %Indice relativo alla sezionwe (i)
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+ ...
                    (coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                delta=lunghezza/nelem_rib_tra;
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(w,:);
                count=count+1;   
                %Longherone anteriore
                for m=1:size_rib_nodes{i}{j}(4)
                    k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3)+4;
                    w=m+size_rib_nodes{i}{j-1}(1)+size_rib_nodes{i}{j-1}(2)+size_rib_nodes{i}{j-1}(3)+4; 
                    lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+(coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+ ...
                        (coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                    delta=lunghezza/nelem_rib_tra;
                    v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                        coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                    coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{j-1}(w,:);
                    count=count+1;
                end
            end        
        end      
    end
    j=nrib_span(i)+1;
    for p=1:nelem_rib_tra
        count=1;
        %Vertice1
        k=1;                                                                %Indice relativo alla centina (j)
        w=1;                                                                %Indice relativo alla sezionwe (i)
        lunghezza=sqrt((coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1))^2+(coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2))^2+ ...
            (coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3))^2);
        delta=lunghezza/nelem_rib_tra;
        v=[coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1) coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2) ...
            coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3)]/lunghezza;
        coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(w,:);
        count=count+1;
        %Pannello superiore
        for k=1:size_sect_nodes{i+1}(1)
            lunghezza=sqrt((coord_sect_nodes{i+1}(k+1,1)-coord_rib_nodes{i}{end}(k+1,1))^2+(coord_sect_nodes{i+1}(k+1,2)-coord_rib_nodes{i}{end}(k+1,2))^2+ ...
                (coord_sect_nodes{i+1}(k+1,3)-coord_rib_nodes{i}{end}(k+1,3))^2);
            delta=lunghezza/nelem_rib_tra;
            v=[coord_sect_nodes{i+1}(k+1,1)-coord_rib_nodes{i}{end}(k+1,1) coord_sect_nodes{i+1}(k+1,2)-coord_rib_nodes{i}{end}(k+1,2) ...
                coord_sect_nodes{i+1}(k+1,3)-coord_rib_nodes{i}{end}(k+1,3)]/lunghezza;
            coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(k+1,:);
            count=count+1;
        end
        %Vertice3
        k=2+size_sect_nodes{i+1}(1);                                        %Indice relativo alla centina (j)
        w=2+size_rib_nodes{i}{end}(1);                                                                %Indice relativo alla sezionwe (i)
        lunghezza=sqrt((coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1))^2+(coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2))^2+ ...
            (coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3))^2);
        delta=lunghezza/nelem_rib_tra;
        v=[coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1) coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2) ...
            coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3)]/lunghezza;
        coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(w,:);
        count=count+1;
        %Longherone posteriore
        for m=1:size_sect_nodes{i+1}(2)
            k=m+size_sect_nodes{i+1}(1)+2;
            w=m+size_rib_nodes{i}{end}(1)+2; 
            lunghezza=sqrt((coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1))^2+(coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2))^2+ ...
                (coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3))^2);
            delta=lunghezza/nelem_rib_tra;
            v=[coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1) coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2) ...
                coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3)]/lunghezza;
            coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(w,:);
            count=count+1;
        end
        %Vertice4
        k=3+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2);                                        %Indice relativo alla centina (j)
        w=3+size_rib_nodes{i}{end}(1)+size_rib_nodes{i}{end}(2);                                                                %Indice relativo alla sezionwe (i)
        lunghezza=sqrt((coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1))^2+(coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2))^2+ ...
            (coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3))^2);
        delta=lunghezza/nelem_rib_tra;
        v=[coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1) coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2) ...
            coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3)]/lunghezza;
        coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(w,:);
        count=count+1;
        %Pannello inferiore
        for m=1:size_sect_nodes{i+1}(3)
            k=m+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+3;
            w=m+size_rib_nodes{i}{end}(1)+size_rib_nodes{i}{end}(2)+3; 
            lunghezza=sqrt((coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1))^2+(coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2))^2+ ...
                (coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3))^2);
            delta=lunghezza/nelem_rib_tra;
            v=[coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1) coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2) ...
                coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3)]/lunghezza;
            coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(w,:);
            count=count+1;
        end
        %Vertice2
        k=4+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3);                                        %Indice relativo alla centina (j)
        w=4+size_rib_nodes{i}{end}(1)+size_rib_nodes{i}{end}(2)+size_rib_nodes{i}{end}(3);                                                                %Indice relativo alla sezionwe (i)
        lunghezza=sqrt((coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1))^2+(coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2))^2+ ...
            (coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3))^2);
        delta=lunghezza/nelem_rib_tra;
        v=[coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1) coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2) ...
            coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3)]/lunghezza;
        coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(w,:);
        count=count+1;   
        %Longherone anteriore
        for m=1:size_sect_nodes{i+1}(4)
            k=m+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3)+4;
            w=m+size_rib_nodes{i}{end}(1)+size_rib_nodes{i}{end}(2)+size_rib_nodes{i}{end}(3)+4; 
            lunghezza=sqrt((coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1))^2+(coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2))^2+ ...
                (coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3))^2);
            delta=lunghezza/nelem_rib_tra;
            v=[coord_sect_nodes{i+1}(k,1)-coord_rib_nodes{i}{end}(w,1) coord_sect_nodes{i+1}(k,2)-coord_rib_nodes{i}{end}(w,2) ...
                coord_sect_nodes{i+1}(k,3)-coord_rib_nodes{i}{end}(w,3)]/lunghezza;
            coord_nodes{i}{j}{p}(count,:)=p*delta*v+coord_rib_nodes{i}{end}(w,:);
            count=count+1;
        end
    end
end















