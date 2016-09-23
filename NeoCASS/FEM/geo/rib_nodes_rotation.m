function[rib_nodes,size_rib_nodes2,ver1_rot,ver2_rot,ver3_rot,ver4_rot]=rib_nodes_rotation(coord_rib_nodes,size_rib_nodes,coord_sect_nodes,size_sect_nodes,rib_vertex,sect_vertex,vers_ae,c_rib,stretch,nrib_span)

%Funzione che ruota la centina in posizione ortogonale all'asse elastico

for i=1:stretch
    a=vers_ae(i,1);b=vers_ae(i,2);c=vers_ae(i,3);                             %Piano normale al versore asse elastico
    for j=1:nrib_span(i)
        d=-a*c_rib{i}(j,1)-b*c_rib{i}(j,2)-c*c_rib{i}(j,3);                   %Intercetta del piano
        if j==1
            %Limite_superiore
            nn=3;
            x0=[sect_vertex{i}(nn,1) sect_vertex{i}(nn,2) sect_vertex{i}(nn,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1))^2+(rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2))^2 ...
            +(rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3))^2);
            v=[rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1) rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2) ...
                rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            vertex3=x0+t*v;
            ver3_rot{i}{j}=vertex3;
            %Limite_inferiore
            nn=4;
            x0=[sect_vertex{i}(nn,1) sect_vertex{i}(nn,2) sect_vertex{i}(nn,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1))^2+(rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2))^2 ...
            +(rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3))^2);
            v=[rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1) rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2) ...
                rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            vertex4=x0+t*v;
            ver4_rot{i}{j}=vertex4;
            count=1;        
            %Vertice 1
            nn=1;
            x0=[sect_vertex{i}(nn,1) sect_vertex{i}(nn,2) sect_vertex{i}(nn,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1))^2+(rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2))^2 ...
            +(rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3))^2);
            v=[rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1) rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2) ...
                rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            ver1_rot{i}{j}=x0+t*v;
            rib_nodes{i}{j}(count,:)=x0+t*v;
            count=count+1;
            %Nodi superiori
            s1=0;
            for k=1:min(size_sect_nodes{i}(1),size_rib_nodes{i}{j}(1))
                s1=s1+1;
                x0=coord_sect_nodes{i}(k+1,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(k+1,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(k+1,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(k+1,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(k+1,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(k+1,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(k+1,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
                if rib_nodes{i}{j}(count-1,1)>=vertex3(1)
                    rib_nodes{i}{j}(count-1,:)=[];
                    count=count-1;
                    s1=s1-1;
                end
            end
            %Vertice 3
            rib_nodes{i}{j}(count,:)=vertex3;
            count=count+1;
            s2=0;
            %Nodi longherone posteriore
            for m=1:size_rib_nodes{i}{j}(2)
                s2=s2+1;
                k=m+size_rib_nodes{i}{j}(1);                                     %k si riferisce alla stazione j
                w=m+size_sect_nodes{i}(1)+2;                                     %w si riferisce alla stazione j-1
                x0=coord_sect_nodes{i}(w,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
            end        
            %Vertice 4
            rib_nodes{i}{j}(count,:)=vertex4;
            count=count+1;
            s3=0;
            %Nodi inferiori
            for m=1:min(size_sect_nodes{i}(3),size_rib_nodes{i}{j}(3))
                s3=s3+1;
                k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2);            %k si riferisce alla stazione j
                w=m+size_sect_nodes{i}(1)+size_sect_nodes{i}(2)+3;              %w si riferisce alla stazione j-1
                x0=coord_sect_nodes{i}(w,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
                if rib_nodes{i}{j}(count-1,1)>=vertex4(1)
                    rib_nodes{i}{j}(count-1,:)=[];
                    count=count-1;
                    s3=s3-1;
                end
            end        
            %Vertice 2
            nn=2;
            x0=[sect_vertex{i}(nn,1) sect_vertex{i}(nn,2) sect_vertex{i}(nn,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1))^2+(rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2))^2 ...
            +(rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3))^2);
            v=[rib_vertex{i}{j}(nn,1)-sect_vertex{i}(nn,1) rib_vertex{i}{j}(nn,2)-sect_vertex{i}(nn,2) ...
                rib_vertex{i}{j}(nn,3)-sect_vertex{i}(nn,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            ver2_rot{i}{j}=x0+t*v;
            rib_nodes{i}{j}(count,:)=x0+t*v;
            count=count+1;
            s4=0;
            %Nodi longherone anteriore
            for m=1:size_rib_nodes{i}{j}(4)
                s4=s4+1;
                k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3);        %k si riferisce alla stazione j
                w=m+size_sect_nodes{i}(1)+size_sect_nodes{i}(2)+size_sect_nodes{i}(3)+4;  %w si riferisce alla stazione j-1
                x0=coord_sect_nodes{i}(w,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_sect_nodes{i}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_sect_nodes{i}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_sect_nodes{i}(w,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
            end 
%             count=count-1
%             i=i
%             j=j
%             [s1+2;s2;s3+2;s4]
            size_rib_nodes2{i}{j}=[s1;s2;s3;s4];
%              pause
        else
            %Limite_superiore
            x0=[rib_vertex{i}{j-1}(3,1) rib_vertex{i}{j-1}(3,2) rib_vertex{i}{j-1}(3,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(3,1)-rib_vertex{i}{j-1}(3,1))^2+(rib_vertex{i}{j}(3,2)-rib_vertex{i}{j-1}(3,2))^2 ...
            +(rib_vertex{i}{j}(3,3)-rib_vertex{i}{j-1}(3,3))^2);
            v=[rib_vertex{i}{j}(3,1)-rib_vertex{i}{j-1}(3,1) rib_vertex{i}{j}(3,2)-rib_vertex{i}{j-1}(3,2) ...
                rib_vertex{i}{j}(3,3)-rib_vertex{i}{j-1}(3,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            vertex3=x0+t*v;
            ver3_rot{i}{j}=x0+t*v;
            %Limite_inferiore
            x0=[rib_vertex{i}{j-1}(4,1) rib_vertex{i}{j-1}(4,2) rib_vertex{i}{j-1}(4,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(4,1)-rib_vertex{i}{j-1}(4,1))^2+(rib_vertex{i}{j}(4,2)-rib_vertex{i}{j-1}(4,2))^2 ...
            +(rib_vertex{i}{j}(4,3)-rib_vertex{i}{j-1}(4,3))^2);
            v=[rib_vertex{i}{j}(4,1)-rib_vertex{i}{j-1}(4,1) rib_vertex{i}{j}(4,2)-rib_vertex{i}{j-1}(4,2) ...
                rib_vertex{i}{j}(4,3)-rib_vertex{i}{j-1}(4,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            vertex4=x0+t*v;
            count=1;
            %Vertice 1
            x0=[rib_vertex{i}{j-1}(1,1) rib_vertex{i}{j-1}(1,2) rib_vertex{i}{j-1}(1,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(1,1)-rib_vertex{i}{j-1}(1,1))^2+(rib_vertex{i}{j}(1,2)-rib_vertex{i}{j-1}(1,2))^2 ...
            +(rib_vertex{i}{j}(1,3)-rib_vertex{i}{j-1}(1,3))^2);
            v=[rib_vertex{i}{j}(1,1)-rib_vertex{i}{j-1}(1,1) rib_vertex{i}{j}(1,2)-rib_vertex{i}{j-1}(1,2) ...
                rib_vertex{i}{j}(1,3)-rib_vertex{i}{j-1}(1,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            ver1_rot{i}{j}=x0+t*v;
            rib_nodes{i}{j}(count,:)=x0+t*v;
            count=count+1;
            s1=0;
            %Nodi superiori
            for k=1:min(size_rib_nodes{i}{j-1}(1),size_rib_nodes{i}{j}(1))
                s1=s1+1;
                x0=coord_rib_nodes{i}{j-1}(k,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(k,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(k,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(k,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(k,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(k,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(k,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
                if rib_nodes{i}{j}(count-1,1)>vertex3(1)
                    rib_nodes{i}{j}(count-1,:)=[];
                    count=count-1;
                    s1=s1-1;
                end
            end
            %Vertice 3
            rib_nodes{i}{j}(count,:)=vertex3;
            count=count+1;
            %Nodi longherone posteriore
            s2=0;
            for m=1:size_rib_nodes{i}{j}(2)
                s2=s2+1;
                k=m+size_rib_nodes{i}{j}(1);                                     %k si riferisce alla stazione j
                w=m+size_rib_nodes{i}{j-1}(1);                                   %w si riferisce alla stazione j-1
                x0=coord_rib_nodes{i}{j-1}(w,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
            end
            %Vertice 4
            rib_nodes{i}{j}(count,:)=vertex4;
            ver4_rot{i}{j}=vertex4;
            count=count+1;
            s3=0;
            %Nodi inferiori
            for m=1:min(size_rib_nodes{i}{j-1}(3),size_rib_nodes{i}{j}(3))
                s3=s3+1;
                k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2);             %k si riferisce alla stazione j
                w=m+size_rib_nodes{i}{j-1}(1)+size_rib_nodes{i}{j-1}(2);         %w si riferisce alla stazione j-1
                x0=coord_rib_nodes{i}{j-1}(w,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
                if rib_nodes{i}{j}(count-1,1)>vertex4(1)
                    rib_nodes{i}{j}(count-1,:)=[];
                    count=count-1;
                    s3=s3-1;
                end
            end
            %Vertice 2
            x0=[rib_vertex{i}{j-1}(2,1) rib_vertex{i}{j-1}(2,2) rib_vertex{i}{j-1}(2,3)];
            lunghezza=sqrt((rib_vertex{i}{j}(2,1)-rib_vertex{i}{j-1}(2,1))^2+(rib_vertex{i}{j}(2,2)-rib_vertex{i}{j-1}(2,2))^2 ...
            +(rib_vertex{i}{j}(2,3)-rib_vertex{i}{j-1}(2,3))^2);
            v=[rib_vertex{i}{j}(2,1)-rib_vertex{i}{j-1}(2,1) rib_vertex{i}{j}(2,2)-rib_vertex{i}{j-1}(2,2) ...
                rib_vertex{i}{j}(2,3)-rib_vertex{i}{j-1}(2,3)]/lunghezza;
            t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
            rib_nodes{i}{j}(count,:)=x0+t*v;
            ver2_rot{i}{j}=x0+t*v;
            count=count+1;
            %Nodi longherone anteriore
            s4=0;
            for m=1:size_rib_nodes{i}{j}(4)
                s4=s4+1;
                k=m+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3);        %k si riferisce alla stazione j
                w=m+size_rib_nodes{i}{j-1}(1)+size_rib_nodes{i}{j-1}(2)+size_rib_nodes{i}{j-1}(3);  %w si riferisce alla stazione j-1
                x0=coord_rib_nodes{i}{j-1}(w,:);
                lunghezza=sqrt((coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1))^2+ ...
                    (coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2))^2+(coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3))^2);
                v=[coord_rib_nodes{i}{j}(k,1)-coord_rib_nodes{i}{j-1}(w,1) coord_rib_nodes{i}{j}(k,2)-coord_rib_nodes{i}{j-1}(w,2) ...
                    coord_rib_nodes{i}{j}(k,3)-coord_rib_nodes{i}{j-1}(w,3)]/lunghezza;
                t=-(a*x0(1)+b*x0(2)+c*x0(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                rib_nodes{i}{j}(count,:)=x0+t*v;
                count=count+1;
            end
%             count=count-1
%             i=i
%             j=j
%             [s1+2;s2;s3+2;s4]
            size_rib_nodes2{i}{j}=[s1;s2;s3;s4];
%              pause
        end
    end
end





