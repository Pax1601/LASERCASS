function[rib_vertex,rib_sup,rib_inf]=rib_point_position(stretch,nrib_span,x_inf,y_inf,z_inf,x_sup,y_sup,z_sup,vertex_x,vertex_y,vertex_z,c_rib)

%Funzione che calcola la posizione dei punti del bordo delle centine nel
%sistema di coordinate parallelo al vento


for i=1:stretch    
    for k=1:4
       lunghezza_v{i}(k)=sqrt((vertex_x{i+1}(k)-vertex_x{i}(k))^2+(vertex_y{i+1}(k)-vertex_y{i}(k))^2+(vertex_z{i+1}(k)-vertex_z{i}(k))^2); 
       vers_vertex{i}(k,:)=[vertex_x{i+1}(k)-vertex_x{i}(k) vertex_y{i+1}(k)-vertex_y{i}(k) vertex_z{i+1}(k)-vertex_z{i}(k)]/lunghezza_v{i}(k); 
    end
    for k=1:length(x_sup{i})
       lunghezza_sup{i}(k)=sqrt((x_sup{i+1}(k)-x_sup{i}(k))^2+(y_sup{i+1}(k)-y_sup{i}(k))^2+(z_sup{i+1}(k)-z_sup{i}(k))^2); 
       vers_sup{i}(k,:)=[x_sup{i+1}(k)-x_sup{i}(k) y_sup{i+1}(k)-y_sup{i}(k) z_sup{i+1}(k)-z_sup{i}(k)]/lunghezza_sup{i}(k); 
    end    
    for k=1:length(x_inf{i})
       lunghezza_inf{i}(k)=sqrt((x_inf{i+1}(k)-x_inf{i}(k))^2+(y_inf{i+1}(k)-y_inf{i}(k))^2+(z_inf{i+1}(k)-z_inf{i}(k))^2); 
       vers_inf{i}(k,:)=[x_inf{i+1}(k)-x_inf{i}(k) y_inf{i+1}(k)-y_inf{i}(k) z_inf{i+1}(k)-z_inf{i}(k)]/lunghezza_inf{i}(k); 
    end
    for j=1:nrib_span(i)
        rib_vertex{i}{j}(1,:)=lunghezza_v{i}(1)*j/(nrib_span(i)+1)*vers_vertex{i}(1,:)+[vertex_x{i}(1) vertex_y{i}(1) vertex_z{i}(1)];
        rib_vertex{i}{j}(2,:)=lunghezza_v{i}(2)*j/(nrib_span(i)+1)*vers_vertex{i}(2,:)+[vertex_x{i}(2) vertex_y{i}(2) vertex_z{i}(2)];
        rib_vertex{i}{j}(3,:)=lunghezza_v{i}(3)*j/(nrib_span(i)+1)*vers_vertex{i}(3,:)+[vertex_x{i}(3) vertex_y{i}(3) vertex_z{i}(3)];
        rib_vertex{i}{j}(4,:)=lunghezza_v{i}(4)*j/(nrib_span(i)+1)*vers_vertex{i}(4,:)+[vertex_x{i}(4) vertex_y{i}(4) vertex_z{i}(4)];
        for k=1:length(x_sup{i})
            rib_sup{i}{j}(k,:)=lunghezza_sup{i}(k)*j/(nrib_span(i)+1)*vers_sup{i}(k,:)+[x_sup{i}(k) y_sup{i}(k) z_sup{i}(k)];
        end
        for k=1:length(x_inf{i})
            rib_inf{i}{j}(k,:)=lunghezza_inf{i}(k)*j/(nrib_span(i)+1)*vers_inf{i}(k,:)+[x_inf{i}(k) y_inf{i}(k) z_inf{i}(k)];
        end
    end
end