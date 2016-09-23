function[coord_rib_nodes,size_rib_nodes]=rib_nodes_position...
    (vertex_x,vertex_y,vertex_z,x_sup,y_sup,z_sup,x_inf,y_inf,z_inf,gap_sup,...
    gap_inf,nelem_tra,nelem_longant,nelem_longpost,stiff_type,ncorr_sup,ncorr_inf)

%Funzione per il calcolo delle coordinate dei nodi del contorno delle centine

gap_sup=gap_sup/nelem_tra;
gap_inf=gap_inf/nelem_tra;
%Pannello superiore
if strcmpi(stiff_type,'var')==1
    n_added=20;
    tang_sup=(vertex_z(3)-z_sup(end))/(vertex_x(3)-x_sup(end));
    deltax=vertex_x(3)-x_sup(end);
    added_x=deltax*(1:n_added)'+vertex_x(3)*ones(n_added,1);
    added_y=vertex_y(3)*ones(n_added,1);
    added_z=tang_sup*deltax*(1:n_added)'+vertex_z(3)*ones(n_added,1);
    pti_sup=[vertex_x(1) vertex_y(1) vertex_z(1);x_sup' y_sup' z_sup';vertex_x(3) vertex_y(3) vertex_z(3);added_x added_y added_z];
else
    pti_sup=[vertex_x(1) vertex_y(1) vertex_z(1);x_sup' y_sup' z_sup';vertex_x(3) vertex_y(3) vertex_z(3)];    
end
l_sup=(diff(pti_sup(:,1)).^2+diff(pti_sup(:,3)).^2).^0.5;
l_incr_sup=zeros(length(pti_sup),1);
for i=1:length(l_sup)
    l_incr_sup(i+1)=l_sup(i)+l_incr_sup(i);
end
ltot_sup=sum(l_sup);


if strcmpi(stiff_type,'var')==1
    ncorr_sup=floor(ltot_sup/gap_sup);
elseif strcmpi(stiff_type,'fix')==1
    gap_sup=ltot_sup/(ncorr_sup+1);
end

for i=1:ncorr_sup
    gap=gap_sup*i;                                                              %Posizione del corrente in coordinate curvilinee
    prima=find(l_incr_sup<gap,1,'last');
    dopo=find(l_incr_sup>gap,1,'first');
    if dopo>=length(pti_sup)
        dopo=length(pti_sup);
        prima=dopo-1;
    end
    res=gap-l_incr_sup(prima);
    coord_sup(i,1)=((pti_sup(dopo,1)-pti_sup(prima,1))*res/l_sup(prima))+pti_sup(prima,1);
    coord_sup(i,2)=pti_sup(prima,2);
    coord_sup(i,3)=((pti_sup(dopo,3)-pti_sup(prima,3))*res/l_sup(prima))+pti_sup(prima,3);
end

%Pannello inferiore
if strcmpi(stiff_type,'var')==1
    tang_inf=(vertex_z(4)-z_inf(1))/(vertex_x(4)-x_inf(1));
    deltax=vertex_x(4)-x_inf(1);
    added_x=deltax*(1:n_added)'+vertex_x(4)*ones(n_added,1);
    added_y=vertex_y(4)*ones(n_added,1);
    added_z=tang_inf*deltax*(1:n_added)'+vertex_z(4)*ones(n_added,1);
    pti_inf=([vertex_x(2) vertex_y(2) vertex_z(2);flipud(x_inf') flipud(y_inf') flipud(z_inf');vertex_x(4) vertex_y(4) vertex_z(4);added_x added_y added_z]);
else
    pti_inf=([vertex_x(2) vertex_y(2) vertex_z(2);flipud(x_inf') flipud(y_inf') flipud(z_inf');vertex_x(4) vertex_y(4) vertex_z(4)]);    
end
l_inf=(diff(pti_inf(:,1)).^2+diff(pti_inf(:,3)).^2).^0.5;
l_incr_inf=zeros(length(pti_inf),1);
for i=1:length(l_inf)
    l_incr_inf(i+1)=l_inf(i)+l_incr_inf(i);
end
ltot_inf=sum(l_inf);                                                            %Lunghezza pannello inferiore

 
if strcmpi(stiff_type,'var')==1
    ncorr_inf=floor(ltot_inf/gap_inf);
elseif strcmpi(stiff_type,'fix')==1
    gap_inf=ltot_inf/(ncorr_inf+1);
end

for i=1:ncorr_inf
    gap=gap_inf*i;                                                              %Posizione del corrente in coordinate curvilinee
    prima=find(l_incr_inf<gap,1,'last');
    dopo=find(l_incr_inf>gap,1,'first');
    if dopo>=length(pti_inf)
        dopo=length(pti_inf);
        prima=dopo-1;
    end
    res=gap-l_incr_inf(prima);
    coord_inf(i,1)=((pti_inf(dopo,1)-pti_inf(prima,1))*res/l_inf(prima))+pti_inf(prima,1);
    coord_inf(i,2)=pti_inf(prima,2);
    coord_inf(i,3)=((pti_inf(dopo,3)-pti_inf(prima,3))*res/l_inf(prima))+pti_inf(prima,3);
end

%Longherone anteriore
ltot_longant=sqrt((vertex_x(2)-vertex_x(1))^2+(vertex_z(2)-vertex_z(1))^2);
gap_longant=ltot_longant/nelem_longant;
coord_longant=zeros(nelem_longant-1,3);
for i=1:nelem_longant-1
    gap=gap_longant*i;                                                      %Posizione del corrente in coordinate curvilinee
    coord_longant(i,:)=[((vertex_x(1)-vertex_x(2))*gap/ltot_longant)+vertex_x(2) vertex_y(2) ((vertex_z(1)-vertex_z(2))*gap/ltot_longant)+vertex_z(2)];
end

%Longherone posteriore
ltot_longpost=sqrt((vertex_x(3)-vertex_x(4))^2+(vertex_z(3)-vertex_z(4))^2);
gap_longpost=ltot_longpost/nelem_longpost;
coord_longpost=zeros(nelem_longpost-1,3);
for i=1:nelem_longpost-1
    gap=gap_longpost*i;                                                      %Posizione del corrente in coordinate curvilinee
    coord_longpost(i,:)=[((vertex_x(4)-vertex_x(3))*gap/ltot_longpost)+vertex_x(3) vertex_y(3) ((vertex_z(4)-vertex_z(3))*gap/ltot_longpost)+vertex_z(3)];
end

coord_rib_nodes=[coord_sup
    coord_longpost
    coord_inf
    coord_longant];
size_rib_nodes=[size(coord_sup,1);size(coord_longpost,1);size(coord_inf,1);size(coord_longant,1)];




























