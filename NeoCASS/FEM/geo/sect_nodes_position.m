function[coord_sect_nodes,nnode_sup,nnode_inf,nnode_longant,nnode_longpost]=sect_nodes_position(vertex_x,vertex_y,vertex_z,x_sup,y_sup,z_sup,x_inf,y_inf,z_inf,ncorr_sup,ncorr_inf,nelem_longant,nelem_longpost,nelem_tra,gap_sup,gap_inf,stiff_type)
%Funzione per il calcolo della posizione dei nodi in una sezione

gap_sup=gap_sup/nelem_tra;
gap_inf=gap_inf/nelem_tra;
%Pannello superiore
pti_sup=[vertex_x(1) vertex_y(1) vertex_z(1);x_sup' y_sup' z_sup';vertex_x(3) vertex_y(3) vertex_z(3)];
l_sup=(diff(pti_sup(:,1)).^2+diff(pti_sup(:,3)).^2).^0.5;
l_incr_sup=zeros(length(pti_sup),1);
for i=1:length(l_sup)
    l_incr_sup(i+1)=l_sup(i)+l_incr_sup(i);
end
ltot_sup=sum(l_sup);                                                            %Lunghezza pannello superiore

if strcmpi(stiff_type,'var')==1
    ncorr_sup=floor((ltot_sup)/gap_sup);
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
pti_inf=([vertex_x(2) vertex_y(2) vertex_z(2);flipud(x_inf') flipud(y_inf') flipud(z_inf');vertex_x(4) vertex_y(4) vertex_z(4)]);
l_inf=(diff(pti_inf(:,1)).^2+diff(pti_inf(:,3)).^2).^0.5;
l_incr_inf=zeros(length(pti_inf),1);
for i=1:length(l_inf)
    l_incr_inf(i+1)=l_inf(i)+l_incr_inf(i);
end
ltot_inf=sum(l_inf);                                                            %Lunghezza pannello inferiore

if strcmpi(stiff_type,'var')==1
    ncorr_inf=floor((ltot_inf)/gap_inf);
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
coord_sect_nodes=[vertex_x(1) vertex_y(1) vertex_z(1);
    coord_sup;
    vertex_x(3) vertex_y(3) vertex_z(3);
    coord_longpost;
    vertex_x(4) vertex_y(4) vertex_z(4);
    coord_inf;
    vertex_x(2) vertex_y(2) vertex_z(2);
    coord_longant];
nnode_sup=size(coord_sup,1);
nnode_inf=size(coord_inf,1);
nnode_longant=size(coord_longant,1);
nnode_longpost=size(coord_longpost,1);
    


