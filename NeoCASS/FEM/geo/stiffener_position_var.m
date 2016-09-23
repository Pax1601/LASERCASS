function[x_corr_sup,y_corr_sup,z_corr_sup,x_corr_inf,y_corr_inf,z_corr_inf,ncorr_sup,ncorr_inf]=stiffener_position_var(vertex_x,vertex_y,vertex_z,x_sup,y_sup,z_sup,x_inf,y_inf,z_inf,ncorr_sup,ncorr_inf,gap_sup,gap_inf,stiff_type)
%Funzione per il calcolo della posizione dei correnti sui pannelli


%Pannello superiore
pti_sup=[vertex_x(1) vertex_y(1) vertex_z(1);x_sup' y_sup' z_sup';vertex_x(3) vertex_y(3) vertex_z(3)];
l_sup=(diff(pti_sup(:,1)).^2+diff(pti_sup(:,3)).^2).^0.5;
l_incr_sup=zeros(length(l_sup),1);
l_incr_sup(1)=l_sup(1);
for i=2:length(l_sup)
    l_incr_sup(i)=l_sup(i)+l_incr_sup(i-1);
end
ltot_sup=sum(l_sup);                                                            %Lunghezza pannello superiore
if strcmpi(stiff_type,'var')==1
    ncorr_sup=floor(ltot_sup/gap_sup);
elseif strcmpi(stiff_type,'fix')==1
    gap_sup=ltot_sup/(ncorr_sup+1);
end

x_corr_sup=zeros(ncorr_sup,1);y_corr_sup=zeros(ncorr_sup,1);z_corr_sup=zeros(ncorr_sup,1);
for i=1:ncorr_sup
    gap=gap_sup*i;                                                             %Posizione del corrente in coordinate curvilinee
    prima=find(l_incr_sup<gap,1,'last');
    dopo=find(l_incr_sup>gap,1,'first');
    res=gap-l_incr_sup(prima);
    x_corr_sup(i)=((pti_sup(dopo,1)-pti_sup(prima,1))*res/l_sup(prima))+pti_sup(prima+1,1);
    y_corr_sup(i)=pti_sup(prima,2);
    z_corr_sup(i)=((pti_sup(dopo,3)-pti_sup(prima,3))*res/l_sup(prima))+pti_sup(prima+1,3);
end

%Pannello inferiore
pti_inf=([vertex_x(2) vertex_y(2) vertex_z(2);flipud(x_inf') flipud(y_inf') flipud(z_inf');vertex_x(4) vertex_y(4) vertex_z(4)]);
l_inf=(diff(pti_inf(:,1)).^2+diff(pti_inf(:,3)).^2).^0.5;
l_incr_inf=zeros(length(l_inf),1);
l_incr_inf(1)=l_inf(1);
for i=2:length(l_inf)
    l_incr_inf(i)=l_inf(i)+l_incr_inf(i-1);
end
ltot_inf=sum(l_inf);                                                            %Lunghezza pannello inferiore

if strcmpi(stiff_type,'var')==1
    ncorr_inf=floor(ltot_inf/gap_inf);
elseif strcmpi(stiff_type,'fix')==1
    gap_inf=ltot_inf/(ncorr_inf+1);
end

x_corr_inf=zeros(ncorr_inf,1);y_corr_inf=zeros(ncorr_inf,1);z_corr_inf=zeros(ncorr_inf,1);
x_inf=pti_inf(2:end-1,1);y_inf=pti_inf(2:end-1,2);z_inf=pti_inf(2:end-1,3);
for i=1:ncorr_inf
    gap=gap_inf*i;                                                              %Posizione del corrente in coordinate curvilinee
    prima=find(l_incr_inf<gap,1,'last');
    dopo=find(l_incr_inf>gap,1,'first');
    res=gap-l_incr_inf(prima);
    x_corr_inf(i)=((pti_inf(dopo,1)-pti_inf(prima,1))*res/l_inf(prima))+pti_inf(prima+1,1);
    y_corr_inf(i)=pti_inf(prima,2);
    z_corr_inf(i)=((pti_inf(dopo,3)-pti_inf(prima,3))*res/l_inf(prima))+pti_inf(prima+1,3);
end