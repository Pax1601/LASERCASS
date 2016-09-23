function[x_corr_inf1,y_corr_inf1,z_corr_inf1,x_corr_sup1,y_corr_sup1,z_corr_sup1,vertex_x1,vertex_y1,vertex_z1,x_air_min,y_air_min,z_air_min,x_air_max,y_air_max,z_air_max]=...
    airfoil_box(profilo1,longant_perc,longpost_perc,section)


x_air1=profilo1(:,1);
y_air1=profilo1(:,2);
z_air1=profilo1(:,3);

num_points = length(x_air1);

x_air=x_air1;
y_air=y_air1;
z_air=z_air1;

% Allineamento della corda all'asse X
%Determinazione degli estremi della corda del profilo 1
x_air_min=x_air(1);
x_air_max=x_air(round(num_points/2));
y_air_min=y_air(1);
y_air_max=y_air(round(num_points/2));
z_air_min=z_air(1);
z_air_max=z_air(round(num_points/2));

x_air_min_start=x_air_min;
z_air_min_start=z_air_min;

% Determinazione della posizione iniziale del bordo d'uscita 
%BU1(1)=x_air_max;
%BU1(2)=y_air(1);
%BU1(3)=z_air_max;
% Calcolo della corda del profilo
corda_air=sqrt((x_air_max-x_air_min)^2+(z_air_max-z_air_min)^2);
alfa_rad=pi-atan2(z_air_min-z_air_max,x_air_min-x_air_max);
%Traslazione del profilo in modo che il bordo d'uscita coincida con
%l'origine del sistema di riferimento
for i=1:num_points
    x_air(i)=x_air(i)-x_air_min;
    z_air(i)=z_air(i)-z_air_min;
end
%Rotazione di un angolo pari ad alfa del profilo alare (Allineamento della
%corda all'asse X)
for i=1:num_points
    d=sqrt(x_air(i)^2+z_air(i)^2);
    rot=d*sin(-alfa_rad);
    beta_rad=atan2(z_air(i),x_air(i));
    mov_x=rot*sin(beta_rad);
    mov_z=rot*cos(beta_rad);
    x_air(i)=x_air(i)+mov_x;
    z_air(i)=z_air(i)+mov_z;
end

% Inserimento dei longheroni per la delimitazione del box
longant=longant_perc*corda_air;
longpost=longpost_perc*corda_air;
clear d

% Divisione del pannello superiore da quello inferiore
j=1;
k=1;
for i=1:num_points
    if i<0.5*num_points+1
        x_inf(j)=x_air(i);
        z_inf(j)=z_air(i);
        j=j+1;
    else
        x_sup(k)=x_air(i);
        z_sup(k)=z_air(i);
        k=k+1;
    end
end  


%Determinazione degli estremi del cassone per il ventre del profilo
supp_ant=corda_air;
supp_post=corda_air;
for i=1:max(size(x_inf))
    d_ant=abs(abs(x_inf(i))-(corda_air-longant));
    d_post=abs(abs(x_inf(i))-(corda_air-longpost));    
    if d_ant < supp_ant
        pos_ant_inf=i;
        supp_ant=d_ant;
    end
    if d_post < supp_post
        pos_post_inf=i;
        supp_post=d_post;
    end
end

%Determinazione degli estremi del cassone per il dorso del profilo
supp_ant=corda_air;
supp_post=corda_air;
for i=1:max(size(x_sup))
    d_ant=abs(abs(x_sup(i))-(corda_air-longant));
    d_post=abs(abs(x_sup(i))-(corda_air-longpost));    
    if d_ant < supp_ant
        pos_ant_sup=i;
        supp_ant=d_ant;
    end
    if d_post < supp_post
        pos_post_sup=i;
        supp_post=d_post;
    end
end

% Identificazione dei punti del profilo a cavallo del ounto di intersezione
% superiore longherone anteriore
if abs(x_sup(pos_ant_sup))<(corda_air-longant)
    intersect(1,1)=x_sup(pos_ant_sup);  %In intersect il primo valore è il punto il secondo indica la direzione (x o z)
    intersect(1,2)=z_sup(pos_ant_sup);
    intersect(2,1)=x_sup(pos_ant_sup+1);
    intersect(2,2)=z_sup(pos_ant_sup+1);
else
    intersect(1,1)=x_sup(pos_ant_sup-1);  
    intersect(1,2)=z_sup(pos_ant_sup-1);
    intersect(2,1)=x_sup(pos_ant_sup);
    intersect(2,2)=z_sup(pos_ant_sup);
end

% Identificazione dei punti del profilo a cavallo del ounto di intersezione
% superiore longherone posteriore
if abs(x_sup(pos_post_sup))<(corda_air-longpost)    
    intersect(5,1)=x_sup(pos_post_sup);  %In intersect il primo valore è il punto il secondo indica la direzione (x o z)
    intersect(5,2)=z_sup(pos_post_sup);
    intersect(6,1)=x_sup(pos_post_sup+1);
    intersect(6,2)=z_sup(pos_post_sup+1);
else
    intersect(5,1)=x_sup(pos_post_sup-1);  
    intersect(5,2)=z_sup(pos_post_sup-1);
    intersect(6,1)=x_sup(pos_post_sup);
    intersect(6,2)=z_sup(pos_post_sup);
end

% Identificazione dei punti del profilo a cavallo del ounto di intersezione
% inferiore longherone anteriore
if abs(x_sup(pos_ant_inf))<(corda_air-longant)
    intersect(3,1)=x_inf(pos_ant_inf);  %In intersect il primo valore è il punto il secondo indica la direzione (x o z)
    intersect(3,2)=z_inf(pos_ant_inf);
    intersect(4,1)=x_inf(pos_ant_inf-1);
    intersect(4,2)=z_inf(pos_ant_inf-1);
else
    intersect(3,1)=x_inf(pos_ant_inf+1);  
    intersect(3,2)=z_inf(pos_ant_inf+1);
    intersect(4,1)=x_inf(pos_ant_inf);
    intersect(4,2)=z_inf(pos_ant_inf);
end

% Identificazione dei punti del profilo a cavallo del ounto di intersezione
% inferiore longherone posteriore
if abs(x_sup(pos_post_inf))<(corda_air-longpost)
    intersect(7,1)=x_inf(pos_post_inf);  %In intersect il primo valore è il punto il secondo indica la direzione (x o z)
    intersect(7,2)=z_inf(pos_post_inf);
    intersect(8,1)=x_inf(pos_post_inf-1);
    intersect(8,2)=z_inf(pos_post_inf-1);
else
    intersect(7,1)=x_inf(pos_post_inf+1);  
    intersect(7,2)=z_inf(pos_post_inf+1);
    intersect(8,1)=x_inf(pos_post_inf);
    intersect(8,2)=z_inf(pos_post_inf);
end

% Identificazione delle rette che collegano i punti sul profilo a cavallo
% delle intersezioni dei longheroni
for k=1:4
    m(k)=(intersect(1+2*(k-1),2)-intersect(2+2*(k-1),2))/(intersect(1+2*(k-1),1)-intersect(2+2*(k-1),1));
    c(k)=intersect(2+2*(k-1),2)-m(k)*intersect(2+2*(k-1),1);
end

%Identificazione vertici del cassone
vertex_x(1)=-(corda_air-longant);
vertex_x(2)=-(corda_air-longant);
vertex_x(3)=-(corda_air-longpost);
vertex_x(4)=-(corda_air-longpost);
vertex_z=zeros(1,4);
for i=1:4
    vertex_z(i)=m(i)*vertex_x(i)+c(i);
end

%Identificazione curve cassone
%Pannello superiore (Descritto dall'anteriore al posteriore)
j=2;
for i=1:max(size(x_sup))
    if abs(x_sup(i))<(corda_air-longant) && abs(x_sup(i))>(corda_air-longpost)
        x_box_sup(j)=x_sup(i);
        z_box_sup(j)=z_sup(i);
        j=j+1;
    end
end
%Aggiunta dei vertici superiori
x_box_sup(1)=vertex_x(1);
z_box_sup(1)=vertex_z(1);
n=max(size(x_box_sup));
x_box_sup(n+1)=vertex_x(3);
z_box_sup(n+1)=vertex_z(3);
%Pannello inferiore (Descritto dal posteriore all'anteriore)
j=2;
for i=1:max(size(x_inf))
    if abs(x_inf(i))<(corda_air-longant) && abs(x_inf(i))>(corda_air-longpost)
        x_box_inf(j)=x_inf(i);
        z_box_inf(j)=z_inf(i);
        j=j+1;
    end
end
%Aggiunta dei vertici inferiori
x_box_inf(1)=vertex_x(4);
z_box_inf(1)=vertex_z(4);
m=max(size(x_box_inf));
x_box_inf(m+1)=vertex_x(2);
z_box_inf(m+1)=vertex_z(2);

%Posizionamento punti sul pannello superiore superiori
supp=0;
l_pann_sup(1,1)=0;
l_pann_sup(1,2)=0;
for i=1:max(size(x_box_sup))-1
    l_pann_sup(i+1,1)=sqrt((x_box_sup(i+1)-x_box_sup(i))^2+(z_box_sup(i+1)-z_box_sup(i))^2);
    l_pann_sup(i+1,2)=l_pann_sup(i+1,1)+supp;
    supp=l_pann_sup(i+1,2);
end
step_sup=l_pann_sup(max(size(x_box_sup)),2)/(50+1);
for i=1:50
    dist=i*step_sup;
        for j=2:max(size(x_box_sup))
            if dist>l_pann_sup(j-1,2) && dist<l_pann_sup(j,2)
                tratto=dist-l_pann_sup(j-1,2);
                xB=x_box_sup(j)-x_box_sup(j-1);
                zB=z_box_sup(j)-z_box_sup(j-1);
                angle_rad_sup(i)=atan2(zB,xB);
                x_corr_sup(i)=tratto*cos(angle_rad_sup(i))+x_box_sup(j-1);
                z_corr_sup(i)=tratto*sin(angle_rad_sup(i))+z_box_sup(j-1);
            end
        end
end
% Posizionamento punti sul pannello inferiore
supp=0;
l_pann_inf(1,1)=0;
l_pann_inf(1,2)=0;
for i=1:max(size(x_box_inf))-1
    l_pann_inf(i+1,1)=sqrt((x_box_inf(i+1)-x_box_inf(i))^2+(z_box_inf(i+1)-z_box_inf(i))^2);
    l_pann_inf(i+1,2)=l_pann_inf(i+1,1)+supp;
    supp=l_pann_inf(i+1,2);
end
step_inf=l_pann_inf(max(size(x_box_inf)),2)/(50+1);
for i=1:50
    dist=i*step_inf;
        for j=2:max(size(x_box_inf))
            if dist>l_pann_inf(j-1,2) && dist<l_pann_inf(j,2)
                tratto=dist-l_pann_inf(j-1,2);
                xB=x_box_inf(j)-x_box_inf(j-1);
                zB=z_box_inf(j)-z_box_inf(j-1);
                angle_rad_inf(i)=atan2(zB,xB);
                x_corr_inf(i)=tratto*cos(angle_rad_inf(i))+x_box_inf(j-1);
                z_corr_inf(i)=tratto*sin(angle_rad_inf(i))+z_box_inf(j-1);
            end
        end
end

%Rototraslazione all'indietro del pannello inferiore
for i=1:max(size(x_corr_inf))
    d=sqrt(x_corr_inf(i)^2+z_corr_inf(i)^2);
    rot=-d*sin(-alfa_rad);
    beta_rad=atan2(z_corr_inf(i),x_corr_inf(i));
    mov_x=rot*sin(beta_rad);
    mov_z=rot*cos(beta_rad);
    x_corr_inf(i)=x_corr_inf(i)+mov_x+x_air_min_start;
    z_corr_inf(i)=z_corr_inf(i)+mov_z+z_air_min_start;
end

%Rototraslazione all'indietro del pannello superiore
for i=1:max(size(x_corr_sup))
    d=sqrt(x_corr_sup(i)^2+z_corr_sup(i)^2);
    rot=-d*sin(-alfa_rad);
    beta_rad=atan2(z_corr_sup(i),x_corr_sup(i));
    mov_x=rot*sin(beta_rad);
    mov_z=rot*cos(beta_rad);
    x_corr_sup(i)=x_corr_sup(i)+mov_x+x_air_min_start;
    z_corr_sup(i)=z_corr_sup(i)+mov_z+z_air_min_start;
end

%Rototraslazione all'indietro dei vertici dei longheroni
for i=1:4
    d=sqrt(vertex_x(i)^2+vertex_z(i)^2);
    rot=-d*sin(-alfa_rad);
    beta_rad=atan2(vertex_z(i),vertex_x(i));
    mov_x=rot*sin(beta_rad);
    mov_z=rot*cos(beta_rad);
    vertex_x(i)=vertex_x(i)+mov_x+x_air_min_start;
    vertex_z(i)=vertex_z(i)+mov_z+z_air_min_start;
end

x_corr_inf1=x_corr_inf; 
%In realtà queste non sono più le coordinate dei correnti ma quelle dei punti che descrivono la geometria
z_corr_inf1=z_corr_inf;
x_corr_sup1=x_corr_sup;
z_corr_sup1=z_corr_sup;
vertex_x1=vertex_x;
vertex_z1=vertex_z;
for i=1:max(size(x_corr_inf1))
    y_corr_inf1(i)=y_air1(1);
end
for i=1:max(size(x_corr_sup1))
    y_corr_sup1(i)=y_air1(1);
end
for i=1:4
    vertex_y1(i)=y_air1(1);
end