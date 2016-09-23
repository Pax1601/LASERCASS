function[start_val, NODEY] = starting_point_opt(stretch,nrib_span,Area,I1,I2,I12,J,nodey,nelem_rib_tra,coord_nodes,size_rib_nodes,coord_sect_nodes,size_sect_nodes,nelem_tra)
%Funzione che determina il valore iniziale dello spessore dei pannelli e l'area dei correnti

count=1;
idx_cg=1;
span_pos(idx_cg)=0;
%
X(idx_cg)=0;
Y(idx_cg)=0;
Z(idx_cg)=0;
%
idx_cg=idx_cg+1;
for i=1:stretch
    for j=1:nrib_span(i)
        span_pos(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,2)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_rib_nodes{i}{j}(1),2)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2),2)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3),2));
        X(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,1)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_rib_nodes{i}{j}(1),1)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2),1)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3),1));
        Z(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,3)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_rib_nodes{i}{j}(1),3)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(3+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2),3)+ ...
            coord_nodes{i}{j}{nelem_rib_tra}(4+size_rib_nodes{i}{j}(1)+size_rib_nodes{i}{j}(2)+size_rib_nodes{i}{j}(3),3));
        Y(idx_cg)=span_pos(idx_cg);
%
        idx_cg=idx_cg+1;
    end
    j=nrib_span(i)+1;
    span_pos(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,2)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_sect_nodes{i+1}(1),2)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(3+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2),2)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(4+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3),2));
    X(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,1)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_sect_nodes{i+1}(1),1)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(3+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2),1)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(4+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3),1));
    Z(idx_cg)=0.25*(coord_nodes{i}{j}{nelem_rib_tra}(1,3)+coord_nodes{i}{j}{nelem_rib_tra}(2+size_sect_nodes{i+1}(1),3)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(3+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2),3)+ ...
        coord_nodes{i}{j}{nelem_rib_tra}(4+size_sect_nodes{i+1}(1)+size_sect_nodes{i+1}(2)+size_sect_nodes{i+1}(3),3));
    Y(idx_cg)=span_pos(idx_cg);
    idx_cg=idx_cg+1;
end
%
X(1)=X(2);
Z(1)=Z(2);
%
nelem = length(X);
Lbeam = zeros(nelem-1,1);
for j = 1:nelem-1
  P2 = [X(j+1), Y(j+1), Z(j+1)];
  P1 = [X(j), Y(j), Z(j)];
  Lbeam(j) = norm(P2-P1);
end
NODEY  = [0; cumsum(Lbeam)]';
%

option=optimset('Algorithm','interior-point','Display','notify','MaxFunEvals',10000,'MaxIter',10000,'TolFun',1e-15,'TolX',1e-15,'DiffMaxChange',1,'DiffMinChange',1e-10);
val0=[5e-3 5e-3 5e-4];  %[Spessore_pannello_superiore Spessore_pannello-longherone Area correnti]
XL = [1e-3 1e-3 1e-5];
XU = [1 1 1];
n_baia=1;
count=1;
%span=span_pos(n_baia);                                                           %Posizione in y della centina
%idx=find(nodey>=span,1,'first');
%if idx==1
obj(1) = interp1(nodey, Area, NODEY(1),'linear', 'extrap');
obj(2) = interp1(nodey, I2, NODEY(1),'linear', 'extrap');
obj(3) = interp1(nodey, I1, NODEY(1),'linear', 'extrap');
obj(4) = interp1(nodey, J, NODEY(1),'linear', 'extrap');
%else
%    obj=[Area(idx-1) I2(idx-1) I1(idx-1) J(idx-1)];
%end
pos=[coord_sect_nodes{1}(:,1) coord_sect_nodes{1}(:,3)];
pos_mod=pos-repmat([mean(coord_sect_nodes{1}(:,1)) mean(coord_sect_nodes{1}(:,3))],length(pos),1);
val=fmincon(@sec_optim_guess,val0,[],[],[],[],XL,XU,[],option,pos_mod,obj,...
            size_sect_nodes{1}(1)+1,size_sect_nodes{1}(3)+1,size_sect_nodes{1}(4)+1,size_sect_nodes{1}(2)+1,nelem_tra);   %Calcolo del minimo vincolato dei parametri
start_val(count,:)=val;
count=count+1;
for i=1:stretch                                                   %Investigo le sole sezioni dove ci sono le centine perchè lì ho il cambio di spessore
    for j=1:nrib_span(i)    
        n_baia=n_baia+1;
%        span=span_pos(n_baia);                                                           %Posizione in y della centina
%        idx=find(nodey>=span,1,'first');
%        if idx==1
%            obj=[Area(1) I2(1) I1(1) I12(1) J(1)];
%        else
%            obj=[Area(idx-1) I2(idx-1) I1(idx-1) J(idx-1)];
%        end
%
        obj(1) = interp1(nodey, Area, NODEY(n_baia),'linear', 'extrap');
        obj(2) = interp1(nodey, I2, NODEY(n_baia),'linear', 'extrap');
        obj(3) = interp1(nodey, I1, NODEY(n_baia),'linear', 'extrap');
        obj(4) = interp1(nodey, J, NODEY(n_baia),'linear', 'extrap');
%

        pos=[coord_nodes{i}{j}{nelem_rib_tra}(:,1) coord_nodes{i}{j}{nelem_rib_tra}(:,3)];
        pos_mod=pos-repmat([mean(coord_nodes{i}{j}{nelem_rib_tra}(:,1)) mean(coord_nodes{i}{j}{nelem_rib_tra}(:,3))],length(pos),1);
        val=fmincon(@sec_optim_guess,val0,[],[],[],[],XL,XU,[],option,pos_mod,obj,size_rib_nodes{i}{j}(1)+1,size_rib_nodes{i}{j}(3)+1,size_rib_nodes{i}{j}(4)+1,size_rib_nodes{i}{j}(2)+1,nelem_tra);   %Calcolo del minimo vincolato dei parametri
        start_val(count,:)=val;
        count=count+1;
    end
    if i<stretch
        n_baia=n_baia+1;
%        span=span_pos(n_baia);                                                          %Posizione in y della centina
%        idx=find(nodey>=span,1,'first');
%        if idx==1
%            obj=[Area(1) I2(1) I1(1) J(1)];
%        else
%            obj=[Area(idx-1) I2(idx-1) I1(idx-1) J(idx-1)];
%        end
        obj(1) = interp1(nodey, Area, NODEY(n_baia),'linear', 'extrap');
        obj(2) = interp1(nodey, I2, NODEY(n_baia),'linear', 'extrap');
        obj(3) = interp1(nodey, I1, NODEY(n_baia),'linear', 'extrap');
        obj(4) = interp1(nodey, J, NODEY(n_baia),'linear', 'extrap');
%
        pos=[coord_sect_nodes{i+1}(:,1) coord_sect_nodes{i+1}(:,3)];
        pos_mod=pos-repmat([mean(coord_sect_nodes{i+1}(:,1)) mean(coord_sect_nodes{i+1}(:,3))],length(pos),1);
        val=fmincon(@sec_optim_guess,val0,[],[],[],[],XL,XU,[],option,pos_mod,obj,size_sect_nodes{i+1}(1)+1,size_sect_nodes{i+1}(3)+1,size_sect_nodes{i+1}(4)+1,size_sect_nodes{i+1}(2)+1,nelem_tra);   %Calcolo del minimo vincolato dei parametri
        start_val(count,:)=val;
        count=count+1; 
    end
end
end

function[I1 I2 J Area]=inertia_calculator(x,pos_mod,nelem_sup,nelem_inf,nelem_longant,nelem_longpost,nelem_tra)
massa=zeros(length(pos_mod),1);
%Pannello superiore
dist=sqrt((pos_mod(2,1)-pos_mod(1,1))^2+(pos_mod(2,2)-pos_mod(1,2))^2);
massa(1:nelem_sup+1)=massa(1:nelem_sup+1)+x(1)*dist;
%Longherone posteriore
dist=sqrt((pos_mod(nelem_sup+2,1)-pos_mod(nelem_sup+1,1))^2+(pos_mod(nelem_sup+2,2)-pos_mod(nelem_sup+1,2))^2);
massa(nelem_sup+1:nelem_sup+nelem_longpost+1)=massa(nelem_sup+1:nelem_sup+nelem_longpost+1)+x(2)*dist;
%Pannello inferiore
dist=sqrt((pos_mod(nelem_sup+nelem_longpost+4,1)-pos_mod(nelem_sup+nelem_longpost+3,1))^2+(pos_mod(nelem_sup+nelem_longpost+4,2)-pos_mod(nelem_sup+nelem_longpost+3,2))^2);
massa(nelem_sup+nelem_longpost+1:nelem_sup+nelem_longpost+nelem_inf+1)=massa(nelem_sup+nelem_longpost+1:nelem_sup+nelem_longpost+nelem_inf+1)+x(1)*dist;
%Longherone anteriore
dist=sqrt((pos_mod(nelem_sup+nelem_longpost+nelem_inf+2,1)-pos_mod(nelem_sup+nelem_longpost+nelem_inf+1,1))^2+(pos_mod(nelem_sup+nelem_longpost+nelem_inf+2,2)-pos_mod(nelem_sup+nelem_longpost+nelem_inf+1,2))^2);
massa(nelem_sup+nelem_longpost+nelem_inf+1:nelem_sup+nelem_longpost+nelem_inf+nelem_longant)=massa(nelem_sup+nelem_longpost+nelem_inf+1:nelem_sup+nelem_longpost+nelem_inf+nelem_longant)+x(2)*dist;
massa(1)=massa(1)+x(2)*dist;
%Ripristino massa dei vertici
massa(1)=massa(1)/2;massa(nelem_sup+1)=massa(nelem_sup+1)/2;
massa(nelem_sup+nelem_longpost+1)=massa(nelem_sup+nelem_longpost+1)/2;massa(nelem_sup+nelem_longpost+nelem_inf+1)=massa(nelem_sup+nelem_longpost+nelem_inf+1)/2;
%Correnti
for i=1:nelem_tra:nelem_sup+1
    massa(i)=massa(i)+x(3);
end
for i=nelem_sup+nelem_longpost+1:nelem_tra:nelem_sup+nelem_longpost+nelem_inf+1
    massa(i)=massa(i)+x(3);
end
Area=sum(massa);
%Calcolo delle inerzie della sezione
I1=sum(massa.*(pos_mod(:,1).^2));
I2=sum(massa.*(pos_mod(:,2).^2));
c=0.5*(abs(pos_mod(1,1)-pos_mod(nelem_sup+1,1))+abs(pos_mod(nelem_sup+nelem_longpost+1,1)-pos_mod(nelem_sup+nelem_longpost+nelem_inf+1,1)));
h=1.1*0.5*(abs(pos_mod(1,2)-pos_mod(nelem_sup+nelem_longpost+nelem_inf+1,2))+abs(pos_mod(nelem_sup+1,2)-pos_mod(nelem_sup+nelem_longpost+1,2)));
J=2*(c*h)^2*x(1)*x(2)/(c*x(2)+h*x(1));
if x(1)==0 && x(2)==0
    J=0;
end
end
