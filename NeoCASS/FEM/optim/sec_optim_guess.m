%Funzionale da minimizzare
function[sol]=sec_optim_guess(x,pos_mod,obj,nelem_sup,nelem_inf,nelem_longant,nelem_longpost,nelem_tra)
[I1 I2 J Area]=inertia_calculator(x,pos_mod,nelem_sup,nelem_inf,nelem_longant,nelem_longpost,nelem_tra);
sc = zeros(4,1);
%sc(1)=((obj(1)-Area))/obj(1);
%sc(2)=((obj(2)-I1))/obj(2); % y
sc(3)=((obj(3)-I2))/obj(3); % 
sc(4)=((obj(4)-J))/obj(4);
sol = norm(sc);
end

%Calcolo delle caratteristiche inerziali di una sezione
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
