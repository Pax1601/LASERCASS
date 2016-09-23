function[]=analytical_buckling(section)
last=section(end).prop(end);
fout=fopen('analytical_buckling.dat','w');
%Instabilità globale
fprintf(fout,'DEQATN      1000 h(Ac,TS,SA)=SQRT(Ac/0.36);\n');
fprintf(fout,'        I0=0.0855*(h**4.);Z=0.5*TS+0.62*h;\n');
fprintf(fout,'        bs=0.16;x=0.55/bs;\n');
fprintf(fout,'        k1=0.0041*(x**3.)-0.0774*(x**2.)+0.494*x-0.0153;\n');
fprintf(fout,'        I=I0+((Ac*(Z**2.))/(1.+(Ac/(k1*bs*TS))));\n');
fprintf(fout,'        D=70.0e+9*(TS**3.)/10.6932;\n');
fprintf(fout,'        y=70.0e+9*I/(bs*D);\n');
fprintf(fout,'        c1=4.4396-1.8146*x+0.0895*y\n');
fprintf(fout,'        +0.1708*x**2-0.0104*x*y+3.2538e-6*y**2;\n');
fprintf(fout,'        c2=4.;k2=MIN(c1,c2);\n');
fprintf(fout,'        Scr=k2*(3.1415**2.)*D/((bs**2.)*TS);\n');
fprintf(fout,'        Scomp=SA*(-TANH(SA)+1.)/2.;\n');
fprintf(fout,'        instglob=ABS(Scomp)/Scr\n');
%DRESP1 Instabilità globale pannello superiore
for i=0:last
    id=1+10*i;
    if id<10
        lab=['S00' num2str(id)];
    elseif id<100
        lab=['S0' num2str(id)];
    else
        lab=['S' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               3        %8d\n','DRESP1',id,lab,id);
end
%DRESP1 Instabilità globale pannello inferiore
for i=0:last
    id=3+10*i;
    if id<10
        lab=['S00' num2str(id)];
    elseif id<100
        lab=['S0' num2str(id)];
    else
        lab=['S' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               3        %8d\n','DRESP1',id,lab,id);
end
%DRESP2 Instabilità globale pannello superiore
sect=1;
for i=0:last
    id=1+10*i;
    if id<10
        lab=['INSTg00' num2str(id)];
    elseif id<100
        lab=['INSTg0' num2str(id)];
    else
        lab=['INSTg' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',5000+id,lab,1000);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DESVAR',section(sect).prop(1)*10+1001,section(sect).prop(1)*10+1);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 Instabilità globale pannello inferiore
sect=1;
for i=0:last
    id=3+10*i;
    if id<10
        lab=['INSTg00' num2str(id)];
    elseif id<100
        lab=['INSTg0' num2str(id)];
    else
        lab=['INSTg' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',5000+id,lab,1000);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DESVAR',section(sect).prop(1)*10+1004,section(sect).prop(1)*10+3);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
for i=0:last
    id=1+10*i;
    fprintf(fout,'%8s%8d%8d%8s%8.3f\n','DCONSTR',id,5000+id,' ',0.999);
end
for i=0:last
    id=3+10*i;
    fprintf(fout,'%8s%8d%8d%8s%8.3f\n','DCONSTR',id,5000+id,' ',0.999);
end
%Instabilita del pannello
%Compressione pannello
fprintf(fout,'DEQATN     15000 tb(Ac,TS,SA)=TS/0.16;\n');
fprintf(fout,'        Scomp=SA*(-TANH(SA)+1.)/2.;\n');
fprintf(fout,'        bt=0.16/TS;d=SQRT(Ac/0.36);t=0.12*d;rapp1=t/TS;\n');
fprintf(fout,'        rapp2=(d/0.16);\n');
fprintf(fout,'        kc=(23.43*rapp1**2-24.55*rapp1+2.22)*(rapp2**3)\n');
fprintf(fout,'        +(-17.92*rapp1**2+16.54*rapp1-1.48)*(rapp2**2)\n');
fprintf(fout,'        +(3.11*rapp1**2-2.74*rapp1+0.24)*rapp2\n');
fprintf(fout,'        +(-6.69*rapp1**2+13.17*rapp1-1.24);\n');
fprintf(fout,'        RLp=abs(Scomp)/((3.1415**2)*kc*70e+9*\n');
fprintf(fout,'        (tb**2)/(12.*(1.-0.1089)))\n');
%Taglio pannello
fprintf(fout,'DEQATN     15003 tb(TS,tau)=TS/0.16;ab=0.55/0.16;\n');        
fprintf(fout,'        ks=-0.138*(ab**3)+1.4076*(ab**2)-4.6814*ab+10.467;\n');
fprintf(fout,'        RSp=abs(tau)/((3.1415**2)*ks*70e+9*(tb**2)/(12.*(1.-0.1089)))\n');
fprintf(fout,'DEQATN     15004 MSp(RL,RS)=(2./(RL+(RL**2+4.*(RS**2))**0.5\n');
fprintf(fout,'        +0.001))-1.\n');
%DRESP2 instabilità pannello superiore a compressione
sect=1;
for i=0:last
    id=1+10*i;
    if id<10
        lab=['RLp00' num2str(id)];
    elseif id<100
        lab=['RLp0' num2str(id)];
    else
        lab=['RLp' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',6000+id,lab,15000);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DESVAR',section(sect).prop(1)*10+1001,section(sect).prop(1)*10+1);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 instabilità pannello inferiore a compressione
sect=1;
for i=0:last
    id=3+10*i;
    if id<10
        lab=['RLp00' num2str(id)];
    elseif id<100
        lab=['RLp0' num2str(id)];
    else
        lab=['RLp' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',6000+id,lab,15000);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DESVAR',section(sect).prop(1)*10+1004,section(sect).prop(1)*10+3);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP1 taglio pannello superiore
for i=0:last
    id=1+10*i;
    if id<10
        lab=['T00' num2str(id)];
    elseif id<100
        lab=['T0' num2str(id)];
    else
        lab=['T' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               5        %8d\n','DRESP1',1000+id,lab,id);
end
%DRESP1 taglio pannello inferiore
for i=0:last
    id=3+10*i;
    if id<10
        lab=['T00' num2str(id)];
    elseif id<100
        lab=['T0' num2str(id)];
    else
        lab=['T' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               5        %8d\n','DRESP1',1000+id,lab,id);
end
%DRESP2 instabilità pannello superiore a taglio
sect=1;
for i=0:last
    id=1+10*i;
    if id<10
        lab=['RSp00' num2str(id)];
    elseif id<100
        lab=['RSp0' num2str(id)];
    else
        lab=['RSp' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',7000+id,lab,15003);
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+1);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id+1000);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 instabilità pannello inferiore a taglio
sect=1;
for i=0:last
    id=3+10*i;
    if id<10
        lab=['RSp00' num2str(id)];
    elseif id<100
        lab=['RSp0' num2str(id)];
    else
        lab=['RSp' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',7000+id,lab,15003);
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+3);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id+1000);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 per il calcolo del margine di sicurezza
for i=0:last
    id=1+10*i;
    if id<10
        lab=['MSp00' num2str(id)];
    elseif id<100
        lab=['MSp0' num2str(id)];
    else
        lab=['MSp' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',8000+id,lab,15004);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DRESP2',6000+id,7000+id);
end
for i=0:last
    id=3+10*i;
    if id<10
        lab=['MSp00' num2str(id)];
    elseif id<100
        lab=['MSp0' num2str(id)];
    else
        lab=['MSp' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',8000+id,lab,15004);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DRESP2',6000+id,7000+id);
end
%DCONSTR del margine di sicurezza dei pannelli
for i=0:last
    id=1+10*i;
    fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',1000+id,8000+id,0.001);
end
for i=0:last
    id=3+10*i;
    fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',1000+id,8000+id,0.001);
end
%Instabilita dell'anima del longherone
%Taglio web
fprintf(fout,'DEQATN     15005 tb(TW,h,tau)=TW/MIN(0.55,h);\n');
fprintf(fout,'        ab=MAX(0.55,h)/MIN(0.55,h);\n');
fprintf(fout,'        ks=-0.138*(ab**3)+1.4076*(ab**2)-4.6814*ab+10.467;\n');
fprintf(fout,'        RSw=abs(tau)/((3.1415**2)*ks*70e+9*(tb**2)/(12.*(1.-0.1089)))\n');
%Instabilità a flessione del longherone
fprintf(fout,'DEQATN     15006 tb(TW,h,SA)=TW/h;ab=0.55/h;\n');
fprintf(fout,'        kb=0.0012*(ab**4)-0.0646*(ab**3)+1.2644*(ab**2)-10.08*ab+61.983;\n');
fprintf(fout,'        sol=abs(SA)/((3.1415**2)*kb*70e+9*(tb**2)/(12.*(1.-0.1089)));\n');
fprintf(fout,'        check=(TANH(ab-3.0)+1.)/2.;RBw=check*sol\n');
%Margine di sicurezza per l'instabilità del longherone
fprintf(fout,'DEQATN     15007 MSw(RS,RB)=(1./(((RS**2)+(RB**2))**0.5+0.001))-1.\n');
%DRESP1 Sforzo di compressione del longherone posteriore
for i=0:last
    id=2+10*i;
    if id<10
        lab=['S00' num2str(id)];
    elseif id<100
        lab=['S0' num2str(id)];
    else
        lab=['S' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               3        %8d\n','DRESP1',id,lab,id);
end
%DRESP1 Sforzo di compressione del longherone anteriore
for i=0:last
    id=4+10*i;
    if id<10
        lab=['S00' num2str(id)];
    elseif id<100
        lab=['S0' num2str(id)];
    else
        lab=['S' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               3        %8d\n','DRESP1',id,lab,id);
end
%DRESP1 sforzo di taglio longherone posteriore
for i=0:last
    id=2+10*i;
    if id<10
        lab=['T00' num2str(id)];
    elseif id<100
        lab=['T0' num2str(id)];
    else
        lab=['T' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               5        %8d\n','DRESP1',1000+id,lab,id);
end
%DRESP1 sforzo di taglio longherone anteriore
for i=0:last
    id=4+10*i;
    if id<10
        lab=['T00' num2str(id)];
    elseif id<100
        lab=['T0' num2str(id)];
    else
        lab=['T' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS  PSHELL               5        %8d\n','DRESP1',1000+id,lab,id);
end
%DRESP2 instabilità a taglio del longherone posteriore
sect=1;
for i=0:last
    id=2+10*i;
    if id<10
        lab=['RSw00' num2str(id)];
        lab2=['h00' num2str(id)];
    elseif id<100
        lab=['RSw0' num2str(id)];
        lab2=['h0' num2str(id)];
    else
        lab=['RSw' num2str(id)];
        lab2=['h' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',6000+id,lab,15005);
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+2);
    fprintf(fout,'%8s%8s%8s\n',' ','DTABLE',lab2);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id+1000);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 instabilità a taglio del longherone anteriore
sect=1;
for i=0:last
    id=4+10*i;
    if id<10
        lab=['RSw00' num2str(id)];
        lab2=['h00' num2str(id)];
    elseif id<100
        lab=['RSw0' num2str(id)];
        lab2=['h0' num2str(id)];
    else
        lab=['RSw' num2str(id)];
        lab2=['h' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',6000+id,lab,15005);
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+4);
    fprintf(fout,'%8s%8s%8s\n',' ','DTABLE',lab2);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id+1000);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 instabilità a flessione del longherone posteriore
sect=1;
for i=0:last
    id=2+10*i;
    if id<10
        lab=['RBw00' num2str(id)];
        lab2=['h00' num2str(id)];
    elseif id<100
        lab=['RBw0' num2str(id)];
        lab2=['h0' num2str(id)];
    else
        lab=['RBw' num2str(id)];
        lab2=['h' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',7000+id,lab,15006);
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+2);
    fprintf(fout,'%8s%8s%8s\n',' ','DTABLE',lab2);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 instabilità a flessione del longherone anteriore
sect=1;
for i=0:last
    id=4+10*i;
    if id<10
        lab=['RBw00' num2str(id)];
        lab2=['h00' num2str(id)];
    elseif id<100
        lab=['RBw0' num2str(id)];
        lab2=['h0' num2str(id)];
    else
        lab=['RBw' num2str(id)];
        lab2=['h' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',7000+id,lab,15006);
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+4);
    fprintf(fout,'%8s%8s%8s\n',' ','DTABLE',lab2);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DRESP2 per il calcolo del margine di sicurezza dei longheroni
for i=0:last
    id=2+10*i;
    if id<10
        lab=['MSw00' num2str(id)];
    elseif id<100
        lab=['MSw0' num2str(id)];
    else
        lab=['MSw' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',8000+id,lab,15007);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DRESP2',6000+id,7000+id);
end
for i=0:last
    id=4+10*i;
    if id<10
        lab=['MSw00' num2str(id)];
    elseif id<100
        lab=['MSw0' num2str(id)];
    else
        lab=['MSw' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',8000+id,lab,15007);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DRESP2',6000+id,7000+id);
end
%DCONSTR del margine di sicurezza dei longheroni
for i=0:last
    id=2+10*i;
    fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',1000+id,8000+id,0.001);
end
for i=0:last
    id=4+10*i;
    fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',1000+id,8000+id,0.001);
end
%Instabilità correnti (Z)
fprintf(fout,'DEQATN     15001 d(Ac,TS,SA)=SQRT(Ac/0.36);\n');
fprintf(fout,'        Scomp=SA*(-TANH(SA)+1.)/2.;\n');
fprintf(fout,'        W=1.7*TS*SQRT(70.e+9/(ABS(Scomp)+10.));\n');
fprintf(fout,'        CG=(-0.5*TS**2*W+0.2232*d**3)/(W*TS+0.36*d**2);\n');
fprintf(fout,'        I=(1./12.)*(W*TS**3+2.*d*(0.12*d**3)+0.12*d**4)\n');
fprintf(fout,'        +TS*W*((CG+0.5*TS)**2)+0.12*(d**2)*\n');
fprintf(fout,'        (CG-0.06*d)**2+0.12*(d**2)*(CG-0.62*d)**2\n');
fprintf(fout,'        +0.12*d**2*(CG-1.18*d)**2;\n');
fprintf(fout,'        raggio=SQRT(I/Ac);L=(0.55*0.7)/raggio;\n');
fprintf(fout,'        instc=1.e+03*(0.0053*L**4-0.399*L**3-42.83*L**2\n');
fprintf(fout,'        +235.84*L+353660.)/(ABS(Scomp)+10.)\n');
%DRESP1 Sforzo di compressione dei correnti
for i=0:last
    id=1+10*i;
    if id<10
        lab=['BS00' num2str(id)];
    elseif id<100
        lab=['BS0' num2str(id)];
    else
        lab=['BS' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS   PBEAM               8        %8d\n','DRESP1',15000+id,lab,1000+id);
end
for i=0:last
    id=4+10*i;
    if id<10
        lab=['BS00' num2str(id)];
    elseif id<100
        lab=['BS0' num2str(id)];
    else
        lab=['BS' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s  STRESS   PBEAM               8        %8d\n','DRESP1',15000+id,lab,1000+id);
end
%DRESP2 Instabilità a compressione dei correnti
sect=1;
for i=0:last
    id=1+10*i;
    if id<10
        lab=['INSTc00' num2str(id)];
    elseif id<100
        lab=['INSTc0' num2str(id)];
    else
        lab=['INSTc' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',9000+id,lab,15001);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DESVAR',section(sect).prop(1)*10+1001,section(sect).prop(1)*10+1);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',15000+id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
sect=1;
for i=0:last
    val=4;
    id=val+10*i;
    if id<10
        lab=['INSTc00' num2str(id)];
    elseif id<100
        lab=['INSTc0' num2str(id)];
    else
        lab=['INSTc' num2str(id)];
    end
    fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',9000+id,lab,15001);
    fprintf(fout,'%8s%8s%8d%8d\n',' ','DESVAR',section(sect).prop(1)*10+1000+val,section(sect).prop(1)*10+3);
    fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',15000+id);
    if i==section(sect).prop(end)
        sect=sect+1;
    end
end
%DCONSTR dell'instabiltà dei correnti
for i=0:last
    val=1;
    id=val+10*i;
    fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',2000+id,9000+id,1.001);
end
for i=0:last
    val=4;
    id=val+10*i;
    fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',2000+id,9000+id,1.001);
end
fclose(fout);