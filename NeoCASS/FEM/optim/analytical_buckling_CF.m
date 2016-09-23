function[]=analytical_buckling_CF(section)
last=section(end).prop(end);
fout=fopen('analytical_buckling.dat','w');
%DRESP1 Sforzo assiale 
for k=1:4
    for i=0:last
        id=k+10*i;
        if id<10
            lab=['AX00' num2str(id)];
        elseif id<100
            lab=['AX0' num2str(id)];
        else
            lab=['AX' num2str(id)];
        end
        fprintf(fout,'%8s%8d%8s CSTRESS   PCOMP               3       8%8d\n','DRESP1',id,lab,id);
    end
end
%DRESP1 Sforzo di taglio 
for k=1:4
    for i=0:last
        id=k+10*i;
        if id<10
            lab=['S00' num2str(id)];
        elseif id<100
            lab=['S0' num2str(id)];
        else
            lab=['S' num2str(id)];
        end
        fprintf(fout,'%8s%8d%8s CSTRESS   PCOMP               5       8%8d\n','DRESP1',id+1000,lab,id);
    end
end
fprintf(fout,'$Instabilita del pannello in composito\n');
fprintf(fout,'$Compressione pannello\n');
fprintf(fout,'DEQATN     15000 a(SA,Tlam)=0.55;b=0.16;\n');
fprintf(fout,'        Scomp=SA*(-TANH(SA)+1.)/2.;T=Tlam/8.;\n');
fprintf(fout,'        D11=1.e-3*(4.5398e+15*(T**3));\n');
fprintf(fout,'        D12=1.e-3*(7.0136e+14*(T**3));\n');
fprintf(fout,'        D22=1.e-3*(1.1367e+15*(T**3));\n');
fprintf(fout,'        D66=1.e-3*(8.0232e+14*(T**3));\n');
fprintf(fout,'        m3=3.;Scr3=((3.1415**2)/(b**2))*(D11*(m3**2)*((b/a)**2)+\n');
fprintf(fout,'        2.*(D12+2.*D66)+D22*((a/b)**2)*((1./m3)**2));\n');
fprintf(fout,'        Scr=Scr3/Tlam;\n');
fprintf(fout,'        RLp=abs(Scomp)/Scr\n');
fprintf(fout,'$Taglio pannello\n');
fprintf(fout,'DEQATN     15003 b(tau,Tlam)=0.16;T=Tlam/8.;\n');
fprintf(fout,'        D11=1.e-3*(4.5398e+15*(T**3));\n');
fprintf(fout,'        D12=1.e-3*(7.0136e+14*(T**3));\n');
fprintf(fout,'        D22=1.e-3*(1.1367e+15*(T**3));\n');
fprintf(fout,'        D66=1.e-3*(8.0232e+14*(T**3));\n');
fprintf(fout,'        K=(D12+2.*D66)/(SQRT(D11*D22));\n');
fprintf(fout,'        Ssh=(4./(b**2))*((D11*(D22**3))**0.25)*\n');
fprintf(fout,'        (8.125+5.045*K)/Tlam;\n');
fprintf(fout,'        RSp=abs(tau)/Ssh\n');
fprintf(fout,'DEQATN     15004 MSp(RL,RS)=(2./(RL+(RL**2+4.*(RS**2))**0.5)\n');
fprintf(fout,'        +0.001)-1.\n');
%DRESP2 instabilità pannello  a compressione
for k=[1 3]
    sect=1;
    for i=0:last
        id=k+10*i;
        id2=6000+10*section(sect).prop(1)+k;
        if id<10
            lab=['RLp00' num2str(id)];
        elseif id<100
            lab=['RLp0' num2str(id)];
        else
            lab=['RLp' num2str(id)];
        end
        fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',5000+id,lab,15000);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP2',id2);
        if i==section(sect).prop(end)
            sect=sect+1;
        end
    end
end

%DRESP2 instabilità pannello superiore a taglio
for k=[1 3]
    sect=1;
    for i=0:last
        id=k+10*i;
        id2=6000+10*section(sect).prop(1)+k;
        if id<10
            lab=['RSp00' num2str(id)];
        elseif id<100
            lab=['RSp0' num2str(id)];
        else
            lab=['RSp' num2str(id)];
        end
        fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',7000+id,lab,15003);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id+1000);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP2',id2);
        if i==section(sect).prop(end)
            sect=sect+1;
        end
    end
end
%DRESP2 per il calcolo del margine di sicurezza
for k=[1 3]
    for i=0:last
        id=k+10*i;
        if id<10
            lab=['MSp00' num2str(id)];
        elseif id<100
            lab=['MSp0' num2str(id)];
        else
            lab=['MSp' num2str(id)];
        end
        fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',8000+id,lab,15004);
        fprintf(fout,'%8s%8s%8d%8d\n',' ','DRESP2',5000+id,7000+id);
    end
end
%DCONSTR del margine di sicurezza dei pannelli
for k=[1 3]
    for i=0:last
        id=k+10*i;
        fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',1000+id,8000+id,0.001);
    end
end
fprintf(fout,'$Instabilita del longherone in composito\n');
fprintf(fout,'$Bending pannello\n');
fprintf(fout,'DEQATN     15005 b(h,SA,Tlam)=h;T=Tlam/8.;\n');
fprintf(fout,'        D11=1.e-3*(4.5398e+15*(T**3));\n');
fprintf(fout,'        D12=1.e-3*(7.0136e+14*(T**3));\n');
fprintf(fout,'        D22=1.e-3*(1.1367e+15*(T**3));\n');
fprintf(fout,'        D66=1.e-3*(8.0232e+14*(T**3));\n');
fprintf(fout,'        Sb=((3.1415**2)/(b**2))*(13.9*SQRT(D11*D22)+11.1*(D12+2.*D66));\n');
fprintf(fout,'        RBw=abs(SA)/(Sb/Tlam)\n');
fprintf(fout,'$Taglio longherone in composito\n');
fprintf(fout,'DEQATN     15006 b(tau,Tlam)=0.16;T=Tlam/8.;\n');
fprintf(fout,'        D11=1.e-3*(4.5398e+15*(T**3));\n');
fprintf(fout,'        D12=1.e-3*(7.0136e+14*(T**3));\n');
fprintf(fout,'        D22=1.e-3*(1.1367e+15*(T**3));\n');
fprintf(fout,'        D66=1.e-3*(8.0232e+14*(T**3));\n');
fprintf(fout,'        K=(D12+2.*D66)/(SQRT(D11*D22));\n');
fprintf(fout,'        Ssh=(4./(b**2))*((D11*(D22**3))**0.25)*\n');
fprintf(fout,'        (8.125+5.045*K);\n');
fprintf(fout,'        RSw=abs(tau)/(Ssh/Tlam)\n');
fprintf(fout,'DEQATN     15007 MSp(RL,RS)=(2./(RL+(RL**2+4.*(RS**2))**0.5)\n');
fprintf(fout,'        +0.001)-1.\n');
%DRESP2 instabilità a flessione del longherone posteriore
for k=[2 4]
    sect=1;
    for i=0:last
        id=k+10*i;
        id2=6000+10*section(sect).prop(1)+k;
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
        fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',5000+id,lab,15005);
        fprintf(fout,'%8s%8s%8s\n',' ','DTABLE',lab2);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP2',id2);
        if i==section(sect).prop(end)
            sect=sect+1;
        end
    end
end
%DRESP2 instabilità a taglio del longherone 
for k=[2 4]
    sect=1;
    for i=0:last
        id=k+10*i;
        id2=6000+10*section(sect).prop(1)+k;
        if id<10
            lab=['RSw00' num2str(id)];
        elseif id<100
            lab=['RSw0' num2str(id)];
        else
            lab=['RSw' num2str(id)];
        end
        fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',7000+id,lab,15006);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP1',id+1000);
        fprintf(fout,'%8s%8s%8d\n',' ','DRESP2',id2);
        if i==section(sect).prop(end)
            sect=sect+1;
        end
    end
end
%DRESP2 per il calcolo del margine di sicurezza dei longheroni
for k=[2 4]
    for i=0:last
        id=k+10*i;
        if id<10
            lab=['MSw00' num2str(id)];
        elseif id<100
            lab=['MSw0' num2str(id)];
        else
            lab=['MSw' num2str(id)];
        end
        fprintf(fout,'%8s%8d%8s%8d\n','DRESP2',8000+id,lab,15007);
        fprintf(fout,'%8s%8s%8d%8d\n',' ','DRESP2',5000+id,7000+id);
    end
end
%DCONSTR del margine di sicurezza dei longheroni
for k=[2 4]
    for i=0:last
        id=k+10*i;
        fprintf(fout,'%8s%8d%8d%8.3f\n','DCONSTR',1000+id,8000+id,0.001);
    end
end


%Instabilità correnti (Z)
fprintf(fout,'DEQATN     15001 d(Ac,SA)=SQRT(Ac/0.36);\n');
fprintf(fout,'        Scomp=SA*(-TANH(SA)+1.)/2.;TS=1.;\n');
fprintf(fout,'        W=0.0;\n');
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
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+1001);
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
    fprintf(fout,'%8s%8s%8d\n',' ','DESVAR',section(sect).prop(1)*10+1000+val);
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