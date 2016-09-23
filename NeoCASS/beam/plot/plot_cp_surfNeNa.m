%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

function plot_cp_surfNeNa(scale,def)
global beam_model
% soluzione nastran:
% alpha beta p q r delatf1, deltaf2, deltaa, deltar, deltac


if def == 1
    ntrim = 2;
else
    ntrim = 1;
end

colore{1} = 'red';
colore{2} = 'red';
coloreN{1} = 'green';
coloreN{2} = 'green';


wing.x = [];wing.y = [];wing.z = [];
canard.x = [];canard.y = [];canard.z = [];
htail.x = [];htail.y = [];htail.z = [];
vtail2.x = [];vtail2.y = [];vtail2.z = [];
vtail.x = [];vtail.y = [];vtail.z = [];

wing.xn = [];wing.yn = [];wing.zn = [];
canard.xn = [];canard.yn = [];canard.zn = [];
htail.xn = [];htail.yn = [];htail.zn = [];
vtail2.xn = [];vtail2.yn = [];vtail2.zn = [];
vtail.xn = [];vtail.yn = [];vtail.zn = [];


for n = 1 : ntrim
    
    % CPna = load('CPNastran.dat');
    if n ==1
        CFna = load('FNastranDelta.dat');
    else
        CFna = load('FNastrandefHinge.dat');
    end
    % CFna = load('FNastran.dat');
%     load('beam_modelCP.mat');
    
    sol = [beam_model.Res.FM.Value(n,2:6),beam_model.Res.CS.Value(n,beam_model.Aero.Trim.CS.MPC == 0)];
    sol(1) =0;
    sol(end) =  0.0175;
    
    [rho, p, T, a] = ISA_h(beam_model.Aero.state.ALT);
    
    pref = 0.5*rho*(a*beam_model.Aero.state.Mach)^2/scale;
    
    dof = (1:length(beam_model.Aero.lattice_vlm.N))';
    dofCS = [];
    for i = 1 :length(beam_model.Aero.geo.nx)
        DOF = beam_model.Aero.lattice_vlm.DOF(i,1,1) :beam_model.Aero.lattice_vlm.DOF(i,1,2);
        for j = 1:beam_model.Aero.geo.ny(i)
            dofCS = [dofCS;DOF(beam_model.Aero.geo.nx(i)*j+beam_model.Aero.geo.fnx(i)*(j-1)+1:j*(beam_model.Aero.geo.nx(i)+beam_model.Aero.geo.fnx(i)))'];
        end
    end
    dofFIX = setdiff(dof,dofCS);
    [dummy,ind]= sort([dofFIX;dofCS]);
    
    % CPna = CPna(ind,:);
    CFna = CFna(ind,:);
    
    CPne = beam_model.Res.CPaero.F0;
    
    for i = 1 : 5
        CPne = CPne +  squeeze(beam_model.Res.CPaero.State(:,:,i))*sol(i);
    end
    
    for i = 1 : length(find(beam_model.Aero.Trim.CS.MPC == 0))
        CPne = CPne +  beam_model.Res.CPaero.Control(:,:,i)*sol(i+5);
    end
    
    if n == 2
        CPned = zeros(length(dof),3);
        
        
        for m = 1:beam_model.Info.ngrid 
            dof = beam_model.Node.DOF(m, 1:6);
            % DISPLACEMENT DOF
            index = find(dof);
            if ~isempty(index)
                for jj = 1 : length(index)
                    if index(jj)<=3
                       CPned = CPned +  beam_model.Res.CPaero.Def(:,:,dof(index(jj)))*beam_model.Res.NDispl(m,index(jj));
                    else
                        CPned = CPned + beam_model.Res.CPaero.Def(:,:,dof(index(jj))) * beam_model.Res.NDispl(m,index(jj));
                    end
                end
            end
%             for k=1:6
%                 % loop on node displacements DOFs
%                 if (beam_model.Node.DOF(N,k))
%                     if N == 66
%                        pippo = 1; 
%                     end
%                     CPned = CPned + beam_model.Res.CPaero.Def(:,:,i) * beam_model.Res.NDispl(N,k);
% 
%                 end
%             end
%             %
            
        end
         CPne = CPne + CPned;
    end
    
    for i = 1 : size(CPne,1)
        if  beam_model.Aero.lattice_vlm.COLLOC(i,2)<0
            CFna(i,3:end) = -CFna(i,3:end);
        end
    end
    
    
    accz = sum((CPne(:,3)))/beam_model.WB.MCG(1,1)
%     accy = [sum((CPne(:,2))),sum((CFna(:,4)))]/2.230819E+05;
    
    
    CPne = -dot(CPne,beam_model.Aero.lattice_vlm.N,2);
    
    
    
    % CPne = CPne/(max(abs(CPne)))*10;
    % CFna = CFna/(max(abs(CFna(:,5))))*10;
%     if n == 1
        figsurf = plot_beam_model(n);
        figCP = plot_beam_model(n+10);
%     else
%         
%     end
    figure(figsurf);
    hold on
    for i = 1 : size(beam_model.Aero.lattice_vlm.DOF,1)%length(dof)%3
        
        nx = beam_model.Aero.geo.nx(i)+beam_model.Aero.geo.fnx(i);
        ny = beam_model.Aero.geo.ny(i);
        
        if (beam_model.Aero.ID(i)>=300 && beam_model.Aero.ID(i)<400)
            span = reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),5,3),nx,ny)-...
                reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,3),nx,ny);
            
            Xcol = reshape(beam_model.Aero.lattice_vlm.COLLOC(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),1),nx,ny);
            
            
            cord = -2*(0.5*(reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,1),nx,ny)+...
                reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),5,1),nx,ny))...
                - Xcol);
            
            span = abs(span);
            Suf = span.*cord;
            
            X = reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,1),nx,ny);
            X = [X,beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),5,1)];
            
            Y = reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,3),nx,ny);
            Y = [Y,beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),5,3)];
            
            Ycp = reshape(beam_model.Aero.lattice_vlm.COLLOC(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),3),nx,ny);
            
            Zne =  reshape( CPne(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),1),nx,ny)./(Suf*pref);
            Zna =  reshape( CFna(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),5),nx,ny)./(Suf*pref);
            
            IntCpne = sum(cord.*Zne);
            Xcpne   =  sum(Zne.*Xcol.*cord)./(sum(Zne.*cord));
            
            IntCpna = sum(cord.*Zna);
            Xcpna=  sum(Zna.*Xcol.*cord)./(sum(Zna.*cord));
            
            if beam_model.Aero.ID(i)>=350
                vtail2.x = [vtail2.x,Xcpne];
                vtail2.y = [vtail2.y,IntCpne];
                vtail2.z = [vtail2.z,Ycp(1,:)];
                
                vtail2.xn = [vtail2.xn,Xcpna];
                vtail2.yn = [vtail2.yn,IntCpna];
                vtail2.zn = [vtail2.zn,Ycp(1,:)];
            else
                vtail.x = [vtail2.x,Xcpne];
                vtail.y = [vtail2.y,IntCpne];
                vtail.z = [vtail2.z,Ycp(1,:)];
                
                vtail.xn = [vtail2.xn,Xcpna];
                vtail.yn = [vtail2.yn,IntCpna];
                vtail.zn = [vtail2.zn,Ycp(1,:)];
            end
            
            Zne = [Zne,CPne(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),1)./(Suf(:,end)*pref)];
            

            Zna = [Zna,CFna(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),5)./(Suf(:,end)*pref)];
            
            surf(X,Zne,Y,'facecolor',colore{n})%,'FaceAlpha',0.7)
            surf(X,Zna,Y,'facecolor',coloreN{n})%,'FaceAlpha',0.7)
            
        else
            span = reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),5,2),nx,ny)-...
                reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,2),nx,ny);
            
                        Xcol = reshape(beam_model.Aero.lattice_vlm.COLLOC(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),1),nx,ny);
            
            cord = -2*(0.5*(reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,1),nx,ny)+...
                reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),5,1),nx,ny))...
                - Xcol);
            span = abs(span);
            
            Suf = span.*cord;
            
            X = reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,1),nx,ny);
            X = [X,beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),5,1)];
            
            Y = reshape(beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),4,2),nx,ny);
            Y = [Y,beam_model.Aero.lattice_vlm.VORTEX(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),5,2)];
        
            Zne =  reshape( CPne(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),1),nx,ny)./(Suf*pref);
            Zna =  reshape( CFna(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),5),nx,ny)./(Suf*pref);
           
            Ycp = reshape(beam_model.Aero.lattice_vlm.COLLOC(beam_model.Aero.lattice_vlm.DOF(i,1):beam_model.Aero.lattice_vlm.DOF(i,2),2),nx,ny);
            
            IntCpne = sum(cord.*Zne);
            Xcpne   =  sum(Zne.*Xcol.*cord)./(sum(Zne.*cord));
            
            IntCpna = sum(cord.*Zna);
            Xcpna   =  sum(Zna.*Xcol.*cord)./(sum(Zna.*cord));
            
            if beam_model.Aero.ID(i)>=200 && beam_model.Aero.ID(i)<300  % wing
                wing.x = [wing.x,Xcpne];
                wing.y = [wing.y,Ycp(1,:)];
                wing.z = [wing.z,IntCpne];
                wing.xn = [wing.xn,Xcpna];
                wing.yn = [wing.yn,Ycp(1,:)];
                wing.zn = [wing.zn,IntCpna];
            end
            if beam_model.Aero.ID(i)>=400 && beam_model.Aero.ID(i)<500  % htail
                htail.x = [htail.x,Xcpne];
                htail.y = [htail.y,Ycp(1,:)];
                htail.z = [htail.z,IntCpne];
                htail.xn = [htail.xn,Xcpna];
                htail.yn = [htail.yn,Ycp(1,:)];
                htail.zn = [htail.zn,IntCpna];
            end
            
            if beam_model.Aero.ID(i)>=500 %canard
                canard.x = [canard.x,Xcpne];
                canard.y = [canard.y,Ycp(1,:)];
                canard.z = [canard.z,IntCpne];
                canard.xn = [canard.xn,Xcpna];
                canard.yn = [canard.yn,Ycp(1,:)];
                canard.zn = [canard.zn,IntCpna];
            end
            
            Zne = [Zne,CPne(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),1)./(Suf(:,end)*pref)];
            

            Zna = [Zna,CFna(beam_model.Aero.lattice_vlm.DOF(i,2)-nx+1:beam_model.Aero.lattice_vlm.DOF(i,2),5)./(Suf(:,end)*pref)];
            legsurf(1) = surf(X,Y,Zne,'facecolor',colore{n});%,'FaceAlpha',0.7);
            legsurf(2) = surf(X,Y,Zna,'facecolor',coloreN{n});%,'FaceAlpha',0.7) ;
        end
        %     end
    end
    
    figure(figCP);
    hold on
    % plot span distribution
    % wing
    [wing.y,sorted] = sort(wing.y);
    wing.x = wing.x(sorted);
    wing.z = wing.z(sorted);
    
    
    wing.yn = wing.yn(sorted);
    wing.xn = wing.xn(sorted);
    wing.zn = wing.zn(sorted);
    
    legCPy(1) = plot3(wing.x,wing.y,wing.z,'linewidth',2,'color',colore{n});
    legCPy(2) = plot3(wing.xn,wing.yn,wing.zn,'linewidth',2,'color',coloreN{n});
    wing.x = []; wing.y = []; wing.z = [];
    wing.xn = []; wing.yn = []; wing.zn = [];
    
    if ~isempty(htail.x)
        [htail.y,sorted] = sort(htail.y);
        htail.x = htail.x(sorted);
        htail.z = htail.z(sorted);
        
        htail.yn = htail.yn(sorted);
        htail.xn = htail.xn(sorted);
        htail.zn = htail.zn(sorted);
        plot3(htail.x,htail.y,htail.z,'linewidth',2,'color',colore{n})
        plot3(htail.xn,htail.yn,htail.zn,'linewidth',2,'color',coloreN{n})
    end
    htail.x = []; htail.y = []; htail.z = [];
    htail.xn = []; htail.yn = []; htail.zn = [];
    
    if ~isempty(canard.x)
        [canard.y,sorted] = sort(canard.y);
        canard.x = canard.x(sorted);
        canard.z = canard.z(sorted);
        
        canard.yn = canard.yn(sorted);
        canard.xn = canard.xn(sorted);
        canard.zn = canard.zn(sorted);
        plot3(canard.x,canard.y,canard.z,'linewidth',2,'color',colore{n})
        plot3(canard.xn,canard.yn,canard.zn,'linewidth',2,'color',coloreN{n})
    end
    canard.x = []; canard.y = []; canard.z = [];
    canard.xn = []; canard.yn = []; canard.zn = [];
    
    if ~isempty(vtail.x)
        [vtail.z,sorted] = sort(vtail.z);
        vtail.x = vtail.x(sorted);
        vtail.y = vtail.y(sorted);
        
        vtail.xn = vtail.xn(sorted);
        vtail.yn = vtail.yn(sorted);
        vtail.zn = vtail.zn(sorted);
        
        plot3(vtail.x,vtail.y,vtail.z,'linewidth',2,'color',colore{n})
        plot3(vtail.xn,vtail.yn,vtail.zn,'linewidth',2,'color',coloreN{n})
    end
    vtail.x = []; vtail.y = []; vtail.z = [];
    vtail.xn = []; vtail.yn = []; vtail.zn = [];
    
    if ~isempty(vtail2.x)
        [vtail2.z,sorted] = sort(vtail2.z);
        vtail2.x = vtail2.x(sorted);
        vtail2.y = vtail2.y(sorted);
        
        vtail2.xn = vtail2.xn(sorted);
        vtail2.yn = vtail2.yn(sorted);
        vtail2.zn = vtail2.zn(sorted);
        
        plot3(vtail2.x,vtail2.y,vtail2.z,'linewidth',2,'color',colore{n})
        plot3(vtail2.xn,vtail2.yn,vtail2.zn,'linewidth',2,'color',coloreN{n})
    end
    vtail2.x = []; vtail2.y = []; vtail2.z = [];
    vtail2.xn = []; vtail2.yn = []; vtail2.zn = [];
    if n==1
        figure(figsurf);
        axis equal
        h = legend(legsurf,'NeoCASS','Nastran');
        set(h,'fontsize',18);
        set(h,'box','off');
        
        figure(figCP);
        axis equal
        h = legend(legCPy,'NeoCASS','Nastran');
        set(h,'fontsize',18);
        set(h,'box','off');
    else
         figure(figsurf);
        axis equal
        h = legend(legsurf,'NeoCASS','Nastran');
        set(h,'fontsize',18);
        set(h,'box','off');
        
        figure(figCP);
        axis equal
        h = legend(legCPy,'NeoCASS','Nastran');
        set(h,'fontsize',18);
        set(h,'box','off');
    end
end

    
%     figure(figsurf);
%     axis equal
%     h = legend(legsurf,'NeoCASS Rigid','NeoCASS Deformable','Nastran Rigid','Nastran Deformable');
%     set(h,'fontsize',18);
%     set(h,'box','off');
%     
%     figure(figCP);
%     axis equal
%     h = legend(legCPy,'NeoCASS Rigid','NeoCASS Deformable','Nastran Rigid','Nastran Deformable');
%     set(h,'fontsize',18);
%     set(h,'box','off');

