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

%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
function plot_control_aelink2(nfig)

global beam_model;

if (beam_model.Aero.geo.nc)
    
    index = find(beam_model.Aero.geo.flapped);
    CSURF_MODES = [];
    DSURF_MODES = [];
    NSURF_MODES = [];
    %
    AMPL = D2R(45);
    %
    if (beam_model.Info.amesh_av_vlm)
        lattice = beam_model.Aero.lattice_vlm;
        cref = 1;
        x1 = mean(beam_model.Aero.lattice_vlm.VORTEX(:,4:5,1),2);
        x2 = mean(beam_model.Aero.lattice_vlm.VORTEX(:,4:5,2),2);
        x3 = mean(beam_model.Aero.lattice_vlm.VORTEX(:,4:5,3),2);
        lattice.MID_DPOINT = [x1, x2, x3];
        beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
    elseif (beam_model.Info.amesh_av_dlm)
        lattice = beam_model.Aero.lattice_dlm;
        cref = beam_model.Aero.ref.C_mac;
        beam_model.Aero.lattice = beam_model.Aero.lattice_dlm;
    end
    cref
    for n=1:length(index)
        REF_POINT(1:3) = lattice.Control.Hinge(n,1,:);
        HINGE_AXIS(1:3) = lattice.Control.Hinge(n,2,:) - lattice.Control.Hinge(n,1,:);
        HINGE_AXIS = HINGE_AXIS ./norm(HINGE_AXIS);
        %       construct fictious modes for control surfaces
        [A, D, B] = ...
            set_patch_crigid_bc(cref, [1 0 0], lattice.MID_DPOINT(:,:), lattice.COLLOC(:,:),...
            lattice.XYZ(:,:,:), lattice.N(:,:), REF_POINT, lattice.Control.DOF(n).data, HINGE_AXIS, ...
            AMPL, 1, true, false);
        NSURF_MODES(:,:,n) = B;
    end
    %
    nptot = size(lattice.COLLOC,1);
    
    if (beam_model.Info.nlink)
        
        %     nfig = nfig;
        
        counter = length(unique(beam_model.Aero.Trim.Link.Master));
        MASTER = unique(beam_model.Aero.Trim.Link.Master);
        SLAVE = unique(beam_model.Aero.Trim.Link.Slave);
        master = zeros(counter,1);
        slave = master;
        sl_control_index = zeros(counter, 2);
        for nmas = 1:length(MASTER)
            for ncontrols = 1:beam_model.Aero.geo.nc
                if (isequal(MASTER{nmas}, lattice.Control.Name{ncontrols})) 
                    master(nmas) = ncontrols;
                    break;
                end
            end
        end
        for nmas = 1:length(SLAVE)
            for ncontrols = 1:beam_model.Aero.geo.nc
                if (isequal(SLAVE{nmas}, lattice.Control.Name{ncontrols}))
                    slave(nmas) = ncontrols;
                    break;
                end
            end
        end
%         Ndisp_TOT =zeros(size(NSURF_MODES,1),size(NSURF_MODES,2));
        figure(nfig);
        if length(MASTER)+length(SLAVE)<beam_model.Aero.geo.nc
            NMS = length(MASTER)+length(SLAVE);
            NMASTER = length(MASTER);
            n1 = ceil(sqrt(NMASTER + beam_model.Aero.geo.nc-NMS)) ;
            n2 = round(sqrt(NMASTER + beam_model.Aero.geo.nc-NMS));
            
            IND = setdiff(1:beam_model.Aero.geo.nc,[master;slave]);
             
            
            for m=1:length(MASTER)
                
                n_displ = NSURF_MODES(:,:,master(m));
                dofsurf = beam_model.Aero.lattice.Control.DOF(master(m)).data;
                for k=1:length(beam_model.Aero.Trim.Link.Slave)
                    if (isequal(beam_model.Aero.Trim.Link.Master{k}, MASTER{m}))
                        for ncontrols = 1:beam_model.Aero.geo.nc
                            if (isequal(beam_model.Aero.Trim.Link.Slave{k}, lattice.Control.Name{ncontrols}))
                                control_index(nmas,ncontrols) = ncontrols;
                                break;
                            end
                        end
                        dofsurf = [dofsurf,beam_model.Aero.lattice.Control.DOF(ncontrols).data];
                        n_displ = n_displ + NSURF_MODES(:,:,ncontrols).*beam_model.Aero.Trim.Link.Coeff(k);
                    end
                end
                %             Ndisp_TOT = Ndisp_TOT + n_displ;
                subplot(n1,n2,m)
                plot_cmotion(dofsurf, cref, n_displ , lattice);
                t = ['Master control: ', MASTER{m}];
                title(t);
            end
            
            for mm = 1 :length(IND)
                subplot(n1,n2,m+mm)
                dofsurf = beam_model.Aero.lattice.Control.DOF(IND(mm)).data;
                plot_cmotion(dofsurf, cref, NSURF_MODES(:,:,IND(mm)) , lattice);
                t = ['Master control: ', beam_model.Aero.lattice_vlm.Control.Name{IND(mm)}];
                title(t);
            end
        else
            n1 = ceil(sqrt(length(MASTER)));
            n2 = round(sqrt(length(MASTER)));
            
            for m=1:length(MASTER)
                
                n_displ = NSURF_MODES(:,:,master(m));
                dofsurf = beam_model.Aero.lattice.Control.DOF(master(m)).data;
                for k=1:length(beam_model.Aero.Trim.Link.Slave)
                    if (isequal(beam_model.Aero.Trim.Link.Master{k}, MASTER{m}))
                        for ncontrols = 1:beam_model.Aero.geo.nc
                            if (isequal(beam_model.Aero.Trim.Link.Slave{k}, lattice.Control.Name{ncontrols}))
                                control_index(nmas,ncontrols) = ncontrols;
                                break;
                            end
                        end
                        dofsurf = [dofsurf,beam_model.Aero.lattice.Control.DOF(ncontrols).data];
                        n_displ = n_displ + NSURF_MODES(:,:,ncontrols).*beam_model.Aero.Trim.Link.Coeff(k);
                    end
                end
                %             Ndisp_TOT = Ndisp_TOT + n_displ;
                subplot(n1,n2,m)
                plot_cmotion(dofsurf, cref, n_displ , lattice);
                t = ['Master control: ', MASTER{m}];
                title(t);
            end
        end
        %       nfig = nfig+1;
        % close(nfig); figure(nfig);
        %       figure(nfig); close(nfig); figure(nfig);
        
%         t = ['Master control: ', MASTER{m}];
%         title(t);
        
    else
        
        fprintf(beam_model.Param.FID, '\n No control surface lik available. Single control rotation showed.');
        figure(nfig);
        n1 = ceil(sqrt(size(NSURF_MODES,3)));
        n2 = round(sqrt(size(NSURF_MODES,3)));
        for n=1:size(NSURF_MODES,3)
%             nfig = nfig+1;
%             figure(nfig); close(nfig); figure(nfig);
%             plot_cmotion(nptot, cref, NSURF_MODES(:,:,n), lattice);
            dofsurf = beam_model.Aero.lattice.Control.DOF(n).data;
            subplot(n1,n2,n)
            plot_cmotion(dofsurf, cref, NSURF_MODES(:,:,n) , lattice);
%             t = ['Master control: ', MASTER{m}];
%             title(t);
        end
%         figure(nfig)
%         plot_cmotion(nptot, cref, sum(NSURF_MODES,3), lattice);
        
    end % link available
    
else
    plot_beam_model(nfig)
    fprintf(beam_model.Param.FID, '\n No control surface available.');
    
end % surf available

end
%*******************************************************************************
function plot_cmotion(nptot, cref, n_displ, lattice)
XYZ = zeros(length(nptot),5,3);
% count  = 0;
for k = 1:length(nptot)
    count = (nptot(k)-1)*4;
    for j = 1:4
        count = count +1;
        XYZ(k, j, 1:3) = n_displ(count, 1:3);
    end
    XYZ(k, 5, 1:3) = XYZ(k, 1, 1:3);
end
XYZ = XYZ + cref .* lattice.XYZ(nptot,:,:);
hold on
ind = setdiff(1:size(lattice.XYZ,1), nptot);
plot3(XYZ(:,:,1)',XYZ(:,:,2)',XYZ(:,:,3)','-go',  'MarkerSize', 2, 'MarkerFaceColor','b');
plot3( cref .* lattice.XYZ(ind,:,1)', cref .* lattice.XYZ(ind,:,2)' ,cref .* lattice.XYZ(ind,:,3)' ,'-mo',  'MarkerSize', 2, 'MarkerFaceColor','b');
grid on;

view([37.5 30]);
axis equal;
end
