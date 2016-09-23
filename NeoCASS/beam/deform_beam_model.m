function beam_model= deform_beam_model(beam_model,pdcylin,semispan)



% get the id of the wing nodes
idw = (find(beam_model.Bar.ID>=2000 & beam_model.Bar.ID<3000));
idWN =  unique([beam_model.Bar.Conn(idw,1);beam_model.Bar.Conn(idw,2);beam_model.Bar.Conn(idw,3)]);
idwing = beam_model.Node.ID(idWN);
if  isfield(pdcylin,'deformation') && isfield(pdcylin.deformation,'bending')
    ss = size(beam_model.Node.DOF);
    
    beam_model.Node.DOF = reshape([1:ss(1)*ss(2)]',ss(2),ss(1))';
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
    
    aeroelastic_interface
    beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
    
    % bending
    SOL = zeros(max(max(beam_model.Node.DOF)),1);
    
    
    if  isfield(pdcylin.deformation,'torsion')
        % torsion
        for i = 1:length (beam_model.Node.ID)
            
            if ismember(beam_model.Node.ID(i),idwing ) || ((beam_model.Node.ID(i)>=20000) && (beam_model.Node.ID(i)<30000))
                for k = 1 : length(pdcylin.deformation.torsion.coefficients)
                    SOL(beam_model.Node.DOF(i,5)) = SOL(beam_model.Node.DOF(i,5)) + ...
                        (pdcylin.deformation.torsion.coefficients(k)* abs(beam_model.Node.Coord(i,2)/semispan).^k)*pdcylin.deformation.torsion.max_tip_value;
                end
                
                
            end
            
        end
        
        beam_model.Aero.lattice_vlm = update_vlm_mesh1(beam_model.Node, SOL, beam_model.Aero);
        
    end
    
    
    
    for i = 1:length (beam_model.Node.ID)
        
        if ismember(beam_model.Node.ID(i),idwing ) || ((beam_model.Node.ID(i)>=20000) && (beam_model.Node.ID(i)<30000))
            
            for k = 1 : length(pdcylin.deformation.bending.coefficients)
                beam_model.Node.Coord(i,3) = beam_model.Node.Coord(i,3) + ...
                    (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Node.Coord(i,2)/semispan).^k)*pdcylin.deformation.bending.max_tip_value;
            end
            
        end
        
    end
    
    for i = 1 : length(idw)
        
        for k = 1 : length(pdcylin.deformation.bending.coefficients)
            beam_model.Bar.Colloc(1,3,idw(i)) = beam_model.Bar.Colloc(1,3,idw(i)) + ...
                (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Bar.Colloc(1,2,idw(i))/semispan).^k)*pdcylin.deformation.bending.max_tip_value;
            
            beam_model.Bar.Colloc(2,3,idw(i)) = beam_model.Bar.Colloc(2,3,idw(i)) + ...
                (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Bar.Colloc(2,2,idw(i))/semispan).^k)*pdcylin.deformation.bending.max_tip_value;    
        end
        
    end
    dof = [];
    for j = 1 : length(beam_model.Aero.ID)
        
        if (beam_model.Aero.ID(j) >=200) && (beam_model.Aero.ID(j) <300)
            d0 = beam_model.Aero.lattice_vlm.DOF(j,1,1);
            d1 = beam_model.Aero.lattice_vlm.DOF(j,1,2);
            dof = [dof,d0:d1];
            for k = 1 : length(pdcylin.deformation.bending.coefficients)
                beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) = beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) + ...
                    (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.COLLOC(d0:d1,2)/semispan).^k)*pdcylin.deformation.bending.max_tip_value;
                beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) = beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) + ...
                    (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,2)/semispan).^k)*pdcylin.deformation.bending.max_tip_value;
                beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) = beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) + ...
                    (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,2)/semispan).^k)*pdcylin.deformation.bending.max_tip_value;
            end
        end
        
    end
    
    for j =1 : length(beam_model.Aero.lattice_vlm.Control.Patch)
        locdof =  beam_model.Aero.lattice_vlm.Control.DOF(j).data;
        ind = intersect(locdof,dof);
        if length(ind)>0
            for k = 1 : length(pdcylin.deformation.bending.coefficients)
                beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) = beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) + ...
                    (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.Control.Hinge(j,1,2)/semispan).^k)*pdcylin.deformation.bending.max_tip_value;
                beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) = beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) + ...
                    (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.Control.Hinge(j,2,2)/semispan).^k)*pdcylin.deformation.bending.max_tip_value;
            end
        end
    end
    
    
    
end