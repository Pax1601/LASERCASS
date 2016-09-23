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
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%*******************************************************************************
% This function interpolates CAERO forces to GUESS stick model for structural sizing
%
% geo struct
% stick struct
% Aero from beam_model.Aero
% lattice from beam_model.Aero.lattice_vlm
% toll pinv tolerance (set default value to zero)
% FORCES [Fx Fy Fz] along CAERO. Run solve_free_lin_trim_guess_std. 
% These values are stored as .Res.Aero.Fa
%
function [loads] = interf_thick_model(pdcylin, geo, stick, Aero, toll, Res)
%
%
  loads.winr.FS = [];
  loads.winr.M = [];
  loads.winr.Mt = [];
  loads.winl.FS = [];
  loads.winl.M = [];
  loads.winl.Mt = [];
  loads.vert.FS = [];
  loads.vert.M = [];
  loads.vert.Mt = [];
  loads.vert2.FS = [];
  loads.vert2.M = [];
  loads.vert2.Mt = [];
  loads.horr.FS = [];
  loads.horr.M = [];
  loads.horr.Mt = [];
  loads.horl.FS = [];
  loads.horl.M = [];
  loads.horl.Mt = [];
  loads.canr.FS = [];
  loads.canr.M = [];
  loads.canr.Mt = [];
  loads.canl.FS = [];
  loads.canl.M = [];
  loads.canl.Mt = [];
  loads.fus.FS = [];
  loads.fus.M = [];
  loads.fus.Mt = [];
%
  PLOT_TEST = 0;
%
  FWR = zeros(1,3);  MWR = zeros(1,3);
  FWL = zeros(1,3);  MWL = zeros(1,3);
  FHTR = zeros(1,3); MHTR = zeros(1,3);
  FHTL = zeros(1,3); MHTL = zeros(1,3);
  FVT = zeros(1,3);  MVT = zeros(1,3);
  FCR = zeros(1,3);  MCR = zeros(1,3);
  FCL = zeros(1,3);  MCL = zeros(1,3);
%
  lattice = Aero.lattice_vlm;
  body = Aero.body;
  FORCES  = Res.F;
%
  if isequal(stick.model.winr, 1)
    m = [];
    m(1) = pdcylin.stick.nwing_carryth;
    m(2) = pdcylin.stick.nwing_inboard;
    m(3) = pdcylin.stick.nwing_midboard;
    m(4) = pdcylin.stick.nwing_outboard;
    m(5) = pdcylin.stick.nwing_winglet;
    PSET = zeros(5,2);
%   CT
    PSET(1,1) = 1;
    PSET(1,2) = m(1)+1;
%   inb
    PSET(2,1) = PSET(1,2); 
    PSET(2,2) = sum(m(1:2))+1; 
%   midb
    PSET(3,1) = PSET(2,2); 
    PSET(3,2) = sum(m(1:3))+1; 
%   outb
    PSET(4,1) = PSET(3,2); 
    PSET(4,2) = sum(m(1:4))+1; 
%   winglet
    PSET(5,1) = PSET(4,2); 
    PSET(5,2) = sum(m(1:5))+1;
    AEROID = [stick.IDCAERO1.winr; stick.IDCAERO1.winrP];
    SETID = [stick.part.winr, stick.part.winrP];
    [loads.winr, FWR ,MWR] = component_aero_force(stick.nodes.winrC2_thick, AEROID, SETID, PSET, Aero, lattice, FORCES, toll);
    if isequal(stick.model.winl, 1)
      AEROID = [stick.IDCAERO1.winl; stick.IDCAERO1.winlP];
      [loads.winl, FWL, MWL] = component_aero_force(stick.nodes.winlC2_thick, AEROID, SETID, PSET, Aero, lattice, FORCES, toll);
    end
  end
  if isequal(stick.model.canr, 1)
    m = [];
    m(1) = pdcylin.stick.ncanard_carryth;
    m(2) = pdcylin.stick.ncanard_inboard;
    m(3) = pdcylin.stick.ncanard_outboard;
    PSET = zeros(3,2);
    PSET(1,1) = 1;
    PSET(1,2) = m(1)+1;
    if (m(1)==0)
      PSET(2,1) = 1; 
      PSET(2,2) = m(2)+1;
    else
      PSET(2,1) = PSET(1,2); 
      PSET(2,2) = m(1)+m(2)+1; 
    end
    PSET(3,1) = PSET(2,2);
    PSET(3,2) = sum(m)+1;
    [loads.canr, FCR, MCR] = component_aero_force(stick.nodes.canrC2_thick, stick.IDCAERO1.canr, stick.part.canr, PSET, Aero, lattice, FORCES, toll);
    if isequal(stick.model.canl, 1)
      [loads.canl, FCL, MCL] = component_aero_force(stick.nodes.canlC2_thick, stick.IDCAERO1.canl, stick.part.canr, PSET, Aero, lattice, FORCES, toll);
    end
  end
% Classic tail conf
  if isequal(stick.model.vert2, 0)
%
    if isequal(stick.model.horr, 1)
      m = [];
      m(1) = pdcylin.stick.nhtail_carryth;
      m(2) = pdcylin.stick.nhtail_inboard;
      m(3) = pdcylin.stick.nhtail_outboard;
      PSET = zeros(3,2);
      PSET(1,1) = 1;
      PSET(1,2) = m(1);
      PSET(2,1) = PSET(1,2); 
      PSET(2,2) = sum(m(1:2))+1; 
      PSET(3,1) = PSET(2,2);
      PSET(3,2) = sum(m)+1;
      [loads.horr, FHTR, MHTR] = component_aero_force(stick.nodes.horrC2_thick, stick.IDCAERO1.horr, stick.part.horr, PSET, Aero, lattice, FORCES, toll);
      if isequal(stick.model.horl, 1)
        [loads.horl, FHTL, MHTL] = component_aero_force(stick.nodes.horlC2_thick, stick.IDCAERO1.horl, stick.part.horr, PSET, Aero, lattice, FORCES, toll);
      end
    end
    HTFROOT = FHTR + FHTL;
    HTMROOT = MHTR + MHTL;
%
    node = 1;
    if isequal(stick.model.vert, 1)
      strcoord = stick.nodes.vert_thick; nodeht = 1; 
      m = [];
      m(1) = pdcylin.stick.nvtail_inboard;
      m(2) = pdcylin.stick.nvtail_outboard;
      PSET = zeros(2,2);
      PSET(1,1) = 1;
      PSET(1,2) = m(1)+1;
      PSET(2,1) = PSET(1,2); 
      PSET(2,2) = sum(m(1:2))+1; 
  %
      if isequal(stick.model.horr, 1)
        % find HT intersection
        htroot = stick.nodes.horrC2_thick(:,1); DIST = realmax; patch = 0;
        for i = length(stick.IDCAERO1.vert):-1:1
          n1 = PSET(stick.part.vert(i),1); n2 = PSET(stick.part.vert(i),2);
          for k=n1:n2
            dist = norm(strcoord(:,k) - htroot);
            if (dist < DIST) 
              DIST = dist; nodeht = k; patch = i; 
            end 
          end
        end
      end
      FS = zeros(geo.vtail.index(end),1); M =  zeros(geo.vtail.index(end),1); Mt = zeros(geo.vtail.index(end),1);
      FOUTPUT = [0 0 0]; MOUTPUT = [0 0 0];
      if (nodeht==1)
        [loads.vert, FVT, MVT] = component_aero_force(stick.nodes.vert_thick, stick.IDCAERO1.vert, stick.part.vert, PSET, Aero, lattice, FORCES, toll);
      else
        for i = length(stick.IDCAERO1.vert):-1:1
          if (i==patch)
            ID = stick.IDCAERO1.vert(i); index = find(Aero.ID==ID);
            n1 = PSET(stick.part.vert(i),1); n2 = PSET(stick.part.vert(i),2);
            nindex = find([n1:n2] == nodeht);
            DOF1 = lattice.DOF(index,1,1);
            DOF2 = lattice.DOF(index,1,2);
            DOF = [DOF1:DOF2];
            [FSY, MBY, MTY, FOUTPUT, MOUTPUT] = aer_str_data(DOF, n1, n2, strcoord, lattice, ...
                                                             toll, FORCES, ...
                                                             FOUTPUT, MOUTPUT, PLOT_TEST, nindex, HTFROOT, HTMROOT);
            FS(n1:n2) = FSY; M(n1:n2)  = MBY; Mt(n1:n2) = MTY;
          else
            ID = stick.IDCAERO1.vert(i); index = find(Aero.ID==ID);
            n1 = PSET(stick.part.vert(i),1); n2 = PSET(stick.part.vert(i),2);
            DOF1 = lattice.DOF(index,1,1);
            DOF2 = lattice.DOF(index,1,2);
            DOF = [DOF1:DOF2];
            [FSY, MBY, MTY, FOUTPUT, MOUTPUT] = aer_str_data(DOF, n1, n2, strcoord, lattice, ...
                                                             toll, FORCES, ...
                                                             FOUTPUT, MOUTPUT, PLOT_TEST, [], [], []); % initial offset for forces
            FS(n1:n2) = FSY; M(n1:n2)  = MBY; Mt(n1:n2) = MTY;
          end
        end
        FVT = FOUTPUT; MVT = MOUTPUT;
        loads.vert.FS = FS;
        loads.vert.M = M;
        loads.vert.Mt = Mt;
      end
    end
  else
% Twin tail connected to htail
% find connection
    if (isequal(stick.model.vert, 0) || isequal(stick.model.horr, 0))
      error('No vtail and/or htail found.')
    end
%
    vtnodes = stick.nodes.vert_thick; nodevt = 1; nodeht = 1;
    htnodes = stick.nodes.horrC2_thick; DIST = realmax; 
    for ih = length(stick.IDCAERO1.horr):-1:1
      nh1 = geo.htail.index(i); nh2 = geo.htail.index(i+1);
      for kh = nh1:nh2
        for iv = length(stick.IDCAERO1.vert):-1:1
          nv1 = geo.vtail.index(i); nv2 = geo.vtail.index(i+1);
          for kv=nv1:nv2
            dist = norm(vtnodes(:,kv) - htnodes(:,kh));
            if (dist < DIST) 
              DIST = dist; nodevt = kv; patchh = iv; nodeht = kh; patchv = ih;
            end 
          end % vt node loop
        end % vt patch loop
      end % ht node loop
    end % ht patch loop
    %
    if (nodevt==1)
      [loads.vert, FVTHTR, MVTHTR] = component_aero_force(0, geo.vtail.index, stick.nodes.vert_thick, stick.IDCAERO1.vert, Aero, lattice, FORCES, toll);
      [loads.vert2, FVTHTL, MVTHTL] = component_aero_force(0, geo.vtail.index, stick.nodes.vert2_thick, stick.IDCAERO1.vert2, Aero, lattice, FORCES, toll);
    else
      [FVTHTR, MVTHTR, loads.vert] =  vt2ht_loads(stick.nodes.vert_thick, lattice, toll, FORCES, PLOT_TEST, patchv, ...
                                      stick.IDCAERO1.vert, Aero, geo.vtail.index, nodevt);
      [FVTHTL, MVTHTL, loads.vert2] = vt2ht_loads(stick.nodes.vert2_thick, lattice, toll, FORCES, PLOT_TEST, patchv, ...
                                      stick.IDCAERO1.vert2, Aero, geo.vtail.index, nodevt);
      [FHTR, MHTR, loads.horr] = vt2ht_loads2(stick.nodes.horrC2_thick, lattice, toll, FORCES, PLOT_TEST, stick.IDCAERO1.horr, ...
                                Aero, geo.htail.index, patchh, nodeht, FVTHTR, FVTHTR);
      [FHTL, MHTL, loads.horl] = vt2ht_loads2(stick.nodes.horlC2_thick, lattice, toll, FORCES, PLOT_TEST, stick.IDCAERO1.horl, ...
                               Aero, geo.htail.index, patchh, nodeht, FVTHTL, FVTHTL);

    end
  end
%
% Fuselage loads
%
  if (isequal(stick.model.fuse, 1))
    FUSFWR = FWR + FWL; FUSMWR = MWR + MWL;
    FUSFCR = FCR + FCL; FUSMCR = MCR + MCL;
    if (isequal(stick.model.vert2, 0))
      FUSFVTR = FVT;      FUSMVTR = MVT;
      if (isequal(stick.model.horr, 1)) && (nodeht==1)
        FUSFHTR = FHTR + FHTL; FUSMHTR = MHTR + MHTL;
      else
        FUSFHTR = zeros(1,3); FUSMHTR = zeros(1,3); 
      end
    else
        FUSFHTR = FHTR + FHTL; FUSMHTR = MHTR + MHTL;
    end
    nn = length(stick.nodes.fuse_thick);
    loads.fus.FS = zeros(nn,1); loads.fus.M  = zeros(nn,1); loads.fus.Mt = zeros(nn,1);
%
%   interpolate body forces
%
    nbody = length(Aero.body.ID);
    if (nbody)
      BFORCES = Res.Fb;
      fusbody = find(Aero.body.ID == 1);
      if ~isempty(fusbody)
        [loads.fus.FS, loads.fus.M, loads.fus.Mt] = baer_str_data(fusbody, stick.nodes.fuse_thick',  [geo.fus.xx(3), 0, geo.fus.zz(3)], [geo.fus.xx(4), 0, geo.fus.zz(4)],body, toll, BFORCES, PLOT_TEST);
      end
    end
%
%   add to fuselage lifting surfaces root forces and moments
%
    if isequal(stick.model.canr, 1)
      loads.fus = add_surf2fuse(loads.fus, stick.nodes.canrC2_thick(:,1), stick.nodes.fuse_thick, FUSFCR,  FUSMCR);
    end
    if (isequal(stick.model.vert2, 0))
      if isequal(stick.model.vert, 1)
        loads.fus = add_surf2fuse(loads.fus, stick.nodes.vert_thick(:,1),   stick.nodes.fuse_thick, FUSFVTR, FUSMVTR);
      end
    end
    if isequal(stick.model.horr, 1)
      loads.fus = add_surf2fuse(loads.fus, stick.nodes.horrC2_thick(:,1), stick.nodes.fuse_thick, FUSFHTR, FUSMHTR);
    end
    if isequal(stick.model.winr, 1)
      loads.fus = add_surf2fuse(loads.fus, stick.nodes.winrC2_thick(:,1), stick.nodes.fuse_thick, FUSFWR,  FUSMWR);
    end
%
    loads.fus.Mt = cumsum(loads.fus.Mt);
    loads.fus.M  = cumsum(loads.fus.M);
    for i=1:length(loads.fus.M)
      loads.fus.M(i) =  loads.fus.M(i) - sum(loads.fus.FS(1:i) .* (stick.nodes.fuse_thick(1,1:i) - stick.nodes.fuse_thick(1,i))');
    end
    loads.fus.FS = cumsum(loads.fus.FS);
  end
%
end
%-------------------------------------------------------------------------------
function  [FS, MB, MT] = baer_str_data(nbody, str_data, ORIG, PY, body, toll, FORCES, PLOT_TEST)

    aero_data = body.lattice.Elem.Midpoint_loc{nbody};
    naer = size(aero_data,1);
%
    nstr = size(str_data,1);
    STRF = zeros(3 * nstr,1);
%
    Y = PY - ORIG; Y = Y ./ norm(Y);
    X = [0 -1 0];
    Z = (crossm(X) * Y')'; Z = Z ./ norm(Z);
    Rmat = [X; Y; Z];
    rotarm = zeros(nstr,3);
    for i=1:nstr
      rotarm(i,:) = (Rmat * (str_data(i,:)-ORIG)')';
    end
    H = beam_interface2(str_data, aero_data, toll, ORIG, Rmat);
    LOCF = FORCES{nbody};
    ROTF = zeros(naer, 3);
%   transform forces
    for k=1:naer
      ROTF(k,:) = (Rmat * LOCF(k,:)')';
    end
%   get only vertical component for transfer
    STRF = H' * [ROTF(:,3); zeros(2*naer,1)];
    FS = STRF(1:nstr);
    MB  = STRF(nstr+1:2*nstr);
    MT  = STRF(2*nstr+1:end);
%
%    fprintf(1, '\nAero Force: %g.', sum(ROTF(:,3)));
%    fprintf(1, '\nStr Force: %g.', sum(FS));
%
%
%   test interface
%
    if ( PLOT_TEST == 1)
      figure(1); hold on;
      plot3(aero_data(:,1), aero_data(:,2),aero_data(:,3),'ro')
      plot3(str_data(:,1), str_data(:,2), str_data(:,3),'ks')
      axis equal;
      title('Meshes'); ylabel('x [m]'); xlabel('y [m]');
%
      figure(2); hold on;
      plot(aero_data(:,2), ROTF(:,3),'-ro');
      plot(str_data(:,2),  STRF(1:nstr),'-ks');
      title('Forces'); ylabel('y [m]'); xlabel('F [N]');
%
      displ = str_data(:,2).^2;
      i = max(displ);
      displ = displ ./ i;
      displa = H * [displ; zeros(2*nstr,1)];
      figure(3); hold on;
      plot(aero_data(:,2), displa(1:naer),'ro');
      plot(str_data(:,2),  displ,'ks');
      title('Displacements'); ylabel('y [m]'); xlabel('dy [m]');
    end

end
%-------------------------------------------------------------------------------
function  [FSY, MBY, MTY, FOUTPUT, MOUTPUT] = aer_str_data(DOF, n1, n2, strcoord, lattice, toll, ...
                                                            FORCES, FYINPUT, MYINPUT, PLOT_TEST, NODEINT, FINT, MINT)

    naer = length(DOF);
    aero_data = zeros(naer, 3);
%
    if (n1<=n2)
      DOFs = [n1:n2];
    else
      DOFs = [n2:-1:n1];
    end
    nstr = length(DOFs);
    str_data = zeros(nstr, 3);
    STRF = zeros(3 * nstr,1);
    v1 = zeros(1,3); v2 = v1;
%   get aero coords
    for (j=1:naer)
      for k=1:3
        v1(1,k) = lattice.VORTEX(DOF(j),4,k);
        v2(1,k) = lattice.VORTEX(DOF(j),5,k);
      end
      aero_data(j, :) = 0.5.*(v1 + v2);
    end
%   get str coords
    for (j=1:nstr)
      for k=1:3
        str_data(j, k) = strcoord(k, DOFs(j));
      end
    end
    ORIG = str_data(1,:);
    Y = str_data(2,:) - ORIG; Y = Y ./ norm(Y);
    X = [1 0 0];
    Z = (crossm(X) * Y')'; Z = Z ./ norm(Z);
    X = (crossm(Y) * Z')'; X = X ./ norm(X);
    Rmat = [X; Y; Z];
    rotarm = zeros(nstr,3);
    for i=1:nstr
      rotarm(i,:) = (Rmat * (str_data(i,:)-ORIG)')';
    end
    H = beam_interface2(str_data, aero_data, toll, ORIG, Rmat);
    LOCF = FORCES(DOF,:);
    ROTF = zeros(naer, 3);
%   transform forces
    for k=1:naer
      ROTF(k,:) = (Rmat * LOCF(k,:)')';
    end
%   get only vertical component for transfer
    STRF = H' * [ROTF(:,3); zeros(2*naer,1)];
    FS = STRF(1:nstr);
    MB  = STRF(nstr+1:2*nstr);
    MT  = STRF(2*nstr+1:end);
%
%    fprintf(1, '\nAero Force: %g.', sum(ROTF(:,3)));
%    fprintf(1, '\nStr Force: %g.', sum(FS));
%
%
%   test interface
%
    if ( PLOT_TEST == 1)
      figure(1); hold on;
      plot3(aero_data(:,1), aero_data(:,2),aero_data(:,3),'ro')
      plot3(str_data(:,1), str_data(:,2), str_data(:,3),'ks')
      axis equal;
      title('Meshes'); ylabel('x [m]'); xlabel('y [m]');
%
      figure(2); hold on;
      plot(aero_data(:,2), ROTF(:,3),'-ro');
      plot(str_data(:,2),  STRF(1:nstr),'-ks');
      title('Forces'); ylabel('y [m]'); xlabel('F [N]');
%
      displ = str_data(:,2).^2;
      i = max(displ);
      displ = displ ./ i;
      displa = H * [displ; zeros(2*nstr,1)];
      figure(3); hold on;
      plot(aero_data(:,2), displa(1:naer),'ro');
      plot(str_data(:,2),  displ,'ks');
      title('Displacements'); ylabel('y [m]'); xlabel('dy [m]');
    end
%   shear
    offset = Rmat * FYINPUT';
    FS(end) = FS(end) + offset(3); 
% 
    if (~isempty(NODEINT))
      offset = Rmat * FINT';
      FS(NODEINT) = FS(NODEINT) + offset(3);
    end
    for i=1:nstr
      FSY(i) = sum(FS(end:-1:i));
    end    
%   bending moment
    offset = Rmat * MYINPUT';
    MB(end) = MB(end) + offset(1);
    if (~isempty(NODEINT))
      offset2 = Rmat * MINT';
      MB(NODEINT) = MB(NODEINT) + offset2(1);
      MT(NODEINT) = MT(NODEINT) + offset2(2);
    end
    for i=nstr:-1:1
      MBY(i) = sum(MB(end:-1:i)) + sum(FS(end:-1:i).*(rotarm(end:-1:i,2) - rotarm(i,2)));
    end
%   torque moment
    MT(end) = MT(end) + offset(2);
    for i=1:nstr
      MTY(i) = sum(MT(end:-1:i));
    end    
%   export offset
    FOUTPUT = ((Rmat') * [0 0 FSY(1)]')';
    MOUTPUT = ((Rmat') * [MBY(1) MTY(1) 0]')';
%
end
%-------------------------------------------------------------------------------
function [output, FOUTPUT, MOUTPUT] = component_aero_force(strcoord, IDCAERO, SETID, SDOF, Aero, lattice, FORCES, toll)
%
  PLOT_TEST = 0;
  nstr = size(strcoord,2);
  naer = length(IDCAERO);
%
  FS = zeros(nstr,1); M =  zeros(nstr,1); Mt = zeros(nstr,1);
  FOUTPUT = [0 0 0]; MOUTPUT = [0 0 0];
%
  [ID, ind] = sort(IDCAERO, 'descend'); % label have growing ID -> outboard
  SETID = SETID(ind);
  for i = 1:naer
    ipatch = find(Aero.ID==ID(i));
    DOF1 = lattice.DOF(ipatch,1,1); DOF2 = lattice.DOF(ipatch,1,2); DOF = [DOF1:DOF2];
    n1 = SDOF(SETID(i),1); n2 = SDOF(SETID(i),2);
    [FSY, MBY, MTY, FOUTPUT, MOUTPUT] = aer_str_data(DOF, n1, n2, strcoord, lattice, ...
                                                     toll, FORCES, ...
                                                     FOUTPUT, MOUTPUT, PLOT_TEST, [], [], []); % initial offset for forces
    FS(n1:n2) = FSY; M(n1:n2)  = MBY; Mt(n1:n2) = MTY; 
  end
%
  output.FS = FS; output.M = M; output.Mt = Mt;
end
%-------------------------------------------------------------------------------
function loads = add_surf2fuse(loads, root, nodes, FR, MR)
  nn = length(nodes);
  dist = zeros(nn,1);
  for k=1:nn
    dist(k) = norm((nodes(:,k) - root));
  end
  [v, i] = min(dist);
  loads.FS(i) = loads.FS(i) + FR(3);
  mb = -(crossm(root - nodes(:,i)) * FR')'; % transport moment
%
  loads.M(i)  = loads.M(i)  + MR(2) + mb(2);
  loads.Mt(i) = loads.Mt(i) + MR(1) + mb(1);
%
end
%-------------------------------------------------------------------------------
function [FVTHT, MVTHT, loads] = vt2ht_loads(strcoord, lattice, toll, FORCES, PLOT_TEST, patchv, IDCAERO, Aero, INDEX, nodevt)
%
  FS = zeros(INDEX(end),1); M =  FS; Mt = FS;
  i = setdiff([1,2], patchv);
  ID = IDCAERO(i); index = find(Aero.ID==ID);
  n1 = geo.vtail.index(i); n2 = geo.vtail.index(i+1);
  [FSY, MBY, MTY, FOUTPUT, MOUTPUT] = aer_str_data(index, n1, n2, strcoord, lattice, ...
                                                   toll, FORCES, ...
                                                   [0 0 0], [0 0 0], PLOT_TEST, [], [], []); % initial offset for forces
  FS(n1:n2) = FSY; M(n1:n2)  = MBY; Mt(n1:n2) = MTY;
%
  i = patchv;
  ID = IDCAERO(i); index = find(Aero.ID==ID);
  n1 = INDEX(i); n2 = INDEX(i+1);
  nlabel = [n1:n2]; nindex = find(nlabel==nodevt);
  nsec = [nlabel(nindex):-1:nlabel(1)];
  [FSY, MBY, MTY, FOUTPUT1, MOUTPUT1] = aer_str_data(index, nsec(1), nsec(end), strcoord, lattice, ...
                                                   toll, FORCES, ...
                                                   [0 0 0], [0 0 0], PLOT_TEST, [], [], []);
  FS(nsec) = FSY; M(nsec)  = MBY; Mt(nsec) = MTY;
  fsroot(1) = FSY(1); mbroot(1) = MBY(1); mtroot(1) = MTY(1);
  FOUTPUT = [0 0 0]; MOUTPUT = [0 0 0];
  nsec = [nlabel(nindex):nlabel(end)];
  [FSY, MBY, MTY, FOUTPUT2, MOUTPUT2] = aer_str_data(index, nsec(1), nsec(end), strcoord, lattice, ...
                                                   toll, FORCES, ...
                                                   FOUTPUT, MOUTPUT, PLOT_TEST, [], [], []);

  FS(nsec) = FSY; M(nsec)  = MBY; Mt(nsec) = MTY;
  fsroot(2) = FSY(1); mbroot(2) = MBY(1); mtroot(2) = MTY(1);
  [v,i] = max(abs(fsroot)); FS(nodevt) = fsroot(i);
  [v,i] = max(abs(mbroot)); M(nodevt)  = mbroot(i);
  [v,i] = max(abs(mtroot)); Mt(nodevt) = mtroot(i);
       %
  FVTHT = FOUTPUT1 + FOUTPUT2;
  MVTHT = MOUTPUT1 + MOUTPUT2;
%        
  loads.FS = FS;
  loads.M = M;
  loads.Mt = Mt;

end
%-------------------------------------------------------------------------------
function [FVT, MVT, loads] = vt2ht_loads2(strcoord, lattice, toll, FORCES, PLOT_TEST, IDCAERO, Aero, INDEX, nodeht, FVTHTR, MVTHTR)
  % introduce vtail forces to htail
  FS = zeros(INDEX(end),1); M =  FS; Mt = FS;
  FOUTPUT = [0 0 0]; MOUTPUT = [0 0 0];
  for i = length(IDCAERO):-1:1
    if (i==patchh)
      ID = IDCAERO(i); index = find(Aero.ID==ID);
      n1 = INDEX(i); n2 = INDEX(i+1);
      nindex = find([n1:n2] == nodeht);
      [FSY, MBY, MTY, FOUTPUT, MOUTPUT] = aer_str_data(index, n1, n2, strcoord, lattice, ...
                                                       toll, FORCES, ...
                                                       FOUTPUT, MOUTPUT, PLOT_TEST, nindex, FVTHTR, MVTHTR);
      FS(n1:n2) = FSY; M(n1:n2)  = MBY; Mt(n1:n2) = MTY;
    else
      ID = IDCAERO(i); index = find(Aero.ID==ID);
      n1 = geo.htail.index(i); n2 = geo.htail.index(i+1);
      [FSY, MBY, MTY, FOUTPUT, MOUTPUT] = aer_str_data(index, n1, n2, strcoord, lattice, ...
                                                       toll, FORCES, ...
                                                       FOUTPUT, MOUTPUT, PLOT_TEST, [], [], []); % initial offset for forces
      FS(n1:n2) = FSY; M(n1:n2)  = MBY; Mt(n1:n2) = MTY;
    end
  end
  FVT = FOUTPUT; MVT = MOUTPUT;
  loads.FS = FS;
  loads.M = M;
  loads.Mt = Mt;
end