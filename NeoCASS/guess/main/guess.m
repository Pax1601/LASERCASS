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
%     080502      1.0     A.Da Ronch       Creation
%     090326      1.3.2   A. De Gaspari    Modification
%     090824      1.3.7   A. De Gaspari    Modification
%     101005      1.5     L. Travaglini    Modification
%     021211      2.0     L. Cavagna       loop along maneuvers with internal loop
%                                          along inertial confs.
%
%
%*******************************************************************************
%
% function guess
%
%   DESCRIPTION: Run GUESS Module from structural sizing to stick model creation
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                filename_geo   string     geometry XML file
%                filename_st    string     flight states XML file
%                filename_tech  string     technology XML file
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                filename_stick string     stick model for SMARTCAD
%    REFERENCES:
%
%*******************************************************************************
function guess_model = guess(filename_geo, filename_tech, filename_stick, filename_trim, model, varargin)
%
global beam_model NMAX EPS
fid = 1;
%
guess_model.GUI.model = model;
%
if isequal(model.guessstd,1)
  GUESS_STD = 1;
else
  GUESS_STD = 0;
end
% force export pipe model only
if isempty(filename_trim)
  GUESS_STD = 0;
end
%
TOLL = realmax;
%
guess_model             = [];
guess_model.aircraft    = [];
guess_model.states      = [];
guess_model.pdcylin     = [];
guess_model.stick_model = [];
guess_model.str_prop    = [];
guess_model.optim       = [];
guess_model.Niter = 0;
optim = [];
% Define default stick model filename
if isempty(filename_stick)
    filename_stick = 'guess_stick_model.dat';
end
indE = find(filename_stick=='.');
indE = indE(end);
filename_tube = [filename_stick(1:indE-1),'_tube.dat'];
%
% check if gues is running in batch
if (nargin == 6)
    inBatch = varargin{1};
else
    inBatch = true;
end
    
    
fprintf(fid, '\nRunning GUESS solver...');
[aircraft, pdcylin, go, geostd, stickstd] =  ControlGEO(filename_geo,filename_tech, filename_trim, model, inBatch); % NMAX and EPS definition
%
if go == 0
    return
end
%
%-------------------------------------------------------------------------------
% LOAD INPUT FILES
%-------------------------------------------------------------------------------
%
% geometry data
fprintf(fid, '\n- Loading input files...');
fprintf(fid, '\n\tGeometry XML file: %s.', filename_geo);
fprintf(fid, '\ndone.');
%
if aircraft.Canard.present
    pdcylin.stick.model.canr = 1;
end
if ~aircraft.Horizontal_tail.present
    pdcylin.stick.model.horr = 0; 
end
if isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present
    pdcylin.stick.model.tboomsr = 1;
else
    pdcylin.stick.model.tboomsr = 0;
end
pdcylin.guess.check = 1;
%
Nconf = length(pdcylin.MassConf.Pass);
%
%-------------------------------------------------------------------------------
% GUESS STD: ANALYTICAL WING AND FUSELAGE WEIGHT ESTIMATION
%-------------------------------------------------------------------------------
if (isequal(GUESS_STD,1))
%
  if isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present
    guess_model = [];
    fprintf(fid, '\n### Warning: tailbooms not supported in GUESS Rigid.');
    fprintf(fid, '\n    Use the GUESS Elastic instead.');
    return;
  end
  if  isfield(aircraft.Vertical_tail,'Twin_tail') && isequal(aircraft.Vertical_tail.Twin_tail, 1)
    guess_model = [];
    fprintf(fid, '\n### Warning: twin tails not supported in GUESS Rigid.');
    fprintf(fid, '\n    Use the GUESS Elastic instead.');
    return;
  end
  STRW = [];
  i = 1;
  fprintf(fid,'\n\t-------------------------------------------- GEOMETRY ----------------------------------------------');
  fprintf(fid, '\n\tCreating baseline geometry...');
  beam_model = load_nastran_model('gstd_model.dat');
  
  if  isfield(pdcylin,'deformation') && isfield(pdcylin.deformation,'bending')
        ss = size(beam_model.Node.DOF);
        
        beam_model.Node.DOF = reshape([1:ss(1)*ss(2)]',ss(2),ss(1))';
        beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
        
        aeroelastic_interface
        beam_model = beam_model;
        beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
        
        % bending
        SOL = zeros(max(max(beam_model.Node.DOF)),1);
        
        
        if  isfield(pdcylin.deformation,'torsion')
           % torsion
           for i = 1:length (beam_model.Node.ID)
           
            if (beam_model.Node.ID(i)>=2000) && (beam_model.Node.ID(i)<3000)
                for k = 1 : length(pdcylin.deformation.bending.coefficients)
                    SOL(beam_model.Node.DOF(i,5)) = SOL(beam_model.Node.DOF(i,5)) + ...
                        (pdcylin.deformation.torsion.coefficients(k)* abs(beam_model.Node.Coord(i,2)).^k)./pdcylin.deformation.torsion.max_tip_value;
                end
                
                
            end
            
           end
           
           beam_model.Aero.lattice_vlm = update_vlm_mesh1(beam_model.Node, SOL, beam_model.Aero);
           
        end
        
        
        
        for i = 1:length (beam_model.Node.ID)
           
            if (beam_model.Node.ID(i)>=2000) && (beam_model.Node.ID(i)<3000)
               
                for k = 1 : length(pdcylin.deformation.bending.coefficients)
                    beam_model.Node.Coord(i,3) = beam_model.Node.Coord(i,3) + ...
                        (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Node.Coord(i,2)).^k)./pdcylin.deformation.bending.max_tip_value;
                end
                
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
                         (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.COLLOC(d0:d1,2)).^k)./pdcylin.deformation.bending.max_tip_value;
                    beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) = beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,2)).^k)./pdcylin.deformation.bending.max_tip_value;
                    beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) = beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,2)).^k)./pdcylin.deformation.bending.max_tip_value;
                end
            end
            
        end
        
        for j =1 : length(beam_model.Aero.lattice_vlm.Control.Patch)
           locdof =  beam_model.Aero.lattice_vlm.Control.DOF(j).data;
           ind = intersect(locdof,dof);
           if length(ind)>0
               for k = 1 : length(pdcylin.deformation.bending.coefficients)
                    beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) = beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.Control.Hinge(j,1,2)).^k)./pdcylin.deformation.bending.max_tip_value;
                    beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) = beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(beam_model.Aero.lattice_vlm.Control.Hinge(j,2,2)).^k)./pdcylin.deformation.bending.max_tip_value;
                end
           end
        end
        
        
        
  end
  i = 1;
  MRP = beam_model.WB.MRP;
  if (isempty(beam_model.Aero.Trim.Man_index))
    nman = beam_model.Info.ntrim;
  else
    nman = length(unique(beam_model.Aero.Trim.Man_index));
  end
  for k=1:nman
    Res{k} = {};
  end
  %
  % Start iterative loop until convergence on structural weight
  str.fus = []; str.wing = []; str.vtail = []; str.htail = []; str.wing.regr.M = 0; str.wing.regr.CG = 0.0;

  while( (i <= NMAX) && (TOLL > EPS))
    guess_model.Niter = guess_model.Niter+1;
    fprintf(fid,'\n\n------------------------------------------------ GUESS IT %d -----------------------------------------------\n', i);
    [pdcylin, loads, str, aircraft, Res, optim] = AFaWWE_std(fid, pdcylin, aircraft, geostd, str, stickstd, filename_tech, i, ...
                                                             Res, optim, beam_model.Aero.Trim.Man_index);
    fprintf(fid, '\ndone.');
    % update structural weight
    aircraft.weight_balance.COG(1,4,1) = 2*sum(str.wing.WTBOX) + str.wing.WTC;
    aircraft.weight_balance.COG(1,1,1) = str.wing.CG;
    if isequal(pdcylin.stick.model.canr, 1)
        aircraft.weight_balance.COG(11,4,1) = 2*sum(str.canard.WTBOX) + str.canard.WTC;
        aircraft.weight_balance.COG(11,1,1) = str.canard.CG;
    end
    if isequal(pdcylin.stick.model.horr, 1)
        aircraft.weight_balance.COG(3,4,1) = 2*sum(str.htail.WTBOX) + str.htail.WTC;
        aircraft.weight_balance.COG(3,1,1) = str.htail.CG;
    end
    if isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present
        aircraft.weight_balance.COG(12,4,1) = 2*sum(str.tbooms.WTOT);
        aircraft.weight_balance.COG(12,1,1) = str.tbooms.CG;
    end
%
    if isequal(aircraft.Vertical_tail.present, 1)
        aircraft.weight_balance.COG(4,4,1) = sum(str.vtail.WTBOX) + str.vtail.WTC;
        aircraft.weight_balance.COG(4,1,1) = str.vtail.CG;
    end
%
    aircraft.weight_balance.COG(5,4,1) = sum(str.fus.WTOT);
    aircraft.weight_balance.COG(5,1,1) = str.fus.CG;
 %
    aircraft.weight_balance.COG(27,1,1) = aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,1,1)'*...
    aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)/...
    (sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)));

    TOTW = sum(aircraft.weight_balance.COG([1,2,3,4,5,10,11,12],4,1));
%   update WB struct
    beam_model = update_mass_std(aircraft, beam_model, [aircraft.weight_balance.COG(27,1,1), 0, 0], ...
                   sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)));
    %
    if (i>1)
      TOLL = abs(STRW(i-1,2) - TOTW) / STRW(1,2);
    end
    STRW = [STRW; i-1, TOTW, TOLL];
    i = i+1;
    %
  end
%
%-------------------------------------------------------------------------------
% GEOMETRIC STICK MODEL GENERATION
%-------------------------------------------------------------------------------
%
%  fprintf(fid,'\n\t-------------------------------------------- STICK MODEL -------------------------------------------');
%  fprintf(fid, '\n\t- Creating baseline stick model...');
%  [stick, geo] = Stick_Model(pdcylin, aircraft, geostd);
  geo   = geostd;
  stick = stickstd;
  fprintf(fid, 'done.');
  %
%-------------------------------------------------------------------------------
% SECTION MECHANICAL PROPERTIES
%-------------------------------------------------------------------------------
%
  fprintf(fid, '\n\t- Estimating mechanical properties...');
  [str] = Prop_Sec(fid, pdcylin, stick, geo, str,aircraft);
  fprintf(fid, '\n\tdone.');
%
%-------------------------------------------------------------------------------
% NON STRUCTURAL MASSES DETERMINATION
%-------------------------------------------------------------------------------
%
  fprintf(fid, '\n\t- Including non-structural masses...');
  [str] = Add_NSM(fid, pdcylin, aircraft, geo, stick, str);
  fprintf(fid, '\n\tdone.');
  guess_model.AeroRes = Res;
%    
else
%
%-------------------------------------------------------------------------------
%  GUESS MOD: SMARTCAD COUPLING
%-------------------------------------------------------------------------------
%
  TOLL = realmax;
  STRW = [];
  ii = 1;
% Start iterative loop until convergence on structural weight
  while( (ii <= NMAX) && (TOLL > EPS))
%   The first analysis needs the definition of stick model that will be
%   using the structural masses passed by weight and balance
    if ii == 1
      [pdcylin, geo, str,aircraft] = AFaWWE_SET(fid, pdcylin, aircraft);
      str.wing.regr.M = 0;
      str.wing.regr.CG = 0.0;
%     create stick model
      fprintf(fid,'\n\t-------------------------------------------- STICK MODEL -------------------------------------------');
      fprintf(fid, '\n- Creating baseline stick model...');
      [stick, geo] = Stick_Model(pdcylin, aircraft, geo);
      fprintf(fid, 'done.');
      Nanaysis = 0;
      for n = 1 : Nconf
          LoadNconf = pdcylin.MassConf.Load(n).data';
          Nanaysis =  Nanaysis + length(LoadNconf);
      end
      ContTrim = 1;
%
      NTOTMAN = 0;
      for n = 1 : Nconf
        LoadNconf = pdcylin.MassConf.Load(n).data';
        mloc = max(LoadNconf);
        if (mloc>NTOTMAN) 
          NTOTMAN = mloc;
        end
      end
%     export files 
      for n = 1 : Nconf
          [file_name_conf, WREG, WREGCG] = export_tube_smartcad(stick, geo, aircraft, [],...
              pdcylin, str, filename_tube, filename_trim, n);
       end
%     break
      if isempty(filename_trim)
        fprintf('\n\nGuess process ended. No trim file provided. Tubular model and 6DOF gstd_model.dat available only.\n\n');
        return
      end
%
%     CONF LOOP
%
      BEAM_MODEL_LOADED = 0;
      INTERP_RUN = 0;
      for NMAN = 1:NTOTMAN
%
        MAN_RUN = 0;
        fprintf(fid, '\n\t- Running maneuver %d...', NMAN);
        for n = 1 : Nconf
          LoadNconf = pdcylin.MassConf.Load(n).data;
          index = find(LoadNconf == NMAN);
          if (~isempty(index))
            fprintf(fid, '\n\t\t- Configuration %d...', n);
            [file_name_conf, WREG, WREGCG] = export_model_smartcad(stick, geo, aircraft, [],...
                pdcylin, str, 'smartcad_loads.dat', filename_trim, n, ii);
%               copy wing regression mass and CG
            str.wing.regr.M = WREG;
            str.wing.regr.CG = WREGCG;
%
            if (~BEAM_MODEL_LOADED)
              beam_model = load_nastran_model('smartcad_loads.dat');
              BEAM_MODEL_LOADED = 1;
              NbarInfo = beam_model.Info.nbar;
              NZ = zeros(NTOTMAN,Nconf);
              VEL = zeros(NTOTMAN,1);
              for kk=1:NTOTMAN
                VEL(kk) = mach_alt2cas(beam_model.Aero.Trim.Mach(kk), beam_model.Aero.Trim.ALT(kk));
                VEL(kk) = cas2eas(VEL(kk),  beam_model.Aero.Trim.ALT(kk));
              end
              internal_loads = zeros(NbarInfo,12,NTOTMAN*Nconf);
            end
            Conm = load_nastran_model(file_name_conf);
            delete(file_name_conf);
            beam_model.Param.LANDG = Conm.Param.LANDG; 
            beam_model.ConM = Conm.ConM;
            beam_model.Info.nconm = Conm.Info.nconm;
            [beam_model.WB.CG, beam_model.WB.MCG, beam_model.WB.MRP] = ...
                wb_set_conm_mass(beam_model.Info.nconm, beam_model.Node.Index, beam_model.Node.Coord, ...
                beam_model.Node.R, beam_model.Param.GRDPNT, beam_model.ConM);
            % set bar mass CG
            [beam_model.WB.CG, beam_model.WB.MCG, beam_model.WB.MRP] =...
                wb_add_bar_mass(beam_model.Info.nbar, beam_model.Node.Coord, beam_model.Node.R, ...
                beam_model.WB.CG, beam_model.Param.GRDPNT, beam_model.WB.MCG, ...
                beam_model.WB.MRP, beam_model.Bar);
            strNode = unique([beam_model.Bar.Conn(:,1);beam_model.Bar.Conn(:,3)]);
            [dummyCG,idSuport] = min((beam_model.WB.CG(1,1)-beam_model.Node.Coord(strNode,1)).^2+...
                                     (beam_model.WB.CG(1,2)-beam_model.Node.Coord(strNode,2)).^2+...
                                     (beam_model.WB.CG(1,3)-beam_model.Node.Coord(strNode,3)).^2);
            beam_model.Param.SUPORT = [beam_model.Node.ID(strNode(idSuport)),123456];

            if (~MAN_RUN)

              if (~INTERP_RUN)
                solve_free_lin_trim_guess(NMAN, INTERP_RUN);
                Interp = beam_model.Aero.Interp;
                INTERP_RUN = 1;
              else
                beam_model.Aero.Interp = Interp;
                solve_free_lin_trim_guess(NMAN, INTERP_RUN);
              end
              MAN_RUN = 1;
              pos = (n-1)*NTOTMAN+NMAN;
              internal_loads(:,:,pos) = sma2guess_internal_load(beam_model);
              NZ(NMAN,n) = beam_model.Res.FM.Value(9)/9.81;
              ContTrim = ContTrim+1;
              KAX = beam_model.Res.Aero.Kax;
              F = beam_model.Res.Aero.F;
              KAH = beam_model.Res.Aero.Kah;
              H0 = beam_model.Res.Aero.H0;
              BK.KAX{NMAN} = KAX;
              BK.F{NMAN} = F;
              BK.KAH{NMAN} = KAH;
              BK.H0{NMAN} = H0;
            else
              solve_free_lin_trim_guess2(NMAN,KAX,F,KAH,H0)
              pos = (n-1)*NTOTMAN+NMAN;
              internal_loads(:,:,pos) = sma2guess_internal_load(beam_model);
              NZ(NMAN,n) = beam_model.Res.FM.Value(9)/9.81;
              ContTrim = ContTrim+1;
            end
          end % if index
        end % conf loop
%
      end % maneuver loop
      % Critical condition
      delete('smartcad_loads.dat');
      %
      [gloads]=gmod_loads(internal_loads, stick, beam_model.Bar,beam_model.Beam, NTOTMAN);
      guess_model.Niter = guess_model.Niter +1;
      fprintf(fid,'\n\n------------------------------------------------ GUESS IT %d -----------------------------------------------\n', guess_model.Niter);
      [pdcylin, geo, loads, str,aircraft,optim] = AFaWWE_mod(fid, guess_model.Niter, pdcylin, aircraft, ...
                                                  filename_tech, gloads, NZ, VEL, stick, optim, geo, NTOTMAN, Nconf);
       str.wing.regr.M = WREG;
       str.wing.regr.CG = WREGCG;
%
      fprintf(fid, '\n\tdone.');
      % update structural weight
      aircraft.weight_balance.COG(1,4,1) = 2*sum(str.wing.WTBOX) + str.wing.WTC;
      aircraft.weight_balance.COG(1,1,1) = (str.wing.CG*(2*sum(str.wing.WBOX) + str.wing.WC) + WREG*WREGCG)/aircraft.weight_balance.COG(1,4,1);
      if isequal(pdcylin.stick.model.canr, 1)
          aircraft.weight_balance.COG(11,4,1) = 2*sum(str.canard.WTBOX) + str.canard.WTC;
          aircraft.weight_balance.COG(11,1,1) = str.canard.CG;
      end
      if isequal(pdcylin.stick.model.horr, 1)
          aircraft.weight_balance.COG(3,4,1) = 2*sum(str.htail.WTBOX) + str.htail.WTC;
          aircraft.weight_balance.COG(3,1,1) = str.htail.CG;
      end
      if isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present
          aircraft.weight_balance.COG(12,4,1) = 2*sum(str.tbooms.WTOT);
          aircraft.weight_balance.COG(12,1,1) = str.tbooms.CG;
      end
      %
      if isequal(pdcylin.stick.model.vert, 1)
          aircraft.weight_balance.COG(4,4,1) = sum(str.vtail.WTBOX) + str.vtail.WTC;
          aircraft.weight_balance.COG(4,1,1) = str.vtail.CG;
          if (isequal(aircraft.Vertical_tail.Twin_tail, 1))
            aircraft.weight_balance.COG(10,4,1) = aircraft.weight_balance.COG(4,4,1);
            aircraft.weight_balance.COG(10,1,1) = str.vtail.CG;
            aircraft.weight_balance.COG(10,2,1) = -aircraft.weight_balance.COG(4,2,1);
          end
      end
      %
      aircraft.weight_balance.COG(5,4,1) = sum(str.fus.WTOT);
      aircraft.weight_balance.COG(5,1,1) = str.fus.CG;

      aircraft.weight_balance.COG(27,1,1) = aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,1,1)'*...
          aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)/...
          (sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)));

      TOTW = sum(aircraft.weight_balance.COG([1,2,3,4,5,10,11,12],4,1));
      if (ii>1)
          TOLL = abs(STRW(ii-1,2) - TOTW) / STRW(1,2);
      end
      STRW = [STRW; ii-1, TOTW, TOLL];
      ii = ii+1;
%
      fprintf(fid, '\n- Creating baseline stick model...');
      [stick, geo] = Stick_Model(pdcylin, aircraft, geo);
      fprintf(fid, 'done.');
%
%-------------------------------------------------------------------------------
%     SECTION MECHANICAL PROPERTIES
%-------------------------------------------------------------------------------
%
      fprintf(fid, '\n- Estimating mechanical properties...');
      [str] = Prop_Sec(fid, pdcylin, stick, geo, str,aircraft);
      fprintf(fid, 'done.');
%
%-------------------------------------------------------------------------------
% NON STRUCTURAL MASSES DETERMINATION
%-------------------------------------------------------------------------------
%
      fprintf(fid, '\n\t- Including non-structural masses...');
      [str] = Add_NSM(fid, pdcylin, aircraft, geo, stick, str);
      fprintf(fid, '\ndone.');
%
    else % if ii>1
%
      NZ = zeros(NTOTMAN,Nconf);
      internal_loads = zeros(NbarInfo,12,NTOTMAN*Nconf);
      ContTrim = 1;
      BEAM_MODEL_LOADED = 0;
      for NMAN = 1:NTOTMAN
%
        MAN_RUN = 0;
        fprintf(fid, '\n\t- Running maneuver %d...', NMAN);
        for n = 1 : Nconf
          LoadNconf = pdcylin.MassConf.Load(n).data;
          index = find(LoadNconf == NMAN);
          if (~isempty(index))
            fprintf(fid, '\n\t\t- Configuration %d...', n);
            [file_name_conf, WREG, WREGCG] = export_model_smartcad(stick, geo, aircraft, beam_model.Aero.ref,...
                pdcylin, str, 'smartcad_loads.dat', filename_trim, n, ii);
%               copy wing regression mass and CG
            str.wing.regr.M = WREG;
            str.wing.regr.CG = WREGCG;
%
            if (~BEAM_MODEL_LOADED)
              beam_model2 = load_nastran_model('smartcad_loads.dat');
              beam_model.PBar = beam_model2.PBar;
              beam_model.Bar = beam_model2.Bar;  
              beam_model.Res = rmfield(beam_model.Res,'Structure');
%
              BEAM_MODEL_LOADED = 1;
              NbarInfo = beam_model.Info.nbar;
              NZ = zeros(NTOTMAN,Nconf);
              internal_loads = zeros(NbarInfo,12,NTOTMAN*Nconf);
            end
            Conm = load_nastran_model(file_name_conf);
            delete(file_name_conf);
            beam_model.Param.LANDG = Conm.Param.LANDG;
            beam_model.ConM = Conm.ConM;
            beam_model.Info.nconm = Conm.Info.nconm;
            [beam_model.WB.CG, beam_model.WB.MCG, beam_model.WB.MRP] = ...
                wb_set_conm_mass(beam_model.Info.nconm, beam_model.Node.Index, beam_model.Node.Coord, ...
                beam_model.Node.R, beam_model.Param.GRDPNT, beam_model.ConM);
%          set bar mass CG
            [beam_model.WB.CG, beam_model.WB.MCG, beam_model.WB.MRP] =...
                wb_add_bar_mass(beam_model.Info.nbar, beam_model.Node.Coord, beam_model.Node.R, ...
                beam_model.WB.CG, beam_model.Param.GRDPNT, beam_model.WB.MCG, ...
                beam_model.WB.MRP, beam_model.Bar);
            strNode = unique([beam_model.Bar.Conn(:,1);beam_model.Bar.Conn(:,3)]);
            [dummyCG,idSuport] = min((beam_model.WB.CG(1,1)-beam_model.Node.Coord(strNode,1)).^2+...
                                     (beam_model.WB.CG(1,2)-beam_model.Node.Coord(strNode,2)).^2+...
                                     (beam_model.WB.CG(1,3)-beam_model.Node.Coord(strNode,3)).^2);
            beam_model.Param.SUPORT = [beam_model.Node.ID(strNode(idSuport)),123456];
%
            if (~MAN_RUN)

              beam_model.Aero.Interp = Interp;
              solve_free_lin_trim_guess2(NMAN,BK.KAX{NMAN},BK.F{NMAN},BK.KAH{NMAN},BK.H0{NMAN});
              MAN_RUN = 1;
              pos = (n-1)*NTOTMAN+NMAN;
              internal_loads(:,:,pos) = sma2guess_internal_load(beam_model);
              NZ(NMAN,n) = beam_model.Res.FM.Value(9)/9.81;
              ContTrim = ContTrim+1;
            else
              solve_free_lin_trim_guess2(NMAN,BK.KAX{NMAN},BK.F{NMAN},BK.KAH{NMAN},BK.H0{NMAN});
              pos = (n-1)*NTOTMAN+NMAN;
              internal_loads(:,:,pos) = sma2guess_internal_load(beam_model);
              NZ(NMAN,n) = beam_model.Res.FM.Value(9)/9.81;
              ContTrim = ContTrim+1;
            end
          end % if index
        end % conf loop
%
      end % maneuver loop
%
      delete('smartcad_loads.dat');
%
      [gloads]=gmod_loads(internal_loads, stick, beam_model.Bar,beam_model.Beam, NTOTMAN);
      guess_model.Niter = guess_model.Niter +1;
      fprintf(fid,'\n\n------------------------------------------------ GUESS IT %d -----------------------------------------------\n', guess_model.Niter);
      [pdcylin, geo, loads, str, aircraft, optim] = AFaWWE_mod(fid, guess_model.Niter, pdcylin, aircraft, ...
                                                     filename_tech, gloads, NZ, VEL, stick, optim, geo, NTOTMAN, Nconf);
      str.wing.regr.M = WREG;
      str.wing.regr.CG = WREGCG;
%
      fprintf(fid, '\n\tdone.');
      % update structural weight
      aircraft.weight_balance.COG(1,4,1) = 2*sum(str.wing.WTBOX) + str.wing.WTC;
      aircraft.weight_balance.COG(1,1,1) = (str.wing.CG*(2*sum(str.wing.WBOX) + str.wing.WC) + WREG*WREGCG)/aircraft.weight_balance.COG(1,4,1);
      if isequal(pdcylin.stick.model.canr, 1)
          aircraft.weight_balance.COG(11,4,1) = 2*sum(str.canard.WTBOX) + str.canard.WTC;
          aircraft.weight_balance.COG(11,1,1) = str.canard.CG;
      end
      if isequal(pdcylin.stick.model.horr, 1)
          aircraft.weight_balance.COG(3,4,1) = 2*sum(str.htail.WTBOX) + str.htail.WTC;
          aircraft.weight_balance.COG(3,1,1) = str.htail.CG;
      end
      if isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present
          aircraft.weight_balance.COG(12,4,1) = 2*sum(str.tbooms.WTOT);
          aircraft.weight_balance.COG(12,1,1) = str.tbooms.CG;
      end
      %
      if isequal(pdcylin.stick.model.vert, 1)
          aircraft.weight_balance.COG(4,4,1) = sum(str.vtail.WTBOX) + str.vtail.WTC;
          aircraft.weight_balance.COG(4,1,1) = str.vtail.CG;
          if (isequal(aircraft.Vertical_tail.Twin_tail, 1))
            aircraft.weight_balance.COG(10,4,1) = aircraft.weight_balance.COG(4,4,1);
            aircraft.weight_balance.COG(10,1,1) = str.vtail.CG;
            aircraft.weight_balance.COG(10,2,1) = -aircraft.weight_balance.COG(4,2,1);
          end
      end
      %
      aircraft.weight_balance.COG(5,4,1) = sum(str.fus.WTOT);
      aircraft.weight_balance.COG(5,1,1) = str.fus.CG;

      aircraft.weight_balance.COG(27,1,1) = aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,1,1)'*...
          aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)/...
          (sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)));
      TOTW = sum(aircraft.weight_balance.COG([1,2,3,4,5,10,11,12],4,1));
      if (ii>1)
          TOLL = abs(STRW(ii-1,2) - TOTW) / STRW(1,2);
      end
      STRW = [STRW; ii-1, TOTW, TOLL];
      ii = ii+1;
      fprintf(fid, '\n- Creating baseline stick model...');
      [stick, geo] = Stick_Model(pdcylin, aircraft, geo);
      fprintf(fid, 'done.');
%
%-------------------------------------------------------------------------------
%     SECTION MECHANICAL PROPERTIES
%-------------------------------------------------------------------------------
%
      fprintf(fid, '\n- Estimating mechanical properties...');
      [str] = Prop_Sec(fid, pdcylin, stick, geo, str,aircraft);
      fprintf(fid, 'done.');
%
%-------------------------------------------------------------------------------
%     NON STRUCTURAL MASSES DETERMINATION
%-------------------------------------------------------------------------------
%
      fprintf(fid, '\n\t- Including non-structural masses...');
      [str] = Add_NSM(fid, pdcylin, aircraft, geo, stick, str);
      fprintf(fid, '\ndone.');
%
    end
  end % smartcad coupling
end% end if exist(filename_trim, 'file')
fprintf('\ndone.\n');
%
%-------------------------------------------------------------------------------
% Export OEW model
%-------------------------------------------------------------------------------
fp = fopen(filename_stick, 'w');
fprintf(fp, '\n$\n$ Summary for total structural masses (CT included except for tailbooms): ');
fprintf(fp, ['\n$ Wing: ',      num2str(aircraft.weight_balance.COG(1,4,1)), ' Kg']);
fprintf(fp, ['\n$ Htail: ',     num2str(aircraft.weight_balance.COG(3,4,1)), ' Kg']);
fprintf(fp, ['\n$ Vtail: ',     num2str(aircraft.weight_balance.COG(4,4,1)), ' Kg']);
fprintf(fp, ['\n$ Fuselage: ',  num2str(aircraft.weight_balance.COG(5,4,1)), ' Kg']);
fprintf(fp, ['\n$ Canard: ',    num2str(aircraft.weight_balance.COG(11,4,1)), ' Kg']);
fprintf(fp, ['\n$ Tailbooms: ', num2str(aircraft.weight_balance.COG(12,4,1)), ' Kg']);
%
fprintf(fp, ['\n$\n$ Summary for secondary masses along fuselage (as PBAR density):']);
fprintf(fp, ['\n$ Systems: ', num2str(aircraft.weight_balance.COG(17,4,1)), ' Kg']);
fprintf(fp, ['\n$ Interior:  ', num2str(aircraft.weight_balance.COG(21,4,1)), ' Kg']);
fprintf(fp, ['\n$ Pilots: ',    num2str(aircraft.weight_balance.COG(22,4,1)), ' Kg']);
fprintf(fp, ['\n$ Crew: ',      num2str(aircraft.weight_balance.COG(23,4,1)), ' Kg']);

% Compute total paint weight
nm = fieldnames(str);
pw = 0;
for k = 1:numel(nm),
    if isfield(str.(nm{k}), 'paint')
        pw = pw + str.(nm{k}).paint;
    end
end
fprintf(fp, ['\n$ Paint: ',     num2str(pw), ' Kg\n$']);
% update distributed masses 
% [strDUM] = Add_NSM(fid, pdcylin, aircraft, geo, stick, str);
strDUM = str;
%
writeFUSE_DP2file(fid, fp, pdcylin, aircraft);
%   MAT1 card
writeMAT12file(fid, fp, pdcylin, stick, aircraft);
%   GRID card
writeGRID2file(fid, fp, stick, aircraft);
%   CBAR card
[stick] = writeCBAR2file(fid, fp, stick, geo, aircraft);
%   RBE0 card
[stick] = writeRBE02file(fid, fp, stick, geo, aircraft);
%   PBAR / PBARSMX card
[geo, strDUM, stick] = setup_PROPERTY_BAR(fid, fp, pdcylin, geo, loads, strDUM, stick, aircraft);
%   CAERO1 card
[stick] = writeCAERO2file(fid, fp, aircraft, stick, geo, 0, pdcylin.smartcad.caerob);
%   AELINK card
writeAELINK2file(fid, fp, pdcylin, geo, stick, aircraft, 1);
%   SET1 card
writeSET12file(fid, fp, stick, aircraft, pdcylin, 1);
switch(pdcylin.smartcad.spline_type)
  case 1
% SPLINE1 card
    TCOND = pdcylin.smartcad.tcond;
    writeSPLINE12file(fid, fp, stick, aircraft, TCOND);
  case 2
% SPLINE2 card
    RMAX = pdcylin.smartcad.rmax ;
    TCOND = pdcylin.smartcad.tcond;
    writeSPLINE22file(fid, fp, stick, aircraft, RMAX, TCOND);
  case 3
% SPLINE3 card
    POLY = pdcylin.smartcad.poly;
    W = pdcylin.smartcad.weight;
    NP = pdcylin.smartcad.npoints;
    RMAX = pdcylin.smartcad.rmax ;
    TCOND = pdcylin.smartcad.tcond;
    writeSPLINE22file(fid, fp, stick, aircraft, POLY, W, NP, RMAX, TCOND);

end
writeRIGIDlinkRBE2(fp, stick, 1);
%
%
%-------------------------------------------------------------------------------
% EXPORT SMARTCAD FILE for MASS CONFS
%-------------------------------------------------------------------------------
%
for i = 1 : Nconf
% update conf
  aircraft2 = aircraft;
  aircraft2.weight_balance.COG(18,4,1) = aircraft.weight_balance.COG(18,4,1) * pdcylin.MassConf.Wfuel(i);
  aircraft2.weight_balance.COG(19,4,1) = aircraft.weight_balance.COG(19,4,1) * pdcylin.MassConf.Cfuel(i);
  aircraft2.weight_balance.COG(24,4,1) = aircraft.weight_balance.COG(24,4,1) * pdcylin.MassConf.Pass(i);
  aircraft2.weight_balance.COG(25,4,1) = aircraft.weight_balance.COG(25,4,1) * pdcylin.MassConf.Baggage(i);
%
  indE = find(filename_stick=='.');
  indE = indE(end);
  est = filename_stick(indE:end);
  filename_stick2 = filename_stick(1:indE-1);
  fprintf(fid, '\n- Exporting stick model output file...');
  fp2 = fopen([filename_stick2,'CONM_CONF',num2str(i),est], 'w');
  point = fp2;
  fprintf(point, ['\n$ Mass Configuration ', num2str(i)]);
  value = aircraft2.weight_balance.COG(25,4,1);
  fprintf(point, ['\n$ Baggage: ',           num2str(pdcylin.MassConf.Baggage(i)*100),'%% (', num2str(value),' Kg)']);
  value = aircraft2.weight_balance.COG(24,4,1);
  fprintf(point, ['\n$ Passengers: ',        num2str(pdcylin.MassConf.Pass(i)   *100),'%% (', num2str(value),' Kg)']);
  value = aircraft2.weight_balance.COG(18,4,1);
  fprintf(point, ['\n$ Wing Fuel: ',         num2str(pdcylin.MassConf.Wfuel(i)  *100),'%% (', num2str(value),' Kg)']);
  value = aircraft2.weight_balance.COG(19,4,1);
  fprintf(point, ['\n$ Central Fuel: ',      num2str(pdcylin.MassConf.Cfuel(i)  *100),'%% (', num2str(value),' Kg)']);
% update conf
  pdcylin2 = pdcylin;
  pdcylin2.MassConf.WfuelArrive = pdcylin.MassConf.WfuelArrive(i);
  pdcylin2.MassConf.WfuelStart = pdcylin.MassConf.WfuelStart(i);
  pdcylin2.MassConf.Wfuel = pdcylin.MassConf.Wfuel(i);
%
  pdcylin2.MassConf.BagArrive = pdcylin.MassConf.BagArrive(i);
  pdcylin2.MassConf.BagStart = pdcylin.MassConf.BagStart(i);
  pdcylin2.MassConf.Baggage = pdcylin.MassConf.Baggage(i);
%
  pdcylin2.MassConf.PaxArrive = pdcylin.MassConf.PaxArrive(i);
  pdcylin2.MassConf.PaxStart = pdcylin.MassConf.PaxStart(i);
  pdcylin2.MassConf.Pass = pdcylin.MassConf.Pass(i);
% update distributed masses to be exported as CONM2
  [strDUM] = Add_NSM(fid, pdcylin2, aircraft, geo, stick, str);
% export conf masses and OEW masses if i == 1
  [strDUM, ~, ~] = Add_NSM_conc(fid, fp2, fp, pdcylin2, aircraft2, geo, stick, strDUM, 1, 1);
  if (i ==1)
    fclose(fp); % close OEW file
    fp = [];
  end
  fclose(fp2);
end
%
% update wing data to account for regression masses
%
aircraft.weight_balance.COG(1,1,1) = ...
     (str.wing.CG*(2*sum(str.wing.WBOX) + str.wing.WC) + strDUM.wing.regr.M*strDUM.wing.regr.CG)/aircraft.weight_balance.COG(1,4,1);
%
% FIND SUPORT NODE
%
indE = find(filename_stick=='.'); indE = indE(end);
ext = filename_stick(indE:end);
for i = 1 : Nconf
  fp = fopen('dummy_model.dat','w');
  filename_conf = [filename_stick(1:indE-1),'CONM_CONF',num2str(i),ext];
  writeINCLUDE2file(fp, filename_stick);
  writeINCLUDE2file(fp, filename_conf);
  fclose(fp);
  beam_model = load_nastran_model('dummy_model.dat');
  strNode = unique([beam_model.Bar.Conn(:,1);beam_model.Bar.Conn(:,3)]);
  [dummyCG,idSuport] = min((beam_model.WB.CG(1,1)-beam_model.Node.Coord(strNode,1)).^2+...
    (beam_model.WB.CG(1,2)-beam_model.Node.Coord(strNode,2)).^2+...
    (beam_model.WB.CG(1,3)-beam_model.Node.Coord(strNode,3)).^2);
  fp = fopen(filename_conf,'a+');
  fprintf(fp, '\n$ SUPORT FOR CURRENT CONFIGURATION\n');
  BULKdataSUPORT(fp,beam_model.Node.ID(strNode(idSuport)));
  fprintf(fp, '$\n');
  fclose(fp);
end
%
beam_model = load_nastran_model(filename_stick);
OEW = beam_model.WB.MCG(1,1);
OEWCG = beam_model.WB.CG;
%
delete('dummy_model.dat')
% save stick model
guess_model.stick_model = stick;
% save geo
guess_model.geo = geo;
% save loads
guess_model.loads = loads;
% save str
guess_model.str_prop = str;
% save pdcylin
guess_model.pdcylin = pdcylin;
% save aircraft
guess_model.aircraft = aircraft;
% save optimization histo
guess_model.optim  = optim;
aircraft = print_WB(fid, str, aircraft, pdcylin, stick, geo, beam_model.Aero.ref, OEW, OEWCG(1));
% save reference MAC data
guess_model.stick_model.ref.C_mac = beam_model.Aero.ref.C_mac;
guess_model.stick_model.ref.XLE_C_mac = beam_model.Aero.ref.XLE_C_mac;
% save reference OEW data
guess_model.stick_model.OEW = OEW;
guess_model.stick_model.OEW_CG = OEWCG;
%-------------------------------------------------------------------------------
%
% OUTPUT ITER HISTORY
%
fprintf(fid,'\n\t-------------------------------------------- CONVERGENCE -------------------------------------------');
fprintf(fid, '\n\t- Refinement loop history:');
fprintf(fid, '\n\t\tIter %4d: Total structural mass: %-g Kg. Tolerance: %-.3e.', STRW(2:end,:)');
fprintf(fid, '\n');
%
%-------------------------------------------------------------------------------
%
% SAVE GUESS MODEL
%
filename_guess = [filename_stick(1:indE-1),'_guess.mat'];
save(filename_guess, 'guess_model');
fprintf(fid, '\n\t- GUESS model saved in %s file.', filename_guess);
%
filename_guess = [filename_stick(1:indE-1),'_guess.txt'];
fp = fopen(filename_guess, 'w'); 
print_WB(fp, str, aircraft, pdcylin, stick, geo, beam_model.Aero.ref, OEW, OEWCG(1)); 
fclose(fp);
fprintf(fid, '\n\t- GUESS summary saved in %s file.', filename_guess);
%
fprintf(fid, '\n\t- SMARTCAD main file with OEW configuration saved in %s.', filename_stick);
for i = 1 :Nconf
    filename_conf = [filename_stick(1:indE-1),'CONM_CONF',num2str(i),ext];
    fprintf(fid, '\n\t- SMARTCAD configuration file saved in %s file.', filename_conf);
end
fprintf(fid, '\n\n');
%filename_nas = [filename_stick(1:indE-1),'.nas'];
%neo2nastran(filename_stick, filename_nas);
%fprintf(fid, '\n\t- NASTRAN main file saved in %s.\n\n', filename_nas);
%
end
function[p2]=trot3(hinge,p,alpha)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TROT: Auxillary rotation function			
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rotates point p around hinge alpha rads.%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ref: 	R�de, Westergren, BETA 4th ed,   
%			studentlitteratur, 1998			    	
%			pp:107-108							   	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: 	Tomas Melin, KTH,Department of%
% 				aeronautics, Copyright 2000	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Context:	Auxillary function for			
%				TORNADO.								
% Called by: setrudder, normals			
% Calls:		norm (MATLAB std fcn)			
%				sin			"						
%				cos			"						
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HELP:		Hinge=vector around rotation  
%						takes place.				
%				p=point to be rotated			
%				alpha=radians of rotation		
%				3D-workspace						
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a=hinge(1);
b=hinge(2);
c=hinge(3);

rho=sqrt(a^2+b^2);
r=sqrt(a^2+b^2+c^2);

if r==0
   cost=0
   sint=1;
else
   cost=c/r;
   sint=rho/r;
end

if rho==0
   cosf=0;
   sinf=1;
else
   cosf=a/rho;
	sinf=b/rho;
end   

cosa=cos(alpha);
sina=sin(alpha);

RZF=[[cosf -sinf 0];[sinf cosf 0];[0 0 1]];
RYT=[[cost 0 sint];[0 1 0];[-sint 0 cost]];
RZA=[[cosa -sina 0];[sina cosa 0];[0 0 1]];
RYMT=[[cost 0 -sint];[0 1 0];[sint 0 cost]];
RZMF=[[cosf sinf 0];[-sinf cosf 0];[0 0 1]];

P=RZF*RYT*RZA*RYMT*RZMF;
p2=P*p';
end