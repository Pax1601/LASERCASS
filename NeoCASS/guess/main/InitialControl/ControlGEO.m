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
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Main script to performed Generic Unknown Estimator for Structural Sizing
%
%
% Called by:    guess.m
%
% Calls:        Geo_Fus.m , Geo_Wing.m , Geo_Vtail.m , Geo_Htail.m
%               Load_Fus.m, Load_Wing.m, Load_Vtail.m, Load_Htail.m
%               Str_Fus.m , Str_Wing.m , Str_Vtail.m , Str_Htail.m
%               Regr_Fus.m, Regr_Wing.m, Regr_Vtail.m, Regr_Htail.m,
%               set_stability_der.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080407      1.0     A. Da Ronch      Creation
%     091119      1.3.9   L. Travaglini    Modification
%
% Modified by Travaglini, adding analysis on canard
%*******************************************************************************
function [aircraft,pdcylin,go,geo,stick] =  ControlGEO(filename_geo, filename_tech, filename_trim, model, inBatch)
global geo_model NMAX EPS
global beam_model
fid = 1;
aircraft = neocass_xmlwrapper(filename_geo);
SIGMA_TAU = sqrt(3);
%
% material data
fprintf(fid, '\n\tTechnology input file: %s.', filename_tech);
if strcmp(filename_geo, filename_tech),
    pdcylin = struct('user_input', struct, 'experienced_user_input', struct);
    if isfield(aircraft,'user_input')
        pdcylin.user_input = aircraft.user_input;
    else
        fprintf('\n\tField user_input is not present in structure');
    end
    if isfield(aircraft,'experienced_user_input')
        pdcylin.experienced_user_input = aircraft.experienced_user_input;
    else
        fprintf('\n\tField experienced_user_input is not present in structure');
    end
    % aircraft = rmfield(aircraft, {'user_input', 'experienced_user_input'});
else
    pdcylin = neocass_xmlwrapper(filename_tech);
    if isfield(pdcylin,'user_input')
        aircraft.user_input = pdcylin.user_input;
    else
        fprintf('\n\tField user_input is not present in structure');
    end
    if isfield(aircraft,'experienced_user_input')
        aircraft.experienced_user_input = pdcylin.experienced_user_input;
    else
        fprintf('\n\tField experienced_user_input is not present in structure');
    end
end
% assembly aircraft to use to check structure input
load('gtemplate_empty.mat');
load('gtemplate_full.mat');
comp = empty_model;

if isfield(aircraft,'Horizontal_tail') && aircraft.Horizontal_tail.present
    comp.Horizontal_tail = full_model.Horizontal_tail;
end
if isfield(aircraft,'Vertical_tail') && aircraft.Vertical_tail.present
    comp.Vertical_tail = full_model.Vertical_tail;
end
if isfield(aircraft,'Canard') && aircraft.Canard.present
    comp.Canard = full_model.Canard;
end
if isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present
    comp.Tailbooms = full_model.Tailbooms; 
end
[go, stringa] = CompareSTR(comp,aircraft);  
if go == 0
    fprintf('\n\tThe following fields are not present in structure aircraft:');
    fprintf('\n');
    for i = 1 : length(stringa)
        fprintf(['\n\t\t',stringa{i}]);
    end
    return
else
    if ~isfield(aircraft.user_input.material_property.wing,'msl')
         fprintf('\n\t### Warning: missing input max. tensile stress for main wing.');
         fprintf('\n\t    Value set equal to max. shear stress * %g', SIGMA_TAU);
        pdcylin.user_input.material_property.wing.msl = SIGMA_TAU * pdcylin.user_input.material_property.wing.fcsw;
    else
      if(aircraft.user_input.material_property.wing.msl<eps)
        pdcylin.user_input.material_property.wing.msl = SIGMA_TAU * pdcylin.user_input.material_property.wing.fcsw;
      else
        pdcylin.user_input.material_property.wing.msl = aircraft.user_input.material_property.wing.msl;
      end
    end
%
    if aircraft.Horizontal_tail.present
        if ~isfield(aircraft.user_input.material_property.htail,'msl')
         fprintf('\n\t### Warning: missing input max. tensile stress for horizontal tailplane.');
         fprintf('\n\t    Value set equal to max. shear stress * %g', SIGMA_TAU);
            pdcylin.user_input.material_property.htail.msl = SIGMA_TAU * pdcylin.user_input.material_property.htail.fcsw;
        else
          if (aircraft.user_input.material_property.htail.msl<eps) 
            pdcylin.user_input.material_property.htail.msl = SIGMA_TAU * pdcylin.user_input.material_property.htail.fcsw;    
          else
            pdcylin.user_input.material_property.htail.msl = aircraft.user_input.material_property.htail.msl;
          end
        end
        aircraft.Horizontal_tail.longitudinal_location = aircraft.Horizontal_tail.x;
        aircraft.Horizontal_tail.vertical_location = aircraft.Horizontal_tail.z;
    end
%    
    if aircraft.Canard.present
        if ~isfield(aircraft.user_input.material_property.canard,'msl')
           fprintf('\n\t### Warning: missing input max. tensile stress for canard.');
           fprintf('\n\t    Value set equal to max. shear stress * %g', SIGMA_TAU);
            pdcylin.user_input.material_property.canard.msl = SIGMA_TAU * pdcylin.user_input.material_property.canard.fcsw;
        else
          if(aircraft.user_input.material_property.canard.msl<eps)
            pdcylin.user_input.material_property.canard.msl = SIGMA_TAU * pdcylin.user_input.material_property.canard.fcsw;
          else
            pdcylin.user_input.material_property.canard.msl = aircraft.user_input.material_property.canard.msl;
          end
        end
        aircraft.Canard.longitudinal_location = aircraft.Canard.x;
        aircraft.Canard.vertical_location = aircraft.Canard.z;
    end
    if ~isfield(aircraft.user_input.material_property.vtail,'msl')
        fprintf('\n\t### Warning: missing input max. tensile stress for vertical taiplane.');
           fprintf('\n\t    Value set equal to max. shear stress * %g', SIGMA_TAU);
        pdcylin.user_input.material_property.vtail.msl = SIGMA_TAU * pdcylin.user_input.material_property.vtail.fcsw;
    else
      if(aircraft.user_input.material_property.vtail.msl<eps)
        pdcylin.user_input.material_property.vtail.msl = SIGMA_TAU * pdcylin.user_input.material_property.vtail.fcsw;
      else
        pdcylin.user_input.material_property.vtail.msl = aircraft.user_input.material_property.vtail.msl;
      end
    end
    aircraft.Vertical_tail.longitudinal_location = aircraft.Vertical_tail.x;
    aircraft.Vertical_tail.vertical_location     = aircraft.Vertical_tail.z;
    
    aircraft.Wing1.longitudinal_location = aircraft.Wing1.x;
    aircraft.Wing1.vertical_location     = aircraft.Wing1.z;
    
    if aircraft.Wing2.present
        aircraft.Wing2.longitudinal_location = aircraft.Wing2.x;
        aircraft.Wing2.vertical_location     = aircraft.Wing2.z;
    end
    winglet = aircraft.Wing1.winglet;
    pdcylin = setup_tech_conversion(pdcylin,aircraft);
    pdcylin = setup_sma_defaults(pdcylin);
    aircraft = setup_geofile_conversion(aircraft); 
    aircraft.winglet = winglet;
    if aircraft.Canard.present
        pdcylin.stick.model.canr = 1;
    else
        pdcylin.stick.model.canr = 0;
    end
    if aircraft.Horizontal_tail.present
        pdcylin.stick.model.horr = 1;
    else
        pdcylin.stick.model.horr = 0;
    end
    if aircraft.Vertical_tail.present
        pdcylin.stick.model.vert = 1;
        if isequal(aircraft.Vertical_tail.Twin_tail, 1)
          pdcylin.stick.model.twin = 1;
        else
          pdcylin.stick.model.twin = 0;
        end
    else
        pdcylin.stick.model.vert = 0;
        pdcylin.stick.model.twin = 0;
    end
    %
    %--------------------------------------------------------------------------
    % Create structure
    %--------------------------------------------------------------------------
    %
    geo.fus     = [];     % Geometry for FUSELAGE
    geo.wing    = [];     % Geometry for WING
    geo.vtail   = [];     % Geometry for VERTICAL TAIL
    geo.htail   = [];     % Geometry for HORIZONTAL TAIL
    geo.wing2   = [];     % Geometry for WING2
    geo.canard  = [];     % Geometry for CANARD
    %
    loads.fus    = [];     % Load for FUSELAGE
    loads.wing   = [];     % Load for WING
    loads.vtail  = [];     % Load for VERTICAL TAIL
    loads.htail  = [];     % Load for HORIZONTAL TAIL
    loads.wing2  = [];
    loads.canard = [];
    %
    str.fus     = [];     % Structure for FUSELAGE
    str.wing    = [];     % Structure for WING
    str.vtail   = [];     % Structure for VERTICAL TAIL
    str.htail   = [];     % Structure for HORIZONTAL TAIL
    str.M       = 0;      % scalar, to determine the mass computed and written in the *.dat file [kg]
    %
    % Process model struct
    %
    aircraft.Joined_wing.present = 0;
    aircraft.Strut_wing.present = 0;
    if (model.Joined_wing)
      fprintf(fid, '\n\t- Joined wing selected.');   
      aircraft.Joined_wing.present = 1;
    end
    if (model.Strut_wing)
      fprintf(fid, '\n\t- Strut braced wing selected.');   
      aircraft.Strut_wing.present = model.Strut_wing;
    end
%
%   switch off regression for strut
    if (aircraft.Strut_wing.present==1)
      fprintf('\n\t### Warning: regression disabled for strut component.');
      fprintf('\n\t### Warning: canard kcon set to 8 (circular section strut).');
      pdcylin.smartcad.canard_regr = 0;
      pdcylin.canard.kcon = 8;
    end

    %
    %--------------------------------------------------------------------------
    % GEOMETRY MODULUS
    %--------------------------------------------------------------------------
    %
    fprintf(fid, '\n\tDetermining geometry parameters...');
    geo = Geo_Fus(pdcylin, aircraft, geo);
    [geo, pdcylin] = Geo_Wing(pdcylin, aircraft, geo);
    if isfield(aircraft, 'Tailbooms') && aircraft.Tailbooms.present
        geo = Geo_Tbooms(pdcylin, aircraft, geo);
        pdcylin.stick.model.tboomsr = 1;
    else
        pdcylin.stick.model.tboomsr = 0;
    end
    if aircraft.Vertical_tail.present
        [geo, pdcylin] = Geo_Vtail(pdcylin, aircraft, geo);
    end
    if aircraft.Horizontal_tail.present   
        [geo, pdcylin] = Geo_Htail(pdcylin, aircraft, geo); 
    end
    if aircraft.Canard.present
        [geo, pdcylin] = Geo_Canard(pdcylin, aircraft, geo);
    end
    if aircraft.wing2.present
        [geo, pdcylin] = Geo_Wing2(pdcylin, aircraft, geo);
    else
        pdcylin.stick.model.win2r = 0;
    end
    fprintf(fid, '\n\tdone.');
    %--------------------------------------------------------------------------
    % Wing
    %--------------------------------------------------------------------------
    if isequal(pdcylin.stick.model.winr, 1) && isequal(pdcylin.stick.model.fuse, 1)
        %
        geo.wing.QC(2,:) = geo.wing.QC(2,:) + geo.fus.R;
        QC = geo.wing.QC(:,1);
        QC(2) = 0.0;
        geo.wing.QC = [QC, geo.wing.QC];
        %
        geo.wing.C2(2,:) = geo.wing.C2(2,:) + geo.fus.R;
        C2 = geo.wing.C2(:,1);
        C2(2) = 0.0;
        geo.wing.C2 = [C2, geo.wing.C2];
        %
        geo.wing.PANE(2,:) = geo.wing.PANE(2,:) + geo.fus.R;
        PANE = [geo.wing.PANE(:,1), geo.wing.PANE(:,1), geo.wing.PANE(:,4), geo.wing.PANE(:,4)];
        PANE(2,1) = 0.0;
        PANE(2,4) = 0.0;
        geo.wing.PANE = [PANE, geo.wing.PANE];
        %
        geo.wing.WING(2,:) = geo.wing.WING(2,:) + geo.fus.R;
        geo.wing.WING = [PANE, geo.wing.WING];
        %
        geo.wing.CAERO1.n = [pdcylin.stick.nwing_carryth; geo.wing.CAERO1.n];
        geo.wing.CAERO1.n_coarse = [pdcylin.stick.nwing_carryth_coarse; geo.wing.CAERO1.n_coarse];
        % Number of nodes within half-carrythrough structure
        nrcth = pdcylin.stick.nwing_carryth;
        % GEO
        Zcth = geo.wing.CR*ones(nrcth,1);
        geo.wing.Z = [Zcth; geo.wing.Z];
        %
        Zscth = geo.wing.CSR*ones(nrcth,1);
        geo.wing.Zs = [Zscth; geo.wing.Zs];
        %
        rscth = geo.wing.rs(1)*ones(nrcth,1);
        geo.wing.rs = [rscth; geo.wing.rs];
        %
        rcth = geo.wing.r(1)*ones(nrcth,1);
        geo.wing.r = [rcth; geo.wing.r];
        %
        incth = zeros(nrcth, 1);
        geo.wing.incidence = [incth; geo.wing.incidence];
        %
        tbscth = geo.wing.tcs*ones(nrcth,1);
        geo.wing.tbs = [tbscth; geo.wing.tbs];
        %
        geo.wing.leny = geo.wing.leny + nrcth;
        %
        geo.wing.box.y = geo.wing.y;
        
        dycth = norm(geo.wing.QC(:,2)-geo.wing.QC(:,1)) / nrcth;
        ycth = (0: dycth : dycth*nrcth)';
        geo.wing.y = [ycth; ycth(end)+geo.wing.y(2:end)];
        %
        geo.wing.box.V = geo.wing.V;
        %
        Swetcth = geo.wing.Swet(1)*ones(nrcth,1);
        geo.wing.Swet = [Swetcth; geo.wing.Swet];
        %
        indexcth = [1; nrcth+1];
        geo.wing.index = [indexcth; indexcth(end)+geo.wing.index(2:end)-1];
        % CAERO1
        geo.wing.CAERO1.dihedral = [0.0; geo.wing.CAERO1.dihedral];
        geo.wing.CAERO1.chord = [geo.wing.CAERO1.chord(1); geo.wing.CAERO1.chord];
        geo.wing.CAERO1.span = [geo.fus.R; geo.wing.CAERO1.span];
        geo.wing.CAERO1.taper = [geo.wing.CAERO1.chord(2)/geo.wing.CAERO1.chord(1); geo.wing.CAERO1.taper];
        geo.wing.CAERO1.sweepQC = [0.0; geo.wing.CAERO1.sweepQC];
        geo.wing.CAERO1.sweepC2 = [0.0; geo.wing.CAERO1.sweepC2];
        geo.wing.CAERO1.incidence = [geo.wing.CAERO1.incidence(1); geo.wing.CAERO1.incidence];
        geo.wing.CAERO1.sup_control.frc = [0.0; 0.0; geo.wing.CAERO1.sup_control.frc];
        geo.wing.CAERO1.sup_control.frs = [1.0; geo.wing.CAERO1.sup_control.frs];
        geo.wing.CAERO1.sup_control.nme = ['    none'; geo.wing.CAERO1.sup_control.nme];
        % AELINK card: set 0
        geo.wing.CAERO1.sup_control.typ = [0; geo.wing.CAERO1.sup_control.typ];
        geo.wing.CAERO1.airfoil = [geo.wing.CAERO1.airfoil(1,:); geo.wing.CAERO1.airfoil(1,:); geo.wing.CAERO1.airfoil];
        if isequal(pdcylin.stick.model.symmXZ, 1)
            geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme(1:length(geo.wing.CAERO1.sup_control.typ),:);...
                '    none';...
                geo.wing.CAERO1.sup_control.nme(length(geo.wing.CAERO1.sup_control.typ)+1:end,:)];
        end
    end
    %--------------------------------------------------------------------------
    % Horizontal tail
    %--------------------------------------------------------------------------
    if isequal(pdcylin.stick.model.horr, 1) && geo.htail.twc > 0.0
        %
        geo.htail.QC(2,:) = geo.htail.QC(2,:) + geo.htail.twc/2;
        QC = geo.htail.QC(:,1);
        QC(2) = 0.0;
        geo.htail.QC = [QC, geo.htail.QC];
        %
        geo.htail.C2(2,:) = geo.htail.C2(2,:) + geo.htail.twc/2;
        C2 = geo.htail.C2(:,1);
        C2(2) = 0.0;
        geo.htail.C2 = [C2, geo.htail.C2];
        %
        geo.htail.PANE(2,:) = geo.htail.PANE(2,:) + geo.htail.twc/2;
        PANE = [geo.htail.PANE(:,1), geo.htail.PANE(:,1), geo.htail.PANE(:,4), geo.htail.PANE(:,4)];
        PANE(2,1) = 0.0;
        PANE(2,4) = 0.0;
        geo.htail.PANE = [PANE, geo.htail.PANE];
        %
        geo.htail.WING(2,:) = geo.htail.WING(2,:) + geo.htail.twc/2;
        geo.htail.WING = [PANE, geo.htail.WING];
        %
        geo.htail.CAERO1.n = [pdcylin.stick.nhtail_carryth; geo.htail.CAERO1.n];
        geo.htail.CAERO1.n_coarse = [pdcylin.stick.nhtail_carryth_coarse; geo.htail.CAERO1.n_coarse];
        % Number of nodes within half-carrythrough structure
        nrcth = pdcylin.stick.nhtail_carryth;
        % GEO
        Zscth = geo.htail.CR*ones(nrcth,1);
        geo.htail.Z = [Zscth; geo.htail.Z];
        %
        ZScth = geo.htail.CSR*ones(nrcth,1);
        geo.htail.Zs = [ZScth; geo.htail.Zs];
        %
        rscth = geo.htail.r(1)*ones(nrcth,1);
        geo.htail.r = [rscth; geo.htail.r];
        %
        rScth = geo.htail.rs(1)*ones(nrcth,1);
        geo.htail.rs = [rScth; geo.htail.rs];
        %
        incth = zeros(nrcth,1);
        geo.htail.incidence = [incth; geo.htail.incidence];
        %
        tbscth = geo.htail.tcs*ones(nrcth,1);
        geo.htail.tbs = [tbscth; geo.htail.tbs];
        %
        geo.htail.leny = geo.htail.leny + nrcth;
        %
        geo.htail.box.y = geo.htail.y;
        
        dycth = norm(geo.htail.QC(:,2)-geo.htail.QC(:,1)) / nrcth;
        ycth = (0: dycth : nrcth*dycth)';
        geo.htail.y = [ycth; ycth(end)+geo.htail.y(2:end)];
        %
        Swetcth = geo.htail.Swet(1)*ones(nrcth,1);
        geo.htail.Swet = [Swetcth; geo.htail.Swet];
        %
        indexcth = [1; nrcth+1];
        geo.htail.index = [indexcth; indexcth(end)+geo.htail.index(2:end)-1];
        % CAERO1
        geo.htail.CAERO1.dihedral = [0.0; geo.htail.CAERO1.dihedral];
        geo.htail.CAERO1.chord = [geo.htail.CAERO1.chord(1); geo.htail.CAERO1.chord];
        geo.htail.CAERO1.span = [geo.htail.twc/2; geo.htail.CAERO1.span];
        geo.htail.CAERO1.taper = [geo.htail.CAERO1.chord(2)/geo.htail.CAERO1.chord(1); geo.htail.CAERO1.taper];
        geo.htail.CAERO1.sweepQC = [0.0; geo.htail.CAERO1.sweepQC];
        geo.htail.CAERO1.sweepC2 = [0.0; geo.htail.CAERO1.sweepC2];
        geo.htail.CAERO1.incidence = [geo.htail.CAERO1.incidence(1); geo.htail.CAERO1.incidence];
        geo.htail.CAERO1.sup_control.frc = [0.0; 0.0; geo.htail.CAERO1.sup_control.frc];
        geo.htail.CAERO1.sup_control.nme = ['    none'; geo.htail.CAERO1.sup_control.nme];
        % AELINK card: set 0
        geo.htail.CAERO1.sup_control.typ = [0; geo.htail.CAERO1.sup_control.typ];
        geo.htail.CAERO1.airfoil = [geo.htail.CAERO1.airfoil(1,:); geo.htail.CAERO1.airfoil(1,:); geo.htail.CAERO1.airfoil];
        if isequal(pdcylin.stick.model.symmXZ, 1)
            geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme(1:length(geo.htail.CAERO1.n),:);...
                '    none';...
                geo.htail.CAERO1.sup_control.nme(length(geo.htail.CAERO1.n)+1:end,:)];
        end
    end
    %--------------------------------------------------------------------------
    % Canard
    %--------------------------------------------------------------------------
    if isequal(pdcylin.stick.model.canr, 1) && geo.canard.twc > 0.0
        %
        geo.canard.QC(2,:) = geo.canard.QC(2,:) + geo.canard.twc/2;
        QC = geo.canard.QC(:,1);
        QC(2) = 0.0;
        geo.canard.QC = [QC, geo.canard.QC];
        %
        geo.canard.C2(2,:) = geo.canard.C2(2,:) + geo.canard.twc/2;
        C2 = geo.canard.C2(:,1);
        C2(2) = 0.0;
        geo.canard.C2 = [C2, geo.canard.C2];
        %
        geo.canard.PANE(2,:) = geo.canard.PANE(2,:) + geo.canard.twc/2;
        PANE = [geo.canard.PANE(:,1), geo.canard.PANE(:,1), geo.canard.PANE(:,4), geo.canard.PANE(:,4)];
        PANE(2,1) = 0.0;
        PANE(2,4) = 0.0;
        geo.canard.PANE = [PANE, geo.canard.PANE];
        %
        geo.canard.WING(2,:) = geo.canard.WING(2,:) + geo.canard.twc/2;
        geo.canard.WING = [PANE, geo.canard.WING];
        %
        geo.canard.CAERO1.n = [pdcylin.stick.ncanard_carryth; geo.canard.CAERO1.n];
        geo.canard.CAERO1.n_coarse = [pdcylin.stick.ncanard_carryth_coarse; geo.canard.CAERO1.n_coarse];
        % Number of nodes within half-carrythrough structure
        nrcth = pdcylin.stick.ncanard_carryth;
        
        % GEO
        Zscth = geo.canard.CR*ones(nrcth,1);
        geo.canard.Z = [Zscth; geo.canard.Z];
        %
        ZScth = geo.canard.CSR*ones(nrcth,1);
        geo.canard.Zs = [ZScth; geo.canard.Zs];
        %
        rscth = geo.canard.r(1)*ones(nrcth,1);
        geo.canard.r = [rscth; geo.canard.r];
        %
        rScth = geo.canard.rs(1)*ones(nrcth,1);
        geo.canard.rs = [rScth; geo.canard.rs];
        %
        incth = zeros(nrcth,1);
        geo.canard.incidence = [incth; geo.canard.incidence];
        %
        tbscth = geo.canard.tcs*ones(nrcth,1);
        geo.canard.tbs = [tbscth; geo.canard.tbs];
        %
        geo.canard.leny = geo.canard.leny + nrcth;
        %
        geo.canard.box.y = geo.canard.y;
        
        dycth = norm(geo.canard.QC(:,2)-geo.canard.QC(:,1)) / nrcth;
        ycth = (0: dycth : nrcth*dycth)';
        geo.canard.y = [ycth; ycth(end)+geo.canard.y(2:end)];
        %
        Swetcth = geo.canard.Swet(1)*ones(nrcth,1);
        geo.canard.Swet = [Swetcth; geo.canard.Swet];
        %
        indexcth = [1; nrcth+1];
        geo.canard.index = [indexcth; indexcth(end)+geo.canard.index(2:end)-1];
        % CAERO1
        geo.canard.CAERO1.dihedral = [0.0; geo.canard.CAERO1.dihedral];
        geo.canard.CAERO1.chord = [geo.canard.CAERO1.chord(1); geo.canard.CAERO1.chord];
        geo.canard.CAERO1.span = [geo.canard.twc/2; geo.canard.CAERO1.span];
        geo.canard.CAERO1.taper = [geo.canard.CAERO1.chord(2)/geo.canard.CAERO1.chord(1); geo.canard.CAERO1.taper];
        geo.canard.CAERO1.sweepQC = [0.0; geo.canard.CAERO1.sweepQC];
        geo.canard.CAERO1.sweepC2 = [0.0; geo.canard.CAERO1.sweepC2];
        geo.canard.CAERO1.incidence = [geo.canard.CAERO1.incidence(1); geo.canard.CAERO1.incidence];
        geo.canard.CAERO1.sup_control.frc = [0.0; 0.0; geo.canard.CAERO1.sup_control.frc];
        geo.canard.CAERO1.sup_control.nme = ['    none'; geo.canard.CAERO1.sup_control.nme];
        % AELINK card: set 0
        geo.canard.CAERO1.sup_control.typ = [0; geo.canard.CAERO1.sup_control.typ];
        geo.canard.CAERO1.airfoil = [geo.canard.CAERO1.airfoil(1,:); geo.canard.CAERO1.airfoil(1,:); geo.canard.CAERO1.airfoil];
        
        if isequal(pdcylin.stick.model.symmXZ, 1)
            geo.canard.CAERO1.sup_control.nme = [geo.canard.CAERO1.sup_control.nme(1:length(geo.canard.CAERO1.n),:);...
                '    none';...
                geo.canard.CAERO1.sup_control.nme(length(geo.canard.CAERO1.n)+1:end,:)];
        end
    end
%
    fprintf(fid, '\n\tCreating geometric stick model...');   
    [stick, geo] = Stick_Model(pdcylin, aircraft, geo);
    fprintf(fid, 'done.');
%   write geometry aero model and grid points    
    fid2 = fopen('gstd_model.dat','w'); 
    writeGRID2file(fid, fid2, stick, aircraft);
    [stick] = writeCAERO2file(1, fid2, aircraft, stick, geo, 0, pdcylin.smartcad.caerob);
    MASTER_LABEL = writeAELINK2file(fid, fid2, pdcylin, geo, stick, aircraft, 1);
    writeAEROS2file(fid2, stick, aircraft);
%   write CONM2 for CG
    all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.vert2 ;stick.ID.canr; stick.ID.canl; stick.ID.horr; stick.ID.horl;stick.ID.tboomsr; stick.ID.tboomsl];
    all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.vert2, stick.nodes.canrC2, stick.nodes.canlC2,stick.nodes.horrC2, stick.nodes.horlC2, stick.nodes.tboomsr, stick.nodes.tboomsl];
    distance = all_nodes - meshgrid([aircraft.weight_balance.COG(27,1,1),aircraft.weight_balance.COG(27,2,1),aircraft.weight_balance.COG(27,3,1)], 1:size(all_nodes,2) )';
    distance = distance(1,:).^2 + distance(2,:).^2 + distance(3,:).^2;
    [dummy, indCG] = min(distance);
    %[stick] = writeRBE02file(fid, fid2, stick, geo, aircraft);
    writeSET12file(fid, fid2, stick, aircraft, pdcylin, 1);
    switch(pdcylin.smartcad.spline_type)
      case 1
    % SPLINE1 card
        TCOND = pdcylin.smartcad.tcond;
        writeSPLINE12file(fid, fid2, stick, aircraft, TCOND);
      case 2
    % SPLINE2 card
        RMAX = pdcylin.smartcad.rmax ;
        TCOND = pdcylin.smartcad.tcond;
        writeSPLINE22file(fid, fid2, stick, aircraft, RMAX, TCOND);
      case 3
    % SPLINE3 card
        POLY = pdcylin.smartcad.poly;
        W = pdcylin.smartcad.weight;
        NP = pdcylin.smartcad.npoints;
        RMAX = pdcylin.smartcad.rmax ;
        TCOND = pdcylin.smartcad.tcond;
        writeSPLINE22file(fid, fid2, stick, aircraft, POLY, W, NP, RMAX, TCOND);

    end
    
%
    fprintf(fid2,'PARAM   GRDPNT  '); label = cnvt2_8chs(all_ID(indCG)); fprintf(fid2, '%c', label); fprintf(fid2,'\n');
%
    M = sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1));
    Jin = diag(aircraft.weight_balance.Imat);
    Offset = aircraft.weight_balance.COG(27,1:3,1) - all_nodes(:, indCG)';
    BULKdataCONM2(fid2, 1,  all_ID(indCG), 0, M, Offset(1) , Offset(2) , Offset(3), Jin(1), 0.0, Jin(2), 0.0, 0.0, Jin(3));
%-------------------------------------------------------------------------------------------------------------
    TrimID = 1;
    if (isempty(filename_trim))
      fprintf(fid2,'TRIM=   ');
      fprintf(fid2, '%c', cnvt2_8chs(TrimID));
      fprintf(fid2, '\n$ Run solve_vlm_rigid and get aero data');
      fprintf(fid2,'\nTRIM    ');
      fprintf(fid2, '%c', cnvt2_8chs(TrimID));
      fprintf(fid2, '1       0.3     0.0     SIDES   0.0     ROLL    0.0     ', TrimID);
      fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     URDD3   9.81    ');
      fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     ANGLEA  0.0     \n        ');
      ncsm = length(MASTER_LABEL.All);
      for (n=1:ncsm)
        [LABLIstr] = cnvt2_8chs(MASTER_LABEL.All{n});
        fprintf(fid2, '%c', LABLIstr); fprintf(fid2, '0.0     '); 
        if (n==4 && ncsm>4) 
          fprintf(fid2,'\n        ');
        end
      end
%
    else
      writeINCLUDE2file(fid2, filename_trim);
      fprintf(fid2, '\n$ Uncomment the following lines to run solve_vlm_rigid and get aero data');
      fprintf(fid2,'\n$TRIM    ');
      fprintf(fid2, '%c', cnvt2_8chs(TrimID));
      fprintf(fid2, '1       0.3     0.0     SIDES   0.0     ROLL    0.0     ', TrimID);
      fprintf(fid2,'\n$        PITCH   0.0     YAW     0.0     URDD2   0.0     URDD3   9.81    ');
      fprintf(fid2,'\n$        URDD4   0.0     URDD5   0.0     URDD6   0.0     ANGLEA  0.0     \n$        ');
      ncsm = length(MASTER_LABEL.All);
      for (n=1:ncsm)
        [LABLIstr] = cnvt2_8chs(MASTER_LABEL.All{n});
        fprintf(fid2, '%c', LABLIstr); fprintf(fid2, '0.0     '); 
        if (n==4 && ncsm>4) 
          fprintf(fid2,'\n$        ');
        end
      end
    end
%
%  Add LG data
%
  Xcg = aircraft.weight_balance.COG(6,1,1);
  Ycg = abs(aircraft.weight_balance.COG(6,2,1));
  Zcg = aircraft.weight_balance.COG(6,3,1);
  if Xcg > 0.0
    fprintf(fid2,'\n'); 
    IDclc = 9000; BULKdataGRID(fid2, IDclc, 0, Xcg, Ycg, Zcg, 0, 0, 0);
    fprintf(fid2, 'PARAM   LANDG   %d\n', IDclc);
    if (Ycg~=0)
      IDclc = 9001; BULKdataGRID(fid2, IDclc, 0, Xcg, -Ycg, Zcg, 0, 0, 0);
      fprintf(fid2, 'PARAM   LANDG   %d\n', IDclc);
    end
  end
  fclose(fid2);
%-------------------------------------------------------------------------------------------------------------
%   write geometry aero model for each item    
    if isequal(stick.model.winr, 1)
      export_component_aero(fid, 'geo_aero_fuse.dat', aircraft, stick, geo, pdcylin, 1, MASTER_LABEL.wing);
    end
    if isequal(stick.model.winr, 1)
      export_component_aero(fid, 'geo_aero_wing.dat', aircraft, stick, geo, pdcylin, 2, MASTER_LABEL.wing);
    end
    if isequal(stick.model.vert, 1)
      export_component_aero(fid, 'geo_aero_vtail.dat', aircraft, stick, geo, pdcylin, 3, MASTER_LABEL.vtail);
    end
    if isequal(stick.model.horr, 1)
      export_component_aero(fid, 'geo_aero_htail.dat', aircraft, stick, geo, pdcylin, 4, MASTER_LABEL.htail);
    end
    if isequal(stick.model.canr, 1)
        export_component_aero(fid, 'geo_aero_canard.dat', aircraft, stick, geo, pdcylin, 5, MASTER_LABEL.canard);
    end
    if isequal(stick.model.vert, 1)
      if (isequal(aircraft.Vertical_tail.Twin_tail, 1))
        export_component_aero(fid, 'geo_aero_vtail2.dat', aircraft, stick, geo, pdcylin, 6, MASTER_LABEL.vtail2);
      end
    end
%
    geo.wing.yy = linspace(geo.wing.C2(2,1),geo.wing.C2(2,end),length(geo.wing.rs))';
    geo.wing.xx = interp1(stick.ptos.winrC2(2,:),stick.ptos.winrC2(1,:),geo.wing.yy,'linear');
    geo.wing.zz = interp1(stick.ptos.winrC2(2,:),stick.ptos.winrC2(3,:),geo.wing.yy,'linear');
    
    
    if  isfield(pdcylin,'deformation') && isfield(pdcylin.deformation,'bending')
        for k = 1 : length(pdcylin.deformation.bending.coefficients)
            geo.wing.zz = geo.wing.zz + ...
                 (pdcylin.deformation.bending.coefficients(k)* abs(geo.wing.yy/geo.wing.b*2).^k)*pdcylin.deformation.bending.max_tip_value;
        end
        
    end
    
    
    
    if aircraft.Horizontal_tail.present
        geo.htail.yy = linspace(geo.htail.C2(2,1),geo.htail.C2(2,end),length(geo.htail.rs))';
        geo.htail.xx = interp1(stick.ptos.horrC2(2,:),stick.ptos.horrC2(1,:),geo.htail.yy,'linear');
        geo.htail.zz = interp1(stick.ptos.horrC2(2,:),stick.ptos.horrC2(3,:),geo.htail.yy,'linear');
    end
    
    if aircraft.Canard.present
        geo.canard.yy = linspace(geo.canard.C2(2,1),geo.canard.C2(2,end),length(geo.canard.rs))';
        geo.canard.xx = interp1(stick.ptos.canrC2(2,:),stick.ptos.canrC2(1,:),geo.canard.yy,'linear');
        geo.canard.zz = interp1(stick.ptos.canrC2(2,:),stick.ptos.canrC2(3,:),geo.canard.yy,'linear');
    end
    if aircraft.Vertical_tail.present
        geo.vtail.zz = linspace(geo.vtail.C2(3,1),geo.vtail.C2(3,end),length(geo.vtail.rs))';
        geo.vtail.xx = interp1(stick.ptos.vert(3,:),stick.ptos.vert(1,:),geo.vtail.zz,'linear');
        geo.vtail.yy = interp1(stick.ptos.vert(3,:),stick.ptos.vert(2,:),geo.vtail.zz,'linear');
    end
    
    if ~exist(filename_trim, 'file')
        LoadConditions = [];
    else
        LoadConditions = get_trim_ID(filename_trim);
        fprintf(fid, '\n\n- Total number of maneuvers: %d.', length(LoadConditions));   
    end
    geo_model.aircraft = aircraft;
    geo_model.pdcylin = pdcylin;
    geo_model.geo = geo; 
    geo_model.stick = stick;
    fprintf(fid, '\n\n- Running SMARTCAD...\n');   
    geo_model.beam_model =  load_nastran_model('gstd_model.dat');
    % correct the aero point if deformation is present
    % bending 
%     if  isfield(pdcylin,'deformation') && isfield(pdcylin.deformation,'bending')
%         %aircraft.deformation.bending.type = ''
%         %aircraft.deformation.bending.coefficients = []
%         %aircraft.deformation.bending.max_tip_value = '' 
%         dof = [];
%         for j = 1 : length(geo_model.beam_model.Aero.ID)
%             
%             if (geo_model.beam_model.Aero.ID(j) >=200) && (geo_model.beam_model.Aero.ID(j) <300)
%                 d0 = geo_model.beam_model.Aero.lattice_vlm.DOF(j,1,1);
%                 d1 = geo_model.beam_model.Aero.lattice_vlm.DOF(j,1,2);
%                 dof = [dof,d0:d1];
%                 for k = 1 : length(pdcylin.deformation.bending.coefficients)
%                     geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) = geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                     geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) = geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                     geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) = geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                 end
%             end
%             
%         end
%         % controlo point 
%         for j =1 : length(geo_model.beam_model.Aero.lattice_vlm.Control.Patch)
%            locdof =  geo_model.beam_model.Aero.lattice_vlm.Control.DOF(j).data;
%            ind = intersect(locdof,dof);
%            if length(ind)>0
%                for k = 1 : length(pdcylin.deformation.bending.coefficients)
%                     geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) = geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                     geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) = geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                 end
%            end
%         end
%         
%     
%     if isfield(pdcylin.deformation,'torsion')
%         %aircraft.deformation.bending.type = ''
%         %aircraft.deformation.bending.coefficients = []
%         %aircraft.deformation.bending.max_tip_value = '' 
%         dof = [];
%         for j = 1 : length(geo_model.beam_model.Aero.ID)
%             
%             if (geo_model.beam_model.Aero.ID(j) >=200) && (geo_model.beam_model.Aero.ID(j) <300)
%                 d0 = geo_model.beam_model.Aero.lattice_vlm.DOF(j,1,1);
%                 d1 = geo_model.beam_model.Aero.lattice_vlm.DOF(j,1,2);
%                 dof = [dof,d0:d1];
%                 for k = 1 : length(pdcylin.deformation.bending.coefficients)
%                     
%                     crot = pdcylin.deformation.torsion.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,2)).^k./pdcylin.deformation.torsion.max_tip_value;
%                     if geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0,2)< 0
%                         crot = -crot;
%                     end
%                     
%                     for kk = 1: length(crot)
%                        
%                         rotM = [cos(crot(kk)) , sin(crot(kk)); -sin(crot(kk)), cos(crot(kk))];
%                         loc = geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,[1,3]) - 
%                         
%                     end
%                     
%                     geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) = geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                     geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) = geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                     geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) = geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                 end
%             end
%             
%         end
%         % controlo point 
%         for j =1 : length(geo_model.beam_model.Aero.lattice_vlm.Control.Patch)
%            locdof =  geo_model.beam_model.Aero.lattice_vlm.Control.DOF(j).data;
%            ind = intersect(locdof,dof);
%            if length(ind)>0
%                for k = 1 : length(pdcylin.deformation.bending.coefficients)
%                     geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) = geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                     geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) = geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) + ...
%                          (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,2)).^k)./pdcylin.deformation.bending.max_tip_value;
%                 end
%            end
%         end
%     end
%         
%         
%     % correct the normals
%     step = size(geo_model.beam_model.Aero.lattice_vlm.COLLOC);
% 	[d e f] = size(geo_model.beam_model.Aero.lattice_vlm.VORTEX);
% 	a = e/2;
% 	b = a + 1;
% 	geo_model.beam_model.Aero.lattice_vlm.N = zeros(d, 3);
% 	geo_model.beam_model.Aero.lattice_vlm.DN = zeros(d, 3);
%       
% 	for t=1:step	%Looping through panels
%         
% 		alpha=geo_model.beam_model.Aero.lattice_vlm.S(t);
% 
% 		for s=1:3						%Looping Through Dimensions.
% 			ra(s)=geo_model.beam_model.Aero.lattice_vlm.VORTEX(t,a,s);
% 			rb(s)=geo_model.beam_model.Aero.lattice_vlm.VORTEX(t,b,s);
% 			rc(s)=geo_model.beam_model.Aero.lattice_vlm.COLLOC(t,s);
% 		end
% 		r0=rb-ra;
% 		r0(1)=0;                    %fix to get normals to not point the right way
% 		r1=rc-ra;
% 		r2=rc-rb;
% 		n=cross(r1,r2);				%Passus to determine normal
% 		nl=sqrt(sum((n.^2),2));    %of panel at collocationpoint.
% 		R = n/nl;							%Normalizing normal.
% 		R2 = trot3(r0,R,-alpha);		%rotating wha trot
% 		geo_model.beam_model.Aero.lattice_vlm.N(t,:) = R2';
% 		geo_model.beam_model.Aero.lattice_vlm.DN(t,:) = R2'-R;
%     
%     end
%     end

    % add the bending and the torsion, than compute the interpolation on
    % the aero model.
    if  isfield(pdcylin,'deformation') && isfield(pdcylin.deformation,'bending')
        ss = size(geo_model.beam_model.Node.DOF);
        
        geo_model.beam_model.Node.DOF = reshape([1:ss(1)*ss(2)]',ss(2),ss(1))';
        geo_model.beam_model.Aero.lattice = geo_model.beam_model.Aero.lattice_vlm;
        beam_model = geo_model.beam_model;
        aeroelastic_interface
        geo_model.beam_model = beam_model;
        geo_model.beam_model.Aero.lattice_vlm = geo_model.beam_model.Aero.lattice;
        
        % bending
        SOL = zeros(max(max(geo_model.beam_model.Node.DOF)),1);
        
        
        if  isfield(pdcylin.deformation,'torsion')
           % torsion
           for i = 1:length (geo_model.beam_model.Node.ID)
           
            if (geo_model.beam_model.Node.ID(i)>=2000) && (geo_model.beam_model.Node.ID(i)<3000)
                for k = 1 : length(pdcylin.deformation.torsion.coefficients)
                    SOL(geo_model.beam_model.Node.DOF(i,5)) = SOL(geo_model.beam_model.Node.DOF(i,5)) + ...
                        (pdcylin.deformation.torsion.coefficients(k)* abs(geo_model.beam_model.Node.Coord(i,2)/geo.wing.b*2).^k)*pdcylin.deformation.torsion.max_tip_value;
                end
                
                
            end
            
           end
           
           geo_model.beam_model.Aero.lattice_vlm = update_vlm_mesh1(geo_model.beam_model.Node, SOL, beam_model.Aero);
           
        end
        
        
        
        for i = 1:length (geo_model.beam_model.Node.ID)
           
            if (geo_model.beam_model.Node.ID(i)>=2000) && (geo_model.beam_model.Node.ID(i)<3000)
               
                for k = 1 : length(pdcylin.deformation.bending.coefficients)
                    geo_model.beam_model.Node.Coord(i,3) = geo_model.beam_model.Node.Coord(i,3) + ...
                        (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Node.Coord(i,2)/geo.wing.b*2).^k)*pdcylin.deformation.bending.max_tip_value;
                end
                
            end
            
        end
        dof = [];
        for j = 1 : length(geo_model.beam_model.Aero.ID)
            
            if (geo_model.beam_model.Aero.ID(j) >=200) && (geo_model.beam_model.Aero.ID(j) <300)
                d0 = geo_model.beam_model.Aero.lattice_vlm.DOF(j,1,1);
                d1 = geo_model.beam_model.Aero.lattice_vlm.DOF(j,1,2);
                dof = [dof,d0:d1];
                for k = 1 : length(pdcylin.deformation.bending.coefficients)
                    geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) = geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.COLLOC(d0:d1,2)/geo.wing.b*2).^k)*pdcylin.deformation.bending.max_tip_value;
                    geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) = geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.VORTEX(d0:d1,:,2)/geo.wing.b*2).^k)*pdcylin.deformation.bending.max_tip_value;
                    geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) = geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.XYZ(d0:d1,:,2)/geo.wing.b*2).^k)*pdcylin.deformation.bending.max_tip_value;
                end
            end
            
        end
        
        for j =1 : length(geo_model.beam_model.Aero.lattice_vlm.Control.Patch)
           locdof =  geo_model.beam_model.Aero.lattice_vlm.Control.DOF(j).data;
           ind = intersect(locdof,dof);
           if length(ind)>0
               for k = 1 : length(pdcylin.deformation.bending.coefficients)
                    geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) = geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,1,2)/geo.wing.b*2).^k)*pdcylin.deformation.bending.max_tip_value;
                    geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) = geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,3) + ...
                         (pdcylin.deformation.bending.coefficients(k)* abs(geo_model.beam_model.Aero.lattice_vlm.Control.Hinge(j,2,2)/geo.wing.b*2).^k)*pdcylin.deformation.bending.max_tip_value;
                end
           end
        end
        
        beam_model = geo_model.beam_model;
        
    end
    
    
    geo_model.WB = aircraft.weight_balance.COG;
    geo_model.LoadConditions = LoadConditions;
    fprintf(fid, 'done.'); 
    if inBatch
        uiwait(ChEcK)
    else
        geo_model.run = 1;
        geo_model.pdcylin.MassConf = [];
        if isempty(NMAX)
            NMAX = 25; % handles.edit1 = edit_nmax
        end
        if isempty(EPS)
            EPS = 0.001; % handles.edit1 = edit_eps
        end

        geo_model.run = 1;
        geo_model.aircraft.Horizontal_tail.Allmovable = 0;
        geo_model.aircraft.Vertical_tail.Allmovable = 0;
        geo_model.aircraft.Canard.Allmovable = 0;
    end
    if isfield(geo_model.pdcylin,'MassConf') && isempty(geo_model.pdcylin.MassConf)
        geo_model.pdcylin.MassConf.Pass = 1;
        geo_model.pdcylin.MassConf.Baggage = 1;
        geo_model.pdcylin.MassConf.Cfuel = 1;
        geo_model.pdcylin.MassConf.Wfuel = 1;
%
        geo_model.pdcylin.MassConf.WfuelStart = 0;
        geo_model.pdcylin.MassConf.WfuelArrive = geo_model.aircraft.fuel.Outboard_fuel_tank_span;
        geo_model.pdcylin.MassConf.WfuelStart = geo_model.geo.fus.R/geo_model.geo.wing.b*2;
%
        geo_model.pdcylin.MassConf.BagArrive = 1;
        geo_model.pdcylin.MassConf.BagStart = 0;
        geo_model.pdcylin.MassConf.PaxArrive = 1;
        geo_model.pdcylin.MassConf.PaxStart = 0;
%
        geo_model.pdcylin.MassConf.Load.data = LoadConditions;
    end
    aircraft = geo_model.aircraft;    
    pdcylin = geo_model.pdcylin;          
    go = geo_model.run;
end
end
%-------------------------------------------------------------------------------
function export_component_aero(fid, filename, aircraft, stick, geo, pdcylin, ITEM_N, CONTR_NAME)
%
  fid2 = fopen(filename, 'w');
  TrimID = cnvt2_8chs(ITEM_N);
  writeAEROS2file(fid2, stick, aircraft);
  fprintf(fid2,'TRIM=   ');
  fprintf(fid2, '%c', TrimID);
  fprintf(fid2,'\nTRIM    %s1       0.3     0.0     SIDES   0.0     ROLL    0.0     ', TrimID);
  fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     URDD3   9.81    ');
  fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     ANGLEA  0.0     \n        ');
  if (ITEM_N>1)
    ncsm = length(CONTR_NAME);
    for (n=1:ncsm)
      [LABLIstr] = cnvt2_8chs(CONTR_NAME{n});
      fprintf(fid2, '%c', LABLIstr); fprintf(fid2, '0.0     '); 
      if (n==4 && ncsm>4) 
        fprintf(fid2,'\n        ');
      end
    end
    [stick] = writeCAERO2file(0, fid2, aircraft, stick, geo, ITEM_N, pdcylin.smartcad.caerob);
    MASTER_LABEL = writeAELINK2file(0, fid2, pdcylin, geo, stick, aircraft, ITEM_N);
  else
    fprintf(fid2,'\n$ Body frame\n');
    P1 = [geo.fus.xx(3); 0; geo.fus.zz(3)]; % origin
    P3 = [geo.fus.xx(4); 0; geo.fus.zz(4)]-P1; % x axis
    P2 = [0; 1; 0]; % y axis
    P2 = crossm(P3)* P2; % z axis
%   export body reference frame
    BULKdataCORD2R(fid2, stick.IDSET.fuse, 0, zeros(3,1), P2, P3);
    P1 = [geo.fus.xx(1); 0; geo.fus.zz(1)] - ...
         [geo.fus.xx(3); 0; geo.fus.zz(3)];
    P2 = [geo.fus.xx(4); 0; geo.fus.zz(4)] - ...
         [geo.fus.xx(3); 0; geo.fus.zz(3)];
    P2 = P2 ./ norm(P2);
    dl = dot(P1, P2);
%     body projected nose
    AP1 = P2.*dl + [geo.fus.xx(3); 0; geo.fus.zz(3)]; 
    P1 = [geo.fus.xx(end); 0; geo.fus.zz(end)] - ...
         [geo.fus.xx(3); 0; geo.fus.zz(3)];
    dl = dot(P1, P2);
    AP2  = P2.*dl + [geo.fus.xx(3); 0; geo.fus.zz(3)]; 
%     projected body length
    L = norm(AP2-AP1);
%     number of elements
    NE = length(geo.fus.thick_dom);
    BULKdataCAEROB(fid2, stick.IDSET.fuse, AP1, stick.IDSET.fuse, L, NE, ...
                   stick.IDSET.fuse, geo.fus.thick_dom./L, geo.fus.thick_ver);
  end
  fclose(fid2);
end
%-------------------------------------------------------------------------------
function print_master_controls(fid, NAME, LHEAD, SURF, ROT, offset)
  ncsm = length(NAME); 
  count = 0;
  nc = length(LHEAD); index = [];
  for (i=1:nc)
    for (j=1:ncsm)
      [LABLIstr] = strtok(cnvt2_8chs(NAME{j}));
      if (length(LABLIstr)>length(SURF{i}))
        if (strcmp(LABLIstr(1:LHEAD(i)), SURF{i})==1)
          index = [index, j];
        end
      end
    end
  end
  index = setdiff([1:ncsm], index);
  nc = length(index);
  for (n=1:nc)
    [LABLIstr] = cnvt2_8chs(NAME{index(n)});
    fprintf(fid, '%c', LABLIstr); 
    [defl] = cnvt2_8chs(ROT(index(n)));
    fprintf(fid, '%c', defl);
    count = count+1;
    if ( (mod(count-offset,4)==0) && (n~=nc))
      fprintf(fid,'\n        ');
    end
  end
end
%-------------------------------------------------------------------------------
function ID = get_trim_ID(filename)
  fp = fopen(filename, 'r');
  ntrim = 0;
  ID = [];
  skip_line = false;
    while ~feof(fp)
      if ~skip_line
        tline = fgetl(fp);
      else
        skip_line = false;
      end
      CARD = strtok(tline);
      switch CARD
        case 'TRIM' % TRIM card for static aeroelasticity
          ntrim = ntrim +1;
          ID(ntrim)  = int32(num_field_parser(tline, 2));
          skip_line = false;
      end
    end
  fclose(fp);
end
%
function num = num_field_parser(line, index)
  FIELD = 8;
  if length(line) < FIELD * (index-1)
    num = 0;
  else
    minc = min(length(line), index * FIELD);
    field = strtok(line((index-1) * FIELD+1:minc));
    if ~isempty(field)
        num = str2num(field);
    else
        num = 0;
    end
  end
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