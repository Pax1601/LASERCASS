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
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
%   Author: <andreadr@kth.se>
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
%*******************************************************************************
function [pdcylin, geo, str , aircraft] = AFaWWE_SET(fid, pdcylin, aircraft)

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
geo.tbooms  = [];     % Geometry for TBOOMS
%
% loads.fus    = [];     % Load for FUSELAGE
% loads.wing   = [];     % Load for WING
% loads.vtail  = [];     % Load for VERTICAL TAIL
% loads.htail  = [];     % Load for HORIZONTAL TAIL
% loads.wing2  = [];
% loads.canard = [];
% %
% str.fus     = [];     % Structure for FUSELAGE
% str.wing    = [];     % Structure for WING
% str.vtail   = [];     % Structure for VERTICAL TAIL
% str.htail   = [];     % Structure for HORIZONTAL TAIL
str.M       = 0;      % scalar, to determine the mass computed and written in the *.dat file [kg]

%
%--------------------------------------------------------------------------
% User-input definition of engines attachement in case they are originally
% defined as "floating" propulsion pods.
%--------------------------------------------------------------------------
%
aircraft = user_input_engines_attach(fid, aircraft);

%
%--------------------------------------------------------------------------
% GEOMETRY MODULUS
%--------------------------------------------------------------------------
%
fprintf(fid, '\n\tCreating baseline geometry...');
% Fuselage
geo   = Geo_Fus(pdcylin, aircraft, geo);
str.M = str.M + aircraft.weight_balance.COG(5,4,1);

% Wing
[geo, pdcylin] = Geo_Wing(pdcylin, aircraft, geo);
str.M = str.M + aircraft.weight_balance.COG(1,4,1);

% Tbooms
if isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present  
    geo = Geo_Tbooms(pdcylin, aircraft, geo);
    str.M = str.M + aircraft.weight_balance.COG(12,4,1); %
end

% VT
if aircraft.Vertical_tail.present
    [geo, pdcylin] = Geo_Vtail(pdcylin, aircraft, geo);
    str.M = str.M + aircraft.weight_balance.COG(4,4,1);
end

% HT
if aircraft.Horizontal_tail.present
    [geo, pdcylin] = Geo_Htail(pdcylin, aircraft, geo);    
    str.M = str.M + aircraft.weight_balance.COG(3,4,1);
end

% Canard
if aircraft.Canard.present
    [geo, pdcylin] = Geo_Canard(pdcylin, aircraft, geo);
    str.M = str.M + aircraft.weight_balance.COG(11,4,1);
end

% Wing2
if aircraft.wing2.present
    [geo, pdcylin] = Geo_Wing2(pdcylin, aircraft, geo);
    geo = Wing_thick(geo, aircraft.weight_balance.COG(2,4,1)/pdcylin.wing.dsw);%?
    str.M = str.M + aircraft.weight_balance.COG(2,4,1); %
else
    pdcylin.stick.model.win2r = 0;
end
fprintf(fid, 'done.');

%**********************************************************************************************************************
% Introduce the carrythrough structure in wing and horizontal formulation. Until now parameters have been defined only
% for the exposed surface (i.e. from wing/fuse and HT/VT or HT/fuse intersection). Now the following parameters have
% been updated to consider the structure within other component.
%**********************************************************************************************************************

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
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme(1:length(geo.wing.CAERO1.n),:);...
            '    none';...
            geo.wing.CAERO1.sup_control.nme(length(geo.wing.CAERO1.n)+1:end,:)];
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
    if nrcth>0
      dycth = norm(geo.canard.QC(:,2)-geo.canard.QC(:,1)) / nrcth;
      ycth = (0: dycth : nrcth*dycth)';
      geo.canard.y = [ycth; ycth(end)+geo.canard.y(2:end)];
    else
      dycth = norm(geo.canard.QC(:,2)-geo.canard.QC(:,1));
      geo.canard.y = geo.canard.y  + dycth;
    end
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
