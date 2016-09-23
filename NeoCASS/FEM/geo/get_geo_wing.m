
function [foil, PANE, caero1, aircraft, geo] =  get_geo_wing(filename_geo)

fid = 1;
fprintf(fid,'\n\n- WING GEOMETRY');
aircraft = neocass_xmlwrapper(filename_geo);
SIGMA_TAU = sqrt(3);
%
% material data
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
    return;
end
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
%
aircraft.Wing1.longitudinal_location = aircraft.Wing1.x;
aircraft.Wing1.vertical_location     = aircraft.Wing1.z;
winglet = aircraft.Wing1.winglet;
pdcylin = setup_tech_conversion(pdcylin,aircraft);
pdcylin = setup_sma_defaults(pdcylin);
aircraft = setup_geofile_conversion(aircraft); 
aircraft.winglet = winglet;
geo.fus     = [];     % Geometry for FUSELAGE
geo.wing    = [];     % Geometry for WING
% GEOMETRY MODULUS
fprintf(fid, '\n\tRunning geo module for fuselage and wing...');
geo = Geo_Fus(pdcylin, aircraft, geo);
[geo, pdcylin] = Geo_Wing_FEMGEN(pdcylin, aircraft, geo);
%
geo.wing.PANE(2,:) = geo.wing.PANE(2,:) + geo.fus.R;
PANE = [geo.wing.PANE(:,1), geo.wing.PANE(:,1), geo.wing.PANE(:,4), geo.wing.PANE(:,4)];
PANE(2,1) = 0.0;
PANE(2,4) = 0.0;
geo.wing.PANE = [PANE, geo.wing.PANE];
%
geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; geo.wing.CAERO1.dihedral(end)];
%geo.wing.CAERO1.twist = [geo.wing.CAERO1.twist, geo.wing.CAERO1.twist(end)];

nc = size(geo.wing.CAERO1.sup_control.nme,1);
caero1.CFRAC(:,1) = [0; geo.wing.CAERO1.sup_control.frc(1:2:2*nc)];
caero1.CFRAC(:,2) = [0; geo.wing.CAERO1.sup_control.frc(2:2:2*nc)];
caero1.SFRAC = [1; geo.wing.CAERO1.sup_control.frs];
caero1.NAME = ['        '; geo.wing.CAERO1.sup_control.nme(1:nc,:)];
caero1.CPOS = geo.wing.CAERO1.sup_control.position;
%
%
%
% determine patch reference frame
%
naer = length(geo.wing.CAERO1.dihedral);
foil = {};
for k=1:naer
  offset = (k-1)*4 + 1;
  x = geo.wing.PANE(:,offset+2) - geo.wing.PANE(:,offset+1);
  c = norm(x);
  CENTER = c/4;
  x(1) = cos(geo.wing.CAERO1.twist(k));
  x(2) = 0;
  x(3) = -sin(geo.wing.CAERO1.twist(k));
  y = zeros(3,1);
  dihed = D2R(geo.wing.CAERO1.dihedral(k));
  y(2) = -sin(dihed);  
  y(3) = cos(dihed);  
  z = crossm(x) * y;
  z = z ./ norm(z);
  % rotation matrix
  y = crossm(z) * x;
  y = y ./ norm(y);
  %
  R = [x, y, z];
  %
  [NODES_U, NODES_L] = load_airfoil(cell2mat(geo.wing.CAERO1.airfoil(k)));
  %
  np = length(NODES_U);
  NODES_U = [NODES_U, zeros(np,1)];
  NODES_L = [NODES_L, zeros(np,1)];
  NODES_U(:,1) = NODES_U(:,1) -.25;
  NODES_L(:,1) = NODES_L(:,1) -.25;
  for j=1:np
    NODES_U(j,:) = (R * (c .* NODES_U(j,:)')+geo.wing.PANE(:,offset+1))'; 
    NODES_L(j,:) = (R * (c .* NODES_L(j,:)')+geo.wing.PANE(:,offset+1))'; 
  end
  NODES_U(:,1) = NODES_U(:,1) + c/4;
  NODES_L(:,1) = NODES_L(:,1) + c/4;
%
  foil{k+1} = [NODES_L(end:-1:1,:); NODES_U];
end
% add root foil
foil{1} = foil{2};
foil{1}(:,2) = 0;
%
PANE = geo.wing.PANE;

fprintf(fid, '\n\tdone.\n');
%
if aircraft.winglet.present>0
  winglet = 1;
else
  winglet = 0;
end
fprintf(fid, '\n\tTotal number of aerodynamic sections: %d.', naer);
fprintf(fid, '\n\t\tCarry through: 1.');
fprintf(fid, '\n\t\tWing: %d.', naer -1 - winglet);
fprintf(fid, '\n\t\tWinglet: %d.', winglet);
%save('wing_fem_geo.mat','geo');
%
end
