% Read FEMGEN input file and create new NASTRAN FILEs
% Inputs
% input_file created at the first FEMGEN run
% outfilename: output main filename
%
function read_input_file(input_file, outfilename)
close all;
dotpos = find(input_file=='.');
if ~isempty(dotpos)
  input_file = input_file(1:dotpos(end)-1);
end
eval(input_file);

param.fore_spar = fore_spar;
param.aft_spar  = aft_spar;
param.nrib_span = nrib_span;
param.rib_thick = rib_thick;
param.nelem_btwn_rib = nelem_btwn_rib;
param.nstr_up_start=nstr_up_start;
param.nstr_low_start=nstr_low_start;
param.nelem_btwn_str = nelem_btwn_str;
param.nelem_fore_spar = nelem_fore_spar;
param.nelem_aft_spar=nelem_aft_spar;
param.nelem_rib=nelem_rib;
param.rib_type=rib_type;
param.str_up_pitch=str_up_pitch;
param.str_low_pitch=str_low_pitch;
param.str_type=str_type;
if (strcmp(str_type, 'var'))
  param.nstr_up_start = 0;
  param.nstr_low_start = 0;
else
  param.str_up_pitch=0;
  param.str_low_pitch=0;
end
param.mat_type=mat_type;
if param.mat_type>2 || param.mat_type<1
  error('Undefined material type. Allowed values are 1 for Aluminum and 2 for Carbon Fiber.');
end
param.foil=foil;
%
guess_model = [];
if ~isempty(guess_filename)
  load(guess_filename)
end
figure(1); close;
open('wing_geo.fig'); view(2); axis equal; hold on;
ribs_data = femgen(outfilename, param, guess_model, X0, section);
load caero.mat
aero_model(bkcaero.PANE, bkcaero.CAERO1.CFRAC, bkcaero.CAERO1.SFRAC, bkcaero.CAERO1.CPOS, bkcaero.CAERO1.NAME, ribs_data,param.nrib_span,aero_mesh);
H = figure(1);
saveas(H,'wing_fem.fig','fig');
