function export_input_file(param, inp, filename)
%
fp = fopen(filename,'w');
fprintf(fp, '%% FEMGEN INPUT FILE\n%%\n');
% spars
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% SPARS\n');
fprintf(fp, '%% Spar chord percentage at control sections\n');
print_param(fp, 'fore_spar',  param.fore_spar, '%f ');
print_param(fp, 'aft_spar',   param.aft_spar,  '%f ');
% ribs geo
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% RIBS\n');
fprintf(fp, '%% Ribs type (''wind'' for ribs along freestream or ''axis'' for ribs orthogonal to wing axis)\n');
fprintf(fp, ['rib_type = ''', param.rib_type,''';\n']);
fprintf(fp, '%% Number of ribs within for each trunk delimited by two control sections\n');
print_param(fp, 'nrib_span',  param.nrib_span, '%d ');
fprintf(fp, '%% Rib thickness [meters] \n');
print_param(fp, 'rib_thick', param.rib_thick,'%f');
% stringers
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% STRINGERS\n');
fprintf(fp, '%% Stringers type (''fix'' for constant spanwise value or ''var'' for variable spanwise variable)\n');
fprintf(fp, ['str_type = ''', param.str_type,''';\n']);
fprintf(fp, '%% Number of stringers at upper/lower section (in case str_type = ''fix'')\n');
print_param(fp, 'nstr_up_start', param.nstr_up_start,'%d');
print_param(fp, 'nstr_low_start', param.nstr_low_start,'%d');
fprintf(fp, '%% Upper/lower stringers pitch (in case str_type = ''var'')\n');
print_param(fp, 'str_up_pitch', param.str_up_pitch,'%f');
print_param(fp, 'str_low_pitch', param.str_low_pitch,'%f');
% meshing
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% MESH MAIN PARAMETERS\n');
fprintf(fp, '%% Number of elements between ribs\n');
print_param(fp, 'nelem_btwn_rib', param.nelem_btwn_rib,'%d');
fprintf(fp, '%% Number of elements between stringers\n');
print_param(fp, 'nelem_btwn_str', param.nelem_btwn_str,'%d');
fprintf(fp, '%% Number of elements along front spar\n');
print_param(fp, 'nelem_fore_spar', param.nelem_fore_spar,'%d');
fprintf(fp, '%% Number of elements along rear spar\n');
print_param(fp, 'nelem_aft_spar', param.nelem_aft_spar,'%d');
fprintf(fp, '%% Number of radial elements on ribs\n');
print_param(fp, 'nelem_rib', param.nelem_rib,'%d');
% material
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% MATERIAL\n');
fprintf(fp, '%% Material type (1 for Aluminum, 2 for Carbon Fiber)\n');
print_param(fp, 'mat_type', param.mat_type,'%d');
% GUESS
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% INITIAL SOLUTION\n');
fprintf(fp, '%% GUESS filename\n');
fprintf(fp, ['guess_filename = ''', inp.guess_filename,''';\n']);
fprintf(fp, '%% Value for skin thickness, spar thickness and stringer area [meters, meters, meters^2]\n%% (if guess_filename is empty)\n');
print_param(fp, 'X0', inp.X0,'%f ');
%
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% AERO MESH PARAMETERS\n');
fprintf(fp, '%% For each trunk provide number of spanwise panel, chordwise panels,\n');
fprintf(fp, '%% control section panels (if present), secondary spanwise and chordwise numnber of panels (if control span is lower than aero patch span)\n');
%
naer = size(inp.aero_mesh,1);
for i=1:naer
  print_param(fp, ['aero_mesh(', num2str(i),',:)'], inp.aero_mesh(i,:)','%d ');
end
%
fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% PROPERTIES LINKS FOR SOL 200\n');
fprintf(fp, '%% A property is assigned for each bay delimited by two ribs (starting from zero)\n');
fprintf(fp, '%% The total number of properties for each set is given by nrib_span +1\n');
fprintf(fp, '%% Link properties within each trunk delimited by two control sections\n');
fprintf(fp, '%% At least %d links must appear\n', length(param.nrib_span));
%
nbay = length(inp.section);
for i=1:nbay
  nlinks = length(inp.section(i).prop);
  if nlinks == 1
    print_param(fp, ['section(', num2str(i),').prop'], inp.section(i).prop','%d');
  else
    stype = repmat('%d ', 1, nlinks);
    print_param(fp, ['section(', num2str(i),').prop'], inp.section(i).prop',stype);
  end
end




fprintf(fp, '%% ----------------------------------------------------------------------------------------------\n');
fprintf(fp, '%% AIRFOILS COORDINATES AT CONTROL SECTIONS\n');
nfoil = length(param.foil);
for i=1:nfoil
  print_param(fp, ['foil{', num2str(i),'}'], param.foil{i}','%f %f %f \n');
end
%

fclose(fp);

end

function print_param(fp, name, value, type)
  if length(value)>1
    fprintf(fp,[name, ' = [ ']);
    fprintf(fp, type, value);
    fprintf(fp,'];\n');
  else
    fprintf(fp,[name, ' = ']);
    fprintf(fp, type, value);
    fprintf(fp,';\n');
  end
end