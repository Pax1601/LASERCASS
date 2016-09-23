The following steps allow to create a set of files to be used in
NASTRAN SOL 200 for structural optimization of the wing box.

1) run guess for sizing
2) run wing_fem.m function. 

This function takes as inputs:
a) the main NASTRAN filename to create which will include several .dat files 
   necessary for the run;
b) xml file with aircraft definition, used to have wing box geometry.
This function has an on-line input section to provide all the required parameters, i.e.
spar position, number and type of ribs, stringers pitch and type, number of mesh elements, material.

3) once the main file is available, the user must run load_ribs_CONM function,
which introduces in the model a set of extra grids (with ID from 2000 to 3000) directly taken from smartcad stick model.
These nodes are linked to the wing-box through RBE3 elements at rib stations and carry lumped masses CONM1.
All these data are export in ns_mass.dat file.
The lumped masses exported are extracted from SMARTCAD model of the semi-wing and represent:
a) non-structural mass 
b) wing tank fuel and central tank (if located at the central node 2000)
This means the user can run this function for all the mass configurations of interest.

4) Finally, the user runs SMARTCAD trim solver:

- solve_free_lin_trim (runs static trim for rigid and deformable beam model)
- solve_vlm_rigid (runs simple aerodynamic solution)
- solve_free_lin_trim_guess_std (runs static trim for rigid only 
  model and CONM2 representing aircraft mass, load gstd_model.dat file)

and exports aerodynamic wing loads using neo2rbeforce_beam2.
This function creates a set of FORCES and MOMENTS to be used during the optimization process.
The user needs to create several load set and include them as SUBSET in the main NASTRAN file.
Use ns_mass.dat file to have a list of the structural grid data to use (from ID 2000)
Provide their ID as input str_id input and their coordinates as str_data input.
ORIGIN and END depends if the axis is almost linear or has kinks.
In this case, use the function for each segment and create a serie of files (with the same ID LOAD) 
to include in the analysis.


For example:
%
dummy_model = load_nastran_model('ns_mass.dat');
%
plot3(dummy_model.Node.Coord(:,1),dummy_model.Node.Coord(:,2),dummy_model.Node.Coord(:,3),'-o')
axis equal
%
% check how many segments can be used. if the axis is pretty linear, one segment is enough.
%
The IDs are: dummy_model.Node.ID

Use:
 
neo2rbeforce_beam2(beam_model,'output.dat',dummy_model.Node.ID,dummy_model.Node.Coord,...
 [200:205],0,dummy_model.Node.Coord(1,:),dummy_model.Node.Coord(end,:),1,1);

to export loads for 200,201,202,203,204,205 aero patched as LOAD 1 for Nastran.
The last parameter given (1) allows to export accelerations for inertial loads.
The spline goes from the first point in the set to the last one.



