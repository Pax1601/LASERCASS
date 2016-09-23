Files included:

XplaneL_neo.dat: NEORESP main file
XplaneLCONM_CONF3.inc: aeroelastic model
ControlSolverParam.inc: settings for control response
dyn_model_res.mat: MATLAB bynary file with results from NEORESP

elev_profile.png: input deflection for elevator
plunge.png: plunge displacement
pitch.png: pitch deflection

Steps:

1) run the preprocessor: init_dyn_model('XplaneL_neo.dat')
2) save the database:

global dyn_model;
save('XplaneL_neo_neoresp.mat', 'dyn_model');

3) run the simulation:
solve_free_lin_dyn('Tmax',5,'dT',5e-3)

