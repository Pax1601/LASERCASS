Files included:

XplaneL_neo.dat: NEORESP main file
XplaneLCONM_CONF3.inc: aeroelastic model
ForseSolverParam.inc: settings for external force response
dyn_model_res.mat: MATLAB bynary file with results from NEORESP

Steps:

1) run the preprocessor: init_dyn_model('XplaneL_neo.dat')
2) save the database:

global dyn_model;
save('XplaneL_neo_neoresp.mat', 'dyn_model');

3) run the simulation:
solve_free_lin_dyn('Tmax',5,'dT',5e-3)

The response will refer to a time-window of 5 sec, sampled at 5e-3 secs.