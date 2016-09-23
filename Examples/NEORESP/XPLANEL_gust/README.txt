Files included:

XplaneL_neo.dat: NEORESP main file
XplaneLCONM_CONF3.inc: aeroelastic model
GustSolverParam.inc: settings for gust response
dyn_model_res.mat: MATLAB bynary file with results from NEORESP

XplaneL_neo_damp.dat: same as XplaneL_neo.dat, but structural damping included

Steps:

1) run the preprocessor: init_dyn_model('XplaneL_neo.dat')
2) save the database:

global dyn_model;
save('XplaneL_neo_neoresp.mat', 'dyn_model');

3) run the simulation:
solve_free_lin_dyn('Tmax',5,'dT',5e-3)

in this case it may take a while because of the reduced size of the time step and 
acceleration modes are ON (PARAM MODACC 0). To disable MODACC modify into PARAM MODACC -1
or delete the command line.
The response will refer to a time-window of 5 sec, sampled at 5e-3 secs.