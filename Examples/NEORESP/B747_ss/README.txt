Files included:

B747_ss.dat: NeoRESP main file
B747-100_MTOW.inc: SMARTCAD aeroelastic model
B747-100_MTOW_CONM.inc: mass configuration
GustSolverParam.inc: settings for gust response

Steps:

1) launch NeoRESP preprocessor: 
   init_dyn_model('B747_ss.dat')

2) save the database:
   global dyn_model;
   save('B747_ss.mat', 'dyn_model');

3) launch Neoresp for state space analysis: solve_free_lin_dyn_ss
Two files with generalized forces will be created, one for Qam due to motion, 
one for Qag due to gust, B747_ss_Ham_M_0.7.mat and B747_ss_Hag_M_0.7.mat.

4) open the script aero_ss.m and look at the parameters used for fitting.
   In this case for Qam we have:
    opt{1} = 2;      % MFD order
    opt{2} = 3;      % MFD algorithm
    opt{3} = 'lmfd'; % left or right MFD
    opt{4} = 2;      % residualization order
    opt{5} = 100;    % weight parameter value W^2

   while for Hag:
    opt{1} = 6;      % MFD order
    opt{2} = 3;      % MFD algorithm
    opt{3} = 'rmfd'; % left or right MFD
    opt{4} = 2;      % residualization order
    opt{5} = 100;    % weight parameter value W^2

    Run the script for Ham:
      aero_ss('B747_ss_Ham_M_0.7.mat','res1.mat')
    Keep all the Hankel singular values and
    check for the quality of the interpolation by looking into the interpolated terms.

    Comment the parameters for Ham and uncomment those for gust.
    Run the script for Hag:
      aero_ss('B747_ss_Hag_M_0.7.mat','res2.mat')
    Keep all the Hankel singular values and
    check for the quality of the interpolation by looking into the interpolated terms.

5) Launch again the solver, providing input and outputs:
     [ss_model,Y,T,X,U] = solve_free_lin_dyn_ss('Tmax',15,'dT',0.0001,'Ham','res1.mat','Hag','res2.mat');
   The response will refer to a time-window of 15 sec, sampled at 1e-4 secs.

6)  Outputs are now available. 
    For example the vertical translation can be plotted
      plot(T,Y(:,1))
    or similarly the first elastic mode
      plot(T,Y(:,3))

    ss_model.OutputGroup.bar_forces has the indeces to recover bar forces.
    Each bar has 12 forces, 6 for each collocation point.
