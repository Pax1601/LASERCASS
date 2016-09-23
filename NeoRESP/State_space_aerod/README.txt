================================================================================

          Improved Rational Matrix Fraction Approximation (RMFA) 

================================================================================

Type  main  on Matlab command window to start the program, and follows the 
instructions.

The identification process begins by approximating the aerodynamic transfer 
matrix with a Matrix Fraction Description (MFD).
Aerodynamic data must be saved in a  *.mat  format file.
The aerodynamic transfer matrix, along the related reduced frequecies, must 
be available as *.mat formatted files, 
named respectively as Ha and k, stored as follows:

k  = vector having dimensions (1 x nk) 
Ha = [Ham Hag] matrix having dimensions (ny x nu x nk), nk = nq + ng,

being nk: number of reduced frequencies
      ny: number of generalized aerodynamic forces
      nu: number of imposed boundary conditions due to motion and gust, if any
      nq: number of free motion coordinates
      ng: number of gusts.
      
The user can select between a left MFD (LMFD) and a right MFD (RMFD), 
i.e. respectively approximating the aerodynamic transfer matrix as Ha = D\N or 
Ha = N/D, D and N being two polynomial matriced defined as: 
         D = D0 + p*D1 + ... + p^(n-1)*D(n-1) + p^n*I
         N = N0 + p*N1 + ... + p^n*Nn + p^(n+1)*N(n+1) + p^(n+ro)*N(n+ro).
and p the harmonic reduced frequency, p = jk.
The paramater ro  (ro=1 or ro=2) is used in defining the roll off behavior 
(1/p^ro) of the filters, obtained using a RMFD, representing discrete gusts, 
see Ref.      

The minimization of the true fit error || Ha - D\N || or || Ha - N/D ||
is achieved by solving a nonlinear least squares (NLS) problem
with a Levenberg-Marquard algorithm.   
The stardad nonlinear LS form is obtained by reformulating the MFD 
using three different methods: 
(1): vectorizing the problem exploiting the Kronecker identity 
     vec( A*X*B ) = kron( B^T, A ) * vec(X). 
(2): exploitng the relation A^-1 = adj(A)/det(A), and so ending with a inverse 
     determinant-weighted nonlinear least squares problem
(3): linearizing the problem so ending up with a linearized incremental form.
The nonlinear LS problem is initialized using a linear LS solution or, if 
available, the matrices of a previous solution.       
       
In the following it should be remarked that the default values will be 
available for the graphic interface only, none being available when the 
related data are given in the input file.

The identification options must be written in a cell array named opt.
The different options available are the following:

opt{1} = order n of the MFD, such that: 
         D = D0 + p*D1 + ... + p^(n-1)*D(n-1) + p^n*I
         N = N0 + p*N1 + ... + p^n*Nn + p^(n+1)*N(n+1) + p^(n+ro)*N(n+ro); 
	 (default 1)

opt{2} = selection of the MFD solution method:
         (1), ('LS1') or ('kron'): Kronecker-based nonlinear least squares
         (2), ('LS2') or ('det'): inverse determinant-weighted form
         (3), ('LS3') or ('lin'): linearized incremental form
	 (default 'det')

opt{3} = select between a left and a right MFD
         ('lmfd'), ('LMFD') or ('left'): left MFD, H = D\N 
         ('rmfd'), ('RMFD') or ('right'): right MFD, H = N/D; 
	 (default 'left')

opt{4} = parameter  ro  used in treating discrete gusts with a RMFD filter. 
         NOTE: if a LMFD is choosen  ro  is set automatically to ro = 2, 
         and opt{4} may be left empty (opt{4} = []);
	 (default 2)

opt{5} = weight applied at the second reduced frequency during the final 
	 linear least square fitting, to enforce the correct low frequency 
	 beahaviour up to the second derivative.      
	 (default 100)
      
The options of the Levenberg-Marquard algorithm, used in solving the nonlinear 
least squares problem, must be stored in a vector named  optsLM, so defined:

optsLM(1) = parameter (tau) used in defining the starting value for the 
            Levenberg-Marquardt damping parameter (lambda) as 
            lambda = tau*max(Jacobian'*Jacobian);
	    (default 1e-3)
             
optsLM(2) = tolerance on the solution gradient, below which the algorithm stops;
	    (default 1e-4)

optsLM(3) = tolerance on the solution variation, below which the algorithm 
	    stops; 
	    (default 1e-6)

optsLM(4) = maximum number of iterations.
	    (default 100)

Enforcement of MFD stability is achieved using minimal eigensolution changes or 
through a poles placement technique.
Both methods check the eigensolutions stability, threshold, at each iteration 
of the MFD identification. If any eigenvalue is below an assigned stability 
threshold value it is shifted to a user given admissible real 
negative eigenvalue. Then either the related eigenvectors are 
 modified so to mantain the original structure, eigshift, of the state matrix 
or a pole placement,a polesplace, of the newly stabilized eigenspectrum is 
applied. 

Eigenvalues shifting may be achieved either by defining a value  eigsopt.bound 
to which the real part of the unstable eigenvalues is placed ('bound' method), 
or by flipping the real part about the imaginary axis ('flip' method). 

The imaginary part of any unstable eigenvalues is set to zero and, in order to 
avoid generating coincident poles, the  bound  value is added 
cumulatively to the just found unstable eigenvalues.

The stabilization parameters must be stored in a structure array named eigsopt
with the following fields:

eigsopt.threshold = threshold value for the eigenvalues real part below which 
                    they are considered unstable and therefore shifted  
	            (default -1e-4)

eigsopt.method    =  used to recover the stable state matrix:
                    'eigshift': eigenvalues shift and eigenvector correction, 
                    'polesplace': poles placement using a state feedback matrix
	            (default 'eigshift')

eigsopt.type      = define the method used to shift the unstable eigenvalues:
                    'flip': flipping about the imaginary axis,
                    'bound': real part of unstable eigenvalues are setted to 
                              the user defined value  eigsopt.bound
	            (default 'bound')

eigsopt.bound     = value to which the real part of any unstable eigenvalue is 
                    setted when  'bound'  option is selected in  eigsopt.type
	            (default -1e-2)

The user can select among different model order reduction techniques.
The choosen algorithm must be defined as a string variable and stored in 
algROM.

algROM = selection among the available model reduction algorithms
         'balance': balanced model truncation via square root method 
         (>> help balancmr, the default)
         'hankel': Hankel minimum degree approximation (MDA) without balancing 
         (>> help hankelmr)
         'schur': balanced model truncation via Schur method                    
         (>> help schurmr)
         'bst': balanced stochastic model truncation (BST) via Schur method   
         (>> help bstmr)

Whatever the choice of the order reduction, the final reduced state space 
system will be selected as the best (in term of a defined norm) between a 
direct truncation, a static residualization, and a first order residualization, 
all followed by a final refitting of the input matrix (B), the output matrix (C)
and the direct matrices (D). In performing such a final fitting the state 
equation residual is extended to third order.
Low frequency equality constraint will be enforced during this very last 
final touch, through the mentioned weighting of the second reduced frequency 
fit.
      
The example folder contains a couple of tests about:
- how to create an option file for the identification program
- input data of a two degrees of freedom airfoil in incompressible flow
- input data of a four/six degrees of freedom AGARD 445.6 wing at Mach 0.678

Ref: Ripepi M., Mantegazza P., 'An improved matrix fraction approximation of 
aerodynamic transfer matrices', AIAA journal, submitted for publication.  
    
   Copyright (C) 2012 Paolo Mantegazza   <mantegazza@aero.polimi.it>
   Copyright (C) 2012 Matteo Ripepi      <ripepi@aero.polimi.it>
  
================================================================================

