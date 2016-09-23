% =========================================================================
%
%           Example script showing how to create an option file     
%
% =========================================================================

% Identification options:
% -------------------------------------------------------------------------
% opt{1}  = MFD order n, such that: 
%           D = D0 + p*D1 + ... + p^(n+1)*D(n-1) + p^n*I;
%           N = N0 + p*N1 +          ...         + p^n*Nn + p^(n+1)*N(n+1) + p^(n+2)*N(n+2); 
%
% opt{2}  = MFD algorithm:
%           (1) Kronecker-based nonlinear least squares
%           (2) Determinant-weighted nonlinear least squares
%           (3) Linearized least squares
%
% opt{3} = choose between left or right MFD
%          (lmfd) left MFD
%          (rmfd) right MFD
%
% opt{4} = ro parameter in treating discrete gusts using the RMFD. 
%          H = N/D with:
%                        D = D0 + ... + Di*p^n; 
%                        N = N0 + ... + Ni*p^(n+ro) 
%          NOTE: when a LMFD is choosen ro is setted automatically to ro = 2;
%
% opt{5} = weights applied at the second reduced frequency 
%          during the final LS solution
%
%
% Levenberg-Marquardt parameters
% -------------------------------------------------------------------------
% tau     = value used in defining Levenberg-Marquardt lambda parameter: lambda = tau*max(Jac'*Jac).
% tolg    = tolerance for gradient stopping criteria  
% tolx    = tolerance for solution variation stopping criteria   
% maxiter = maximum number of iterations
%
%
% Stabilizations parameters
% -------------------------------------------------------------------------
% eigsopt.threshold = threshold value for the eigenvalues real part below which they are shifted  
%
% eigsopt.type      = unstable eigenvalues shifting method
%                      'flip': flipping about the imaginary axis
%                     'bound': real part of unstable eigenvalues are setted to a user defined value
%
% eigsopt.bound     = value to which the real part of any unstable eigenvalue is setted 
%                     (if 'bound' option is selected in eigsopt.type)
%
% eigsopt.method    = method of eigenvalues shifting
%                       'eigshift': eigenvalues shift and eigenvector correction to mantain the state matrix structure
%                     'polesplace': poles placement using a state feedback matrix
%
% 
% Model reduction algorithm
% -------------------------------------------------------------------------
% algROM = available model reduction algorithms
%          'balance': balanced model truncation via square root method              (help balancmr)
%           'hankel': Hankel minimum degree approximation (MDA) without balancing   (help hankelmr)
%            'schur': balanced model truncation via Schur method                    (help schurmr)
%              'bst': balanced stochastic model truncation (BST) via Schur method   (help bstmr)
%
%
% Continuation procedure
% -------------------------------------------------------------------------
% restart = filename of the previous MFD stable solution calculated 
%           used as initial guess to starts the identification.
% =========================================================================

clear all
close all
clc

%--------------------------------------------------------------------------
%                              USER INPUT DATA
%--------------------------------------------------------------------------
% Identification options
opt{1} = 1;      % MFD order
opt{2} = 1;      % MFD algorithm
opt{3} = 'lmfd'; % left or right MFD
opt{4} = 2;      % ro parameter in treating discrete gusts using the RMFD 
opt{5} = 100;    % weight parameter value W^2 

% Levenberg-Marquardt parameters
tau     = 1e-3;  
tolg    = 1e-6;  
tolx    = 1e-6; 
maxiter = 100;
optsLM = [tau tolg tolx maxiter]; 

% stabilizations parameters
eigsopt.threshold = -1e-4;   
eigsopt.type      = 'bound';      % 'bound' or 'flip'
eigsopt.bound     =  -0.005;
eigsopt.method    = 'polesplace'; %'polesplace' or 'eigshift'

% Model reduction algorithm
algROM = 'balance'; % 'schur', 'hankel', 'bst'
%--------------------------------------------------------------------------

% save options in a file
save('example_optionfile.mat')
