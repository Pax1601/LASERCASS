% =========================================================================
%
%          Improved Matrix Fraction Description (MFD) identification
%
% =========================================================================
%
%
%   Copyright (C) 2012 Paolo Mantegazza   <mantegazza@aero.polimi.it>
%   Copyright (C) 2012 Matteo Ripepi      <ripepi@aero.polimi.it>
%  
%   This program is free software; you can redistribute it and/or
%   modify it under the terms of the GNU General Public License as
%   published by the Free Software Foundation; either version 3 of the
%   License, or (at your option) any later version.
%  
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%  
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
%  
%   Ref: Ripepi M., Mantegazza P., 'An improved matrix fraction approximation of
%        aerodynamic transfer matrices', AIAA journal, submitted for publication.
%   Changes:
%   - saturation of determinant weighting;
%   - enforcement of MFD stability choosable between minimal eigensolution
%     changes or poles placement;
%   - added initial touch-up of the MFD output matrix;
%   - balanced reduction chosen to be the best between truncation and
%     low frequency residualisation of the balanced reduction (norm
%     bound given up in favour of a better fit, whenever possible)
%   - state equation residual extended to third order;
%   - low frequency equality constraint simplified through
%     weighting in the final touch;
%   
%
% =========================================================================
%
%                                  INPUT
% -------------------------------------------------------------------------
% redfreq = vector of the reduced frequencies, dimensions (1 x nk) 
% Ha      = aerodynamic tranfer matrix (or generalized aerodynamic forces GAF)
%           dimensions (ny x nu x nk)
%
%
%                     LIST OF OPTIONS AND PARAMETERS
%
% Identification options:
% -------------------------------------------------------------------------
% opt{1} = MFD order n, such that: 
%          D = D0 + p*D1 + ... + p^(n+1)*D(n-1) + p^n*I;
%          N = N0 + p*N1 +          ...         + p^n*Nn + p^(n+1)*N(n+1) + p^(n+2)*N(n+2); 
%
% opt{2} = MFD algorithm:
%          (1) Kronecker-based nonlinear least squares
%          (2) Determinant-weighted nonlinear least squares
%          (3) Linearized least squares
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
% opt{5} = weight applied on the second reduced frequency 
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
%
% =========================================================================

clear all
close all
clc

fprintf('=========================================================================\n\n');
fprintf('        Improved Matrix Fraction Description (MFD) identification        \n\n');
fprintf('=========================================================================\n\n');

%--------------------------------------------------------------------------
%                              PROCESSING
%--------------------------------------------------------------------------

% ask for options file
inputoptfilename = input('\n>> Input file name containing the identification options\n   (press enter if not available): ','s');

if exist(inputoptfilename,'file') || exist( strcat(inputoptfilename,'.mat'),'file')
    
    % load *.mat file containing the identification options
    load( inputoptfilename, '-mat' );
    
    % load aerodynamic transfer matrix and reduced frequencies
    inputdatafilename = input('\n>> Input file name containing the aerodynamic data: ','s');
    while ~exist(inputdatafilename,'file') && ~exist( strcat(inputdatafilename,'.mat'),'file')
        fprintf(1,'   File not found.\n');
        inputdatafilename = input('\n>> Input file name containing the aerodynamic data: ','s');
    end
    load( inputdatafilename, '-mat' );
    
    % ask for previous stable solutions MFD matrices
    restart = input('\n>> Input file name of a previous solution for matrices inizialization\n   (press enter if not available): ','s');
    
    % execute the improved matrix fraction approximation
    solution = improvedMFDfun(k,Ha,opt,optsLM,eigsopt,algROM,restart);
    
    % save data
    fprintf(1,'\n\nSave results\n');
    fprintf(1,'-------------------------------------------------------------------------\n');
    outfilename = input('\n\n>> Enter output file name: ','s');
    save( outfilename, '-mat' );
    
else
    
    fprintf(1,'\nOptions file not found. Opening graphical user interface.\n');
    
    % load the GUI interface
    mygui
    
end
%--------------------------------------------------------------------------