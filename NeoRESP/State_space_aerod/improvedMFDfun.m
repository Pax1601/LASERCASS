function solution = improvedMFDfun(redfreq,Ha,opt,optsLM,eigsopt,algROM,restart)
% =========================================================================
%                                                            improvedMFDfun
% =========================================================================
%
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
%   - residual output order set for LMFD
%
%
% =========================================================================
% 
%
%                                  INPUT
% -------------------------------------------------------------------------
% redfreq = vector of the reduced frequencies, dimensions (1 x nk) 
% Ha      = aerodynamic tranfer matrix (or generalized aerodynamic forces GAF)
%           dimensions (ny x nu x nk)
%
% Identification options
% -------------------------------------------------------------------------
% opt{1}  = MFD order n, such that: 
%           D = D0 + p*D1 + ... + p^(n+1)*D(n-1) + p^n*I;
%           N = N0 + p*N1 +          ...         + p^n*Nn + p^(n+ro)*N(n+ro); 
%           where ro is the residualization order provided in opt{4} (see below)
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
% opt{4} = residualization order of approximation. 
%          H = N/D with:
%                        D = D0 + ... + Di*p^n; 
%                        N = N0 + ... + Ni*p^(n+ro) 
%
% opt{5} = weights applied at the second reduced frequency 
%          during the final LS solution
%
% opt{6} = roll off shape filter parameter rosf in treating discrete gusts using the RMFD. 
%          H = N/D with:
%                        D = D0 + ... + Di*p^(n+rosf); 
%                        N = N0 + ... + Ni*p^n 

%
% Levenberg-Marquardt parameters
% -------------------------------------------------------------------------
% tau     = value used in defining Levenberg-Marquardt 
%           lambda parameter: lambda = tau*max(Jac'*Jac).
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
%                     'bound': real part of unstable eigenvalues are set to a user defined value
%
% eigsopt.bound     = value to which the real part of any unstable eigenvalue is set 
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
%
% Continuation procedure
% -------------------------------------------------------------------------
% restart = filename of the previous MFD stable solution calculated 
%           used as initial guess to starts the identification.
%
% =========================================================================


% =========================================================================
% Pre-processing
% =========================================================================
ordMFD  = opt{1}; % order of the MFD
NLSalg  = opt{2}; % nonlinear LS algorithm
MFDtype = opt{3}; % left or right MFD
ro      = opt{4}; % residualization order 
weight  = opt{5}; % weight on second reduced frequency
% MODIFICARE
%rosf    = opt{6}; % roll off shape filter
%
ro = 2
rosf = 0
% Orders
if rosf
   orderD = ordMFD + rosf;
   orderN = ordMFD;
   ro = 0;
else
   orderD = ordMFD;
   orderN = ordMFD + ro;
end

% auxiliary variables
ny = size(Ha,1); % number of output of the aerodynamic tranfer matrix
nu = size(Ha,2); % number of input of the aerodynamic tranfer matrix
 
% verify first reduced frequency k = 0
%if redfreq(1) ~= 0, redfreq(1) = 0;
%    fprintf(1,'\nWarning: first reduced frequency has been set to k = 0. \n');
%end

% harmonic reduced frequencies
p = 1i*redfreq;

% extract GAFs matrices at the first reduced frequency
Hval{1} = squeeze( Ha(:,:,1) );

% impose real GAF at k = 0
H0 = real( Hval{1} );
H0R = ( Hval{1} );
% flag activating the stabilization procedure by eigenvalues shifting
eigsopt.flagstab = 1;      

%
% =========================================================================
% Improved MFD identification process
% =========================================================================
    
% Matrices initialization
% =========================================================================

% checks for previous solution existence
if exist(restart,'file') || exist( strcat(restart,'.mat'),'file')
    
    % Continuation procedure
    % ---------------------------------------------------------------------
    % starts the MFD using the previous stable MFD solution calculated
    fprintf(1,'\n\nInitialize matrices by using the previous stable MFD solution\n');
    fprintf(1,'-------------------------------------------------------------------------\n');
    load(restart,'solution');
    
    Dnls = solution.restart.D;
    Nnls = solution.restart.N;
    
    oldDordMFD = length(Dnls) - 1; % old D order
    oldNordMFD = length(Nnls) - 1; % old N order
    ii = 1;
    Dls = cell(1,orderD+1); % new D
    Nls = cell(1,orderN+1); % new N
    % initialize matrices
    for m = 0:orderD,
        if m <= oldDordMFD, Dls{ii+m} = Dnls{ii+m};
        else Dls{ii+m} = zeros( size(Dls{ii}) ); end
    end
    
    for m = 0:orderN,
        if m <= oldNordMFD, Nls{ii+m} = Nnls{ii+m};
        else Nls{ii+m} = zeros( size(Nls{ii}) ); end
    end
    
else
    
    fprintf(1,'\nRestart file not found. Initializing matrices with least squares solution.\n');
    
    % Linear least-squares with imposed Ha at p=0
    % ---------------------------------------------------------------------
    fprintf(1,'\n\nLinear least squares solution\n');
    fprintf(1,'-------------------------------------------------------------------------\n');
    switch MFDtype
        case {'lmfd','left','LMFD'},  [Dls,Nls] = MyIdentification_LinearLS(ordMFD,p,Ha,H0,1,ro);
        case {'rmfd','right','RMFD'}, [Dls,Nls] = MyIdentification_RightMFD_LinearLS(ordMFD,p,Ha,H0,1,ro);
    end
    fprintf(1,'...linear LS matrices initialization done.\n');
    
end

% evaluate approximated aerodynamic transfer matrix
Happrox = MFDapprox(ordMFD,p,Dls,Nls,MFDtype,ro);

% error calculation
errorcalc(Ha,Happrox,1);


% Nonlinear least-squares algorithms
% =========================================================================
[Dnls,Nnls] = myMFDidentification(ordMFD,MFDtype,NLSalg,p,Ha,H0,Dls,Nls,ro,optsLM,eigsopt,1);

% save MFD matrices for successive continuation procedure
solution.restart.D = Dnls;
solution.restart.N = Nnls;


% State space realization
% =========================================================================
fprintf(1,'\n\nState space realization:\n');
fprintf(1,'-------------------------------------------------------------------------\n');
[E0,E1,E2,SS_A,SS_B,SS_C] = mySSrealization(ordMFD,ro,Dnls,Nnls,MFDtype);

% error calculation
Happrox = SSapprox(p,SS_A,SS_B,SS_C,E0,E1,E2);
err = errorcalc(Ha,Happrox,1);


% Check stability
% =========================================================================
% TODO: compare eigenvalues arising from RMA and those of State-space realization if are the same

% check eigenvalues of RMFA identification
eigsopt.flagstab = 0; % set to avoid entering in stabilization statement
[is_stab,~] = myStabilization(ordMFD,Dnls,Nnls,eigsopt);

% check eigenvalues of state space realization
is_stab = all( real(eig(SS_A)) < 0 ) & is_stab;

if is_stab, checkstab = 'stable'; else checkstab = 'unstable'; end
fprintf(1,'\nCheck stability of state-space system: %s\n', checkstab);


% (E,C) refitting before model order reduction
% =========================================================================
err0 = err; % error coming from MFD state space realization
fprintf(1,'\n\n(E,C) refitting before model order reduction:\n');
fprintf(1,'-------------------------------------------------------------------------\n');
nx   = size(SS_A,1);
B{1} = SS_B;   B{2} = zeros(nx,nu);   B{3} = zeros(nx,nu);   B{4} = zeros(nx,nu);
[Happrox,SS_Ctmp,E0tmp,E1tmp,E2tmp] = MyIdentification_LinearLS_ECrefitting(p,Ha,H0R,SS_A,B,weight,ro);

% error calculation
err = errorcalc(Ha,Happrox,1);

% solution update
if err < err0,  SS_C = SS_Ctmp;
else fprintf(1,'Warning: error increased... (E,C) refitting solution discarded.\n');
end


% Model order reduction
% =========================================================================   
fprintf(1,'\n\nModel order reduction:\n');
fprintf(1,'-------------------------------------------------------------------------\n');

% model reduction without residualization
%--------------------------------------------------------------------------
[SS_Atmp1,~,SS_Ctmp1,SS_Dtmp1,ordred1] = full2ROM(SS_A,SS_B,SS_C,[],algROM, [] ); % order selected by user
if norm(SS_Dtmp1) ~= 0, % check SS_D null
    error('Direct input-output matrix is not null after model order reduction.'); 
end

% Final (E,B) refitting
[Happrox,B0tmp1,B1tmp1,B2tmp1,B3tmp1,E0tmp1,E1tmp1,E2tmp1] = MyIdentification_LinearLS_EBrefitting(p,Ha,H0R,SS_Atmp1,SS_Ctmp1,weight,ro);

% error calculation
err1 = errorcalc(Ha,Happrox,0);
err2 = inf;
err3 = inf;


% model reduction with residualization
% -------------------------------------------------------------------------
nx = size(SS_A,1); % original system (not reduced)
if ordred1 < nx % if the user want to reduce the order try if a residualization improve the results
    
    if ( strcmp(algROM,'balance') ) || ( strcmp(algROM,'schur') ), ordred2 = nx-1; % because balancmr and schurmr does not transform the system if the order is mantained
    else ordred2 = nx; end
    
    % static residualization
    % ---------------------------------------------------------------------
    [SS_Atmp2,~,SS_Ctmp2,SS_Dtmp2,~] = full2ROM(SS_A,SS_B,SS_C,[],algROM, ordred2 );
    if norm(SS_Dtmp2) ~= 0 % check SS_D null
        error('Direct input-output matrix in not null after model order reduction.');
    end
    
    % extract submatrices
    A11 = SS_Atmp2(1:ordred1,1:ordred1);
    A12 = SS_Atmp2(1:ordred1,ordred1+1:end);
    A22 = SS_Atmp2(ordred1+1:end,ordred1+1:end);
    A21 = SS_Atmp2(ordred1+1:end,1:ordred1);
    C1  = SS_Ctmp2(:,1:ordred1);
    C2  = SS_Ctmp2(:,ordred1+1:end);
    
    % check singularity
    checkstatic = 1;
    if cond(A22) > 10^15,  checkstatic = 0;
        fprintf(1,'Warning: near singular system after static residualization... abort process.\n');
        err2 = inf;
    end
    
    if checkstatic
        
        % calculate state matrix
        invA22 = A22\eye(ordred2-ordred1);
        SS_Atmp2 = A11 - A12*( invA22*A21 );
        SS_Ctmp2 = C1  - C2 *( invA22*A21 );
        
        % check stability
        eig2 = eig(SS_Atmp2);
        checkstatic = ~sum( real( eig2 ) > -1e-6 );
        if ~checkstatic
            fprintf(1,'Warning: unstable system after static residualization... abort process.\n'); err2 = inf;
        else
            
            % Final (E,B) refitting
            [Happrox,B0tmp2,B1tmp2,B2tmp2,B3tmp2,E0tmp2,E1tmp2,E2tmp2] = MyIdentification_LinearLS_EBrefitting(p,Ha,H0R,SS_Atmp2,SS_Ctmp2,weight,ro);
            
            % error calculation
            err2 = errorcalc(Ha,Happrox,0);
            
        end
        
    end
    
    % first order residualization
    % ---------------------------------------------------------------------
    Ix = eye(ordred1);
    EE = Ix + A12*( (invA22^2)*A21 );
    
    % check singularity
    checkres = 1;
    if ( cond(EE) > 10^15 )  ||  ( cond(A22) > 10^12 ),  checkres = 0;
        fprintf(1,'Warning: near singular system after first order residualization... abort process.\n');
        err3 = inf;
    end
    
    if checkres
        
        % calculate state matrix
        SS_Atmp3 = EE\( A11 - A12*( invA22*A21 ) );
        SS_Ctmp3 = C1 - C2*( ( (invA22^2)*A21) * SS_Atmp3 + (invA22*A21) );
        
        % check stability
        eig3 = eig(SS_Atmp3);
        checkres = ~sum( real( eig3 ) > -1e-6 );
        if ~checkres
            fprintf(1,'Warning: unstable system after first order residualization... abort process.\n');
            err3 = inf;
        else
            
            % Final (E,B) refitting
            [Happrox,B0tmp3,B1tmp3,B2tmp3,B3tmp3,E0tmp3,E1tmp3,E2tmp3] = MyIdentification_LinearLS_EBrefitting(p,Ha,H0R,SS_Atmp3,SS_Ctmp3,weight,ro);
            
            % error calculation
            err3 = errorcalc(Ha,Happrox,0);
            
        end
        
    end
    
end    

% update solution
% ---------------------------------------------------------------------
if  (err2 < err1) && (err2 < err3) && checkstatic
    
    err = err2;
    
    % Static residualization applied
    SS_A = SS_Atmp2; SS_C = SS_Ctmp2;
    B0 = B0tmp2; B1 = B1tmp2; B2 = B2tmp2; B3 = B3tmp2;
    E0 = E0tmp2; E1 = E1tmp2; E2 = E2tmp2;
    fprintf(1,'\nStatic residualization has been applied.\n');
    
elseif (err3 < err1) && (err3 < err2) && checkres
    
    err = err3;
    
    % First order residualization applied
    SS_A = SS_Atmp3; SS_C = SS_Ctmp3;
    B0 = B0tmp3; B1 = B1tmp3; B2 = B2tmp3; B3 = B3tmp3;
    E0 = E0tmp3; E1 = E1tmp3; E2 = E2tmp3;
    fprintf(1,'\nFirst order residualization has been applied.\n');
    
else
    
    err = err1;
    
    % solution update without residualization
    SS_A = SS_Atmp1; SS_C = SS_Ctmp1;
    B0 = B0tmp1; B1 = B1tmp1; B2 = B2tmp1; B3 = B3tmp1;
    E0 = E0tmp1; E1 = E1tmp1; E2 = E2tmp1;
    fprintf(1,'\nTruncation has been applied.\n');
    
end
fprintf(1,'error: %12.6e\n', err);

% Eigenvalues
eigvalA = eig(SS_A);
fprintf(1,'\nEigenvalues after order reduction: \n');
fprintf(1,'%14.3e%11.3ei\n',[real(eigvalA), imag(eigvalA)].');


% Alternate (E,C) - (E,B) refitting
% =========================================================================
fprintf(1,'\nAlternate (E,C) - (E,B) refitting:\n');
fprintf(1,'-------------------------------------------------------------------------\n');

B0tmp  = B0; B1tmp = B1; B2tmp = B2; B3tmp = B3;
err0   = err;  % error coming from previous solution
errnew = inf;  % to enter the while statement
errold = err0;
errred = inf;
while (errnew > 0.002)  &&  (errred > 0.01) 
    
    % (E,C) fitting 
    % ---------------------------------------------------------------------
    B = cell(1,4);  B{1} = B0tmp;  B{2} = B1tmp;  B{3} = B2tmp;  B{4} = B3tmp;
    [Happrox,SS_Ctmp,~,~,~] = MyIdentification_LinearLS_ECrefitting(p,Ha,H0R,SS_A,B,weight,ro);
    % error calculation
    fprintf(1,'(E,C) fitting '); errorcalc(Ha,Happrox,1);
    
    % (E,B) fitting 
    % ---------------------------------------------------------------------
    [Happrox,B0tmp,B1tmp,B2tmp,B3tmp,E0tmp,E1tmp,E2tmp] = MyIdentification_LinearLS_EBrefitting(p,Ha,H0R,SS_A,SS_Ctmp,weight,ro);
    % error calculation
    fprintf(1,'(E,B) fitting '); errnew = errorcalc(Ha,Happrox,1);
    errred = abs((errnew - errold)/errold);
    errold = errnew;
    fprintf(1,'Error reduction: %e\n', errred);
end

if errnew < err0
    % solution update
    SS_C = SS_Ctmp;
    E0 = E0tmp;  E1 = E1tmp;  E2 = E2tmp;
    B0 = B0tmp;  B1 = B1tmp;  B2 = B2tmp;  B3 = B3tmp;
    fprintf(1,'Error decreased... alternate (E,C)-(E,B) refitting solution accepted.\n');
else
    fprintf(1,'Warning: error increased... alternate (E,C)-(E,B) refitting solution discarded.\n');
end


% =========================================================================
% Post-processing
% =========================================================================

% Final form of the second-order residualization
% =========================================================================

% Second order residualization in input and output
% ------------------------------------------------
AA    = SS_A;
BB{1} = B0;           % referred to p^0
BB{2} = B1;           % referred to p^1
BB{3} = B2 + AA*B3;   % referred to p^2
CC    = SS_C;
DD{1} = E0;           % referred to p^0
DD{2} = E1;           % referred to p^1
DD{3} = E2 + CC*B3;   % referred to p^2

solution.inoutresid.AA = AA;
solution.inoutresid.BB = BB;
solution.inoutresid.CC = CC;
solution.inoutresid.DD = DD;

% Second order residualization only for output
%---------------------------------------------
AA    = SS_A;
BB    = B0 + AA*( B1 + AA*( B2 + AA*B3 ) );
CC    = SS_C;
DD{1} = E0 + CC*( B1 + AA*( B2 + AA*B3 ) );  % referred to p^0
DD{2} = E1 + CC*( B2 + AA*B3 );              % referred to p^1
DD{3} = E2 + CC*B3;                          % referred to p^2

solution.outresid.AA = AA;
solution.outresid.BB = BB;
solution.outresid.CC = CC;
solution.outresid.DD = DD;
     

% Evaluate approximation (second order residualization only for output)
% =========================================================================
Happrox = SSapprox(p,AA,BB,CC,DD{1},DD{2},DD{3});


% Plot results
% =========================================================================
fprintf(1,'\n\nPlot results\n');
fprintf(1,'-------------------------------------------------------------------------\n');
labeltitle = 'Improved MFD - aerodynamic matrix plot';
labelleg   = 'MFD approx.';
nfig = 1;
plotfigures(p,Ha,Happrox,labeltitle,labelleg,nfig);

