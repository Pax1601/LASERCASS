function [D,N] = myMFDidentification(ord,MFDtype,NLSalg,p,H,H0,Dinit,Ninit,ro,optsLM,eigsopt,weight)
% =========================================================================
%                                                       myMFDidentification
% =========================================================================
%
% Description: nonlinear least squares Matrix Fraction Description  
%              identification with state matrix stabilization.
%
% -------------------------------------------------------------------------
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
%
%
% =========================================================================

% -------------------------------------------------------------------------
% Pre-processing
% -------------------------------------------------------------------------

% auxiliary variables
if isempty(ro), ro = 2; end
orderD = ord;           % D0 + ... + Di*p^orderD
orderN = ord + ro;      % N0 + ... + Ni*p^orderN

% select MFD function
switch MFDtype % select Left MFD or Right MFD
    
    case {'left','LEFT','lmfd','LMFD'}
        switch NLSalg % select nonlinear least squares algorithm
            case {'LS1','kron',1},      myMFDfun4LM = 'MyIdentification_Kron_Fun_4LM';
            case {'LS2','det','adj',2}, myMFDfun4LM = 'MyIdentification_Adjugate_Fun_4LM';
            case {'LS3','lin',3},       myMFDfun4LM = 'MyIdentification_LinIncr_Fun_4LM';
        end
         
    case {'right','RIGHT','rmfd','RMFD'}
        switch NLSalg % select nonlinear least squares algorithm
            case {'LS1','kron',1},      myMFDfun4LM = 'MyIdentification_RightMFD_Kron_Fun_4LM';
            case {'LS2','det','adj',2}, myMFDfun4LM = 'MyIdentification_RightMFD_Adjugate_Fun_4LM';
            case {'LS3','lin',3},       myMFDfun4LM = 'MyIdentification_RightMFD_LinIncr_Fun_4LM';
        end   
end

% assembling initial guess of the unknown matrix
thetaold = assembleunknowns( MFDtype, NLSalg, Dinit, Ninit );

% -------------------------------------------------------------------------
% Identification process
% -------------------------------------------------------------------------

% Nonlinear least square solution using Levenberg-Marquardt algorithm 
% -------------------------------------------------------------------------
theta = myLevMarquardt(myMFDfun4LM, thetaold, optsLM, ord,p,H,H0,weight,ro);

% Stabilization
% -------------------------------------------------------------------------
if eigsopt.flagstab
    
    % Extract Di, Ni matrices
    [D,N] = extractmatrices( MFDtype, NLSalg, theta, H0, orderD, orderN );
    
    % Stabilization
    [is_stab,D] = myStabilization(ord,D,N,eigsopt);
      
    % iterations to obtain the stability
    % ---------------------------------------------------------------------
    iter_stab =  1;
    Niter_stab = 100;
    while (~is_stab) && (iter_stab <= Niter_stab)
          
        % assembling the unknown matrix theta
        theta = assembleunknowns( MFDtype, NLSalg, D, N );
        
        % solving Levenberg-marquardt for theta
        theta = myLevMarquardt(myMFDfun4LM, theta, optsLM, ord,p,H,H0,weight,ro);    
        
        % Extract Di, Ni matrices
        [D,N] = extractmatrices( MFDtype, NLSalg, theta, H0, orderD, orderN );
        
        % Stabilization
        [is_stab,D] = myStabilization(ord,D,N,eigsopt);
        
        fprintf('\niter: %d  - Stability [(0) unstable, (1) stable]: %d\n',iter_stab,is_stab);
        fprintf('---------------------------------------------------\n');
        iter_stab = iter_stab + 1;

    end
    % ---------------------------------------------------------------------
    
    % assembling the unknown matrix theta
    theta = assembleunknowns( MFDtype, NLSalg, D, N );

end
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% Post-processing
% -------------------------------------------------------------------------

% Extract matrices from LM solution
[D,N] = extractmatrices( MFDtype, NLSalg, theta, H0, orderD, orderN );

