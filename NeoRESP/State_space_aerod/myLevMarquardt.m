function xlm = myLevMarquardt(fun, x0, opts, varargin)
% =========================================================================
%                                                            myLevMarquardt
% =========================================================================
%
% Description: Levenberg-Marquardt's algorithm for solving nonlinear least
%              squares problems involving unknowns in a matrix form.
%
% -------------------------------------------------------------------------
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
% =========================================================================
%
%                                 INPUT
% -------------------------------------------------------------------------
%  fun:  handle to the function providing the residual r and jacobian J.
%
%   x0:  initial guess for the unknown matrix.
% 
% opts:  vector containing the algorithm options
%        opts = [tau tolg tolx maxiter], where:
%         tau: used to inizialize the Lev-Mar. damping parameter (lambda)
%        tolg: used in stopping criteria for the gradient solution
%        tolx: used in stopping criteria for the solution
%     maxiter: maximum number of iterations
%
%                                 OUTPUT
% -------------------------------------------------------------------------
%  xlm:  solution matrix.
%
% =========================================================================

% Check initial guess
[err x] = checkx(x0);
if  ~err,
    [err f r J] = evalrJ(fun, x0, varargin{:});
    iter = 1;
end

if  ~err
    
    A = J'*J;
    g = J'*r;
 
    ng = norm(g,inf);
    err = checkinf(A,ng); % check overflow 
    
else
    f  = NaN; ng = NaN;
end

if  err
    xlm = x0; 
    nh  = 0;  nstep = 0;  iter = 1;
    lambda = opts(1); 
    printinfo( nstep, iter, err, f, ng, nh, lambda, A );
    return
end

% algorithm options
if isempty(opts) || (nargin < 3) % use default options
    opts = [ 1e-3  1e-4  1e-8  100 ];
end
tau     = opts(1);
tolg    = opts(2);
tolx    = opts(3);
maxiter = opts(4);

%  Parameters initialization
lambda = tau * max(diag(A));
nu    = 2;
nh    = 0;
err   = 0;
nstep = 0;

% Levenberg-Marquardt iterations
% -------------------------------------------------------------------------
while ~err
    
    % Check tolerances on gradient (g) and solution variation (h)
    if  ng <= tolg, 
        err = 1; % Stopped by small gradient
    else
        
        % Least squares solution for the incremental unknown matrix
        % -----------------------------------------------------------------
        [h lambda] = solve4h(A,g,lambda);
        nh = norm(h);
        nx = tolx + norm(x);
        if  nh <= tolx*nx, err = 2; end % Stopped by small solution variation
        
    end
    
    if ~err

        % update solution
        % -----------------------------------------------------------------
        xnew = x - h;  % new solution
        
        % setting at zero the matrix elements below an imposed threshold value 
        threshold = 10^-12;
        xnew(abs(xnew) < threshold) = 0;
 
        % calculate jacobian and residual at the new solution point
        % -----------------------------------------------------------------
        [err fn rn Jn] = evalrJ(fun, xnew, varargin{:});
        iter = iter + 1;
        
        % calculate dL
        dL = ( h(:)'*(lambda*h(:) - g(:)) )/2;
        
        if ~err
            
            % calculate df
            df = ( (r(:) - rn(:))' * (r(:) + rn(:)) )/2;
            
            % check for gain ratio df/dL
            % -------------------------------------------------------------
            if (dL > 0) && (df > 0)           % Update x and modify lambda
                nstep = nstep + 1;
                
                % update variable
                x  = xnew;
                f  = fn;
                J  = Jn;
                r  = rn;
                A = J'*J;
                g = J'*r;
                ng = norm(g,inf);
                
                % modify lambda
                lambda = lambda * max( 1/3, 1 - ( 2*df/dL - 1 )^3 );
                nu = 2;
                
            else                                  % Same x, increase lambda
                lambda = lambda*nu;
                nu = 2*nu;
            end
            
            % check iterations and solution convergence
            % -------------------------------------------------------------
            if iter > maxiter,
                err = 3;  % number of iterations exceeded
            else
                err = checkinf( A, ng );  % check overflow 
            end
            
        end
    end
end

% Final solution
% -------------------------------------------------------------------------
xlm = x;
if err < 0, f = NaN;  ng = NaN; end

% Print to screen informations
% -------------------------------------------------------------------------
printinfo( nstep, iter, err, f, ng, nh, lambda, A );

end


function printinfo( nstep, iter, err, f, ng, nh, lambda, A )
% -------------------------------------------------------------------------
% Print to screen informations
% -------------------------------------------------------------------------
fprintf(1,'\nLevenberg-Marquardt\n');
fprintf(1,'   - no. of solution steps: %d\n',nstep);
fprintf(1,'   - total iterations: %d\n',iter);
switch err
    case  1,  stoptext = 'small gradient.';
    case  2,  stoptext = 'small solution variation.';
    case  3,  stoptext = 'maximum number of iterations reached.';
    case -1,  stoptext = 'overflow during computation. Solution, residual or jacobian may be not finite.';
end
fprintf(1,'   - stop by: %s\n',stoptext);
fprintf(1,'   - objective function:      %9.4e\n',f);
fprintf(1,'   - gradient inf-norm:       %9.4e\n',ng);
fprintf(1,'   - solution increment norm: %9.4e\n',nh);
fprintf(1,'   - lambda:    %9.4e\n',lambda);
fprintf(1,'   - norm(J''J): %9.4e\n',norm(A));
fprintf(1,'   - rank(J''J): %d\n',rank(A));
fprintf(1,'   - size(J''J): %d x %d\n',size(A,1), size(A,2));

end


function [h, lambda] = solve4h(A,g,lambda)
% -------------------------------------------------------------------------
% LS SOLVER for Levenberg-Marquardt algorithm
% -------------------------------------------------------------------------

% Solve (A + lambda*I)h = -g, or (A + lambda*diag(A))h = -g
mA = max(abs(A(:)));
if mA == 0  % zero matrix
    h = -g/lambda;  return
end

% Cholesky decomposition with check of matrix positive-definiteness
n = size(A,1);  
I = eye(n);
check = 1;
while check
    
    % Levenberg algorithm
    [R check] = chol( A + lambda*I );
    
    % check for near singularity
    if check == 0
        check = rcond(R) < 1e-15;
    end
    
    if check,
        lambda = max(10*lambda, eps*mA);
    end
    
end

% Solve  (R'*R)h = -g, with R upper triangular matrix
h = - R\(R'\g);

end


function [err, x] = checkx(x0)
% -------------------------------------------------------------------------
% Check solution
% -------------------------------------------------------------------------
if  all(isfinite(x0(:))) && isfinite(norm(x0(:))) 
    err = 0; x = x0;
else 
    err = -1; x = [ ];
end

end


function  [err, f, r, J] = evalrJ(fun, x, varargin)
% -------------------------------------------------------------------------
% Evaluate residual r and jacobian J
% -------------------------------------------------------------------------
[r J] = feval(fun,x,varargin{:});

% objective function
f = ( r(:)' * r(:) )/2; 

if  all(isfinite(r(:))) && all(isfinite(J(:))) && isfinite(f), err = 0;   
else err = -1; return, end

end

function err = checkinf( A, ng )
% -------------------------------------------------------------------------
%  check overflow during computation
% -------------------------------------------------------------------------
if  isinf(norm(A(:),inf)) || isinf(ng), 
    err = -1; 
else
    err = 0;
end

end

