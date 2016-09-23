function [D,N] = extractmatrices( MFDtype, NLSalg, theta, H0, ordD, ordN )
% =========================================================================
%                                                           extractmatrices
% =========================================================================
%
% Description: extract the matrix fraction description matrices D and N
%              from the unknown matrix (theta)
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

% auxiliary variables
ii = 1;  % starting index (because arrays starts at index 1 in MATLAB)
dy = size(H0,1);
du = size(H0,2);
Iu = eye(du);
Iy = eye(dy);
nu = 0;
ny = 0;

% initialization
D = cell(1,ordD);
N = cell(1,ordN);

% select Left MFD or Right MFD
switch MFDtype
    
    
    case {'left','LEFT','lmfd','LMFD'}
        
        % select nonlinear least squares algorithm
        % -----------------------------------------------------------------
        switch NLSalg
            
            case {'LS1','kron',1}
                % ---------------------------------------------------------
                % Reshaping the unknown from a vector to a matrix
                theta = reshape( theta, dy, ordD*dy + ordN*du );
                for m = 0 : (ordD-1)
                    D{ii+m} = theta(:,ny*dy+nu*du+(1:dy)); ny=ny+1;
                end
                for m = 1 : ordN
                    N{ii+m} = theta(:,ny*dy+nu*du+(1:du)); nu=nu+1;
                end
                
            case {'LS2','det','adj',2, 'LS3','lin',3 }
                % ---------------------------------------------------------
                for m = 0 : (ordD-1)
                    D{ii+m} = ( theta(ny*dy+nu*du+(1:dy),:) )'; ny = ny+1;
                end
                for m = 1 : (ordN)
                    N{ii+m} = ( theta(ny*dy+nu*du+(1:du),:) )'; nu = nu+1;
                end
                
        end
        
        % Apply constraints
        D{ii+ordD} = Iy;
        N{ii+0} = D{ii+0}*H0;  % N{ii+0} calculated from H(0) constraint
        
          
    case {'right','RIGHT','rmfd','RMFD'}
        
        % select nonlinear least squares algorithm
        % -----------------------------------------------------------------
        switch NLSalg
            
            case {'LS1','kron',1}
                % ---------------------------------------------------------
                % Reshaping the unknown from a vector to a matrix
                theta = reshape( theta, du, ordD*du + ordN*dy );
                % unknown matrix defined as theta = [ D0' D1' ... Dn' N1' N2' ...  N(n+2)' ];
                for m = 0:ordD-1
                    D{ii+m} = ( theta(:,ny*dy+nu*du+(1:du)) )'; nu=nu+1;
                end
                for m = 1:ordN
                    N{ii+m} = ( theta(:,ny*dy+nu*du+(1:dy)) )'; ny=ny+1;
                end
                
            case {'LS2','det','adj',2, 'LS3','lin',3 }
                % ---------------------------------------------------------
                % unknown matrix defined as theta = [D0; D1; ... Dn; N1; N2; ... N(n+2)];
                for m = 0 : (ordD-1)
                    D{ii+m} = theta( ny*dy+nu*du+(1:du), : ); nu = nu+1;
                end
                for m = 1 : ordN
                    N{ii+m} = theta( ny*dy+nu*du+(1:dy), : ); ny = ny+1;
                end
                
        end
        
        % Apply constraints
        D{ii+ordD} = Iu;
        N{ii+0} = H0*D{ii+0};  % N{ii+0} calculated from H(0) constraint
        
end