function theta = assembleunknowns(MFDtype, NLSalg, D,N)
% =========================================================================
%                                                          assembleunknowns
% =========================================================================
%
% Description: assemble the unknown matrix (theta) from the matrix fraction
%              description matrices D and N
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
ii   = 1;
dy   = size(N{1},1);
du   = size(N{1},2);
ordD = length(D) - 1;
ordN = length(N) - 1;

% select Left MFD or Right MFD
switch MFDtype
    
    case {'left','LEFT','lmfd','LMFD'}
    % ---------------------------------------------------------------------   
    
        % select nonlinear least squares algorithm
        switch NLSalg
            
            case {'LS1','kron',1}
            % -------------------------------------------------------------
                theta = zeros( dy, ordD*dy + ordN*du );
                for m = 0:ordD-1 % because D{ii+ordD} = eye
                    theta(:, m*dy + (1:dy) ) = D{ii+m};
                end
                for m = 1:ordN % because N{ii+0} calculated from H(0) constraint
                    theta(:, ordD*dy + (m-1)*du + (1:du) ) = N{ii+m};
                end
                
                % stack data as a vector
                thetatmp = theta(:);
                clear theta
                theta = thetatmp;
                
            
            case {'LS2','det','adj',2, 'LS3','lin',3 }
            % -------------------------------------------------------------   
                theta = zeros( ordD*dy + ordN*du, dy );  
                for m = 0:ordD-1 
                    theta( m*dy + (1:dy), : ) = ( D{ii+m} )';  
                end
                for m = 1:ordN  
                    theta( ordD*dy + (m-1)*du + (1:du), : ) = ( N{ii+m} )';
                end
     
        end
        
        
    case {'right','RIGHT','rmfd','RMFD'}
    % ---------------------------------------------------------------------    
    
        % select nonlinear least squares algorithm
        switch NLSalg
            
            case {'LS1','kron',1}
            % -------------------------------------------------------------   
            % unknown matrix defined as theta = [ D0' D1' ... Dn' N1' N2' ...  N(n+2)' ];
            theta = zeros( du, ordD*du + ordN*dy );
            for m = 0:ordD-1
                theta( :, m*dy + (1:du) ) = ( D{ii+m} )' ;
            end
            for m = 1:ordN
                theta( :, ordD*du + (m-1)*dy + (1:dy) ) = ( N{ii+m} )';
            end
            
            % stack data as a vector
            thetatmp = theta(:);
            clear theta
            theta = thetatmp;
                
            
            case {'LS2','det','adj',2, 'LS3','lin',3 }
            % -------------------------------------------------------------
            % unknown matrix defined as theta = [D0; D1; ... Dn; N1; N2; ... N(n+2)];
            theta = zeros( ordD*du + ordN*dy, du );
            for m = 0:(ordD-1)
                theta( m*du + (1:du), : ) = D{ii+m};
            end
            for m = 1:ordN
                theta( ordD*du + (m-1)*dy + (1:dy), : ) = N{ii+m};
            end
                
        end
        
end