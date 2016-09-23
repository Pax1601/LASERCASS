function [is_stab,Di] = myStabilization(ord,Di,Ni,eigsopt)
% =========================================================================
%                                                           myStabilization
% =========================================================================
%
% Description: stabilization procedure of the identified state space matrix
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

% auxiliary variables
Dord = ord + 1;

dd = size(Di{1},1);
du = size(Ni{1},2);
Id = eye(dd);

% -------------------------------------------------------------------------
% Controllability state-space realization for stabilization purposes
% -------------------------------------------------------------------------
nx = ord*dd;
SS_A = zeros( nx, nx );
SS_B = zeros( nx, du );
for k = 1:Dord-1
    SS_A( dd*(ord-1) + (1:dd) , dd*(k-1) + (1:dd)  ) = -Di{k};
    if k ~= 1
        SS_A( dd*(ord-k)+(1:dd), dd + dd*(ord-k) + (1:dd) ) = Id;
    end
     SS_B( dd*(ord-1) + (1:dd) ,1:dd) = Id;
end

% -------------------------------------------------------------------------
% State space stabilization
% -------------------------------------------------------------------------

% calculate eigenvalues and right eigenvectors
[ReigvecA,ReigvalA] = eig(SS_A,'nobalance'); 

% reorder eigenvalues
[~,reord] = sort(-real(diag(ReigvalA)),'ascend'); 
ReigvalA = diag(ReigvalA(reord,reord));
ReigvecA = ReigvecA(:,reord);

eigvalA = ReigvalA;
fprintf(1,'\nEigenvalues:\n');
fprintf(1,'%14.3e%11.3ei\n',[real(eigvalA), imag(eigvalA)].');

% check stability (eigenvalues having a real part greater than 
% the threshold value are considered unstable)
is_stab = ~sum(real(eigvalA) >= eigsopt.threshold );

if (eigsopt.flagstab) % flag stabilization 
    
    if ( ~is_stab )
        
        % real part of any unstable eigenvalue is set to a stable value
        % -----------------------------------------------------------------
        eigvalA_Re = real(eigvalA);      
        eigvalA_Im = imag(eigvalA);
        
        % search for unstable eigenvalues
        unstablogic = (eigvalA_Re > eigsopt.threshold);
        iunstab     = find( unstablogic == 1 );
        nunstab     = length(iunstab);
        
        eigvalA_Im(iunstab) = 0;
        
        % define counter vector used to increase bound of the threshold value
        % when eigenvalues are not complex conjugate
        counter = zeros(nx,1);
        count = 0;
        for i = 1:nunstab
            counter( iunstab(i) ) = count; % counter = cumsum( unstablogic ) - 1;
            if i < nunstab
                if ( eigvalA_Re(iunstab(i)) == eigvalA_Re(iunstab(i+1)) ) ...
                        && ( eigvalA_Im(iunstab(i)) == -eigvalA_Im(iunstab(i+1)) )  &&  (eigvalA_Im(iunstab(i)) ~= 0)
                else
                    % increase count if unstable and not complex conjugate
                    count = count + 1;
                end
            end
        end
        
        % shifting method
        switch eigsopt.type
            
            case {'bound'}
                % real part of any unstable eigenvalue is set to the user defined value 'eigbound'.
                EpsilonEig = eigsopt.bound;
                eigvalA_Re(iunstab) = EpsilonEig;
                eigvalA_Re(iunstab) = eigvalA_Re(iunstab) + counter(iunstab)*eigsopt.bound; % added to avoid coincident poles
            case {'flip'}
                % flipping all the unstable poles around the imaginary axis.
                EpsilonEig = -1;
                eigvalA_Re(iunstab) = EpsilonEig*eigvalA_Re(iunstab);
                eigvalA_Re(iunstab) = eigvalA_Re(iunstab) + counter(iunstab)*eigsopt.bound; % added to avoid coincident poles
        end
 
        % New eigenvalues
        eigvalA = eigvalA_Re + 1i*eigvalA_Im;
        
        % New state space matrix after eigenvalues and eigenvector modification
        % -----------------------------------------------------------------
        switch eigsopt.method
            
            case 'eigshift'
                % modify eigenvector to maintain the structure of matrix SS_A
                for j = 1:length(iunstab)
                    for k = 1:ord-1
                        ReigvecA( k*dd + (1:dd), iunstab(j) ) = eigvalA(iunstab(j))^k *  ReigvecA( (1:dd), iunstab(j) );
                    end
                end
                SS_A_stab = real( (ReigvecA*diag(eigvalA))/ReigvecA );
                
            case 'polesplace'
                G = place(SS_A,SS_B,eigvalA);
                SS_A_stab = SS_A - SS_B*G;
        end
   
        % check matrix structure after eigenvalues shifting
        % -----------------------------------------------------------------
        % check passed if = 0;
        tol = 1e-8;
        check = 0;
        for k = 1:Dord-1
            check = check + norm( double( isinf( SS_A_stab(dd*(ord-1) + (1:dd), dd*(k-1) + (1:dd)) ) ) );        % check Di blocks
            if k ~= 1
                check = check + ( norm( SS_A_stab(dd*(ord-k) + (1:dd), 1:dd) ) > tol );                          % check zeros blocks
                check = check + ( norm( SS_A_stab( dd*(ord-k)+(1:dd), dd + dd*(ord-k) + (1:dd) ) - Id ) > tol ); % check eye(.) blocks
            end
        end
        if check
            disp('Warning: the state space matrix structure is not mantained after eigenvalues shifting.');
            pause
        end

        % New polynomial matrices
        % -----------------------------------------------------------------
        % for controllability form
        clear Di
        Di = cell(1,ord);
        ii = 1;
        for k = 0:ord-1
            Di{ii+k} = -SS_A_stab( dd*(ord-1) + (1:dd), k*dd + (1:dd) );
            Di{ii+k} = real( Di{ii+k} );
        end
        Di{ii+ord} = Id;
        is_stab = 0; % because the stability has to arise from the minimization and not from the Re[eig]<0 imposition
            
    end
 
end

