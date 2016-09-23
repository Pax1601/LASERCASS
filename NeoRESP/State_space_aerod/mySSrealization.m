function [E0,E1,E2,SS_A,SS_B,SS_C] = mySSrealization(ord,ro,Di,Ni,type)
% =========================================================================
%                                                           mySSrealization
% =========================================================================
%
% Description: contruct a canonical (controllable or observable) state 
%              space realization, given the MFD matrices D and N. 
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
Dord = ord  + 1;
Nord = Dord + ro;

dy = size(Ni{1},1);
du = size(Ni{1},2);
dd = size(Di{1},1);
Iy = eye(dy);
Iu = eye(du);

% -------------------------------------------------------------------------
% Reformulate in terms of H = E0 + p*E1 + p^2*E2 + D\R  or  H = E0 + p*E1 + p^2*E2 + R/D
% -------------------------------------------------------------------------
Ri = cell(1,ord);
switch type
    
    case {'lmfd','LMFD','left','obs','OBS','Obs'}
        
      if ro==2
        % calculate E,R for LMFD case H = E0 + p*E1 + p^2*E2 + D\R
        E2 = Di{Dord}\Ni{Nord};
        E1 = Di{Dord}\( Ni{Nord-1} - Di{Dord-1}*E2 );
        if Dord > 2
            E0 = Di{Dord}\( Ni{Dord} - Di{Dord-2}*E2 - Di{Dord-1}*E1 );
        else
            E0 = Di{Dord}\( Ni{Dord} - Di{Dord-1}*E1 );
        end
        
        ii = 1;
        Ri{ii+0} = Ni{ii+0} - Di{ii+0}*E0;                % R0
        Ri{ii+1} = Ni{ii+1} - Di{ii+0}*E1 - Di{ii+1}*E0;  % R1
        for k = 2:ord-1
            Ri{ii+k} = Ni{ii+k} - Di{ii+k-2}*E2 - Di{ii+k-1}*E1 - Di{ii+k}*E0;
        end
%
      else
        % calculate E,R for LMFD case H = E0 + p*E1 + D\R
        E2 = zeros(dy,du);
        E1 = Di{Dord}\Ni{Nord};
        E0 = Di{Dord}\( Ni{Dord} - Di{Dord-1}*E1 );
%        
        ii = 1;
        Ri{ii+0} = Ni{ii+0} - Di{ii+0}*E0;                % R0
        for k = 1:ord-1
            Ri{ii+k} = Ni{ii+k} - Di{ii+k-1}*E1 - Di{ii+k}*E0;
        end
%
      end
        
    case {'rmfd','RMFD','right','ctr','Ctr','CTR'}
        
      if ro==2
      % calculate E,R for RMFD case H = E0 + p*E1 + p^2*E2 + R/D
        E2 = Ni{Nord}/Di{Dord}; 
        E1 = ( Ni{Nord-1} - E2*Di{Dord-1} )/Di{Dord};
        if Dord > 2
            E0 = ( Ni{Dord} - E2*Di{Dord-2} - E1*Di{Dord-1} )/Di{Dord};
        else
            E0 = ( Ni{Dord} - E1*Di{Dord-1} )/Di{Dord};
        end
        
        ii = 1;
        Ri{ii+0} = Ni{ii+0} - E0*Di{ii+0};               % R0
        Ri{ii+1} = Ni{ii+1} - E1*Di{ii+0} - E0*Di{ii+1}; % R1
        for k = 2:ord-1
            Ri{ii+k} = Ni{ii+k} - E2*Di{ii+k-2} - E1*Di{ii+k-1} - E0*Di{ii+k};
        end
%
      else
      % calculate E,R for RMFD case H = E0 + p*E1 + R/D
        E2 = zeros(dy,du);
        E1 = Ni{Nord}/Di{Dord};
        E0 = ( Ni{Dord} - E1*Di{Dord-1} )/Di{Dord};
%        
        ii = 1;
        Ri{ii+0} = Ni{ii+0} - E0*Di{ii+0};                % R0
        for k = 1:ord-1
            Ri{ii+k} = Ni{ii+k} - E1*Di{ii+k-1} - E0*Di{ii+k};
        end
%
      end        
    otherwise
        
        error('Option not allowed. Choose between: LMFD/RMFD or obs/ctr');
        
end

% -------------------------------------------------------------------------
% State-Space Realization
% -------------------------------------------------------------------------
nx = ord*dd;
SS_A = zeros( nx, nx );
SS_B = zeros( nx, du );
SS_C = zeros( dy, nx );

switch type
    
    case {'lmfd','LMFD','left','obs','OBS','Obs'}
        
        % observability form
        for k = ord:-1:1
            SS_A( dy*(ord-k) + (1:dy), (1:dy) ) = -Di{k};
            if k ~= 1
                SS_A( dy*(ord-k) + (1:dy), dy + dy*(ord-k) + (1:dy) ) = Iy;
            end
            SS_B( dy*(ord-k) + (1:dy), (1:du) ) = Ri{k};
        end
        SS_C( 1:dy, 1:dy ) = Iy;
        
        
    case {'rmfd','RMFD','right','ctr','Ctr','CTR'}
        
        % CHECK structure of SS_C, if wrong...
        % controllability form
        for k = 1:ord
            SS_A( (1:du) , du*(k-1) + (1:du)  ) = -Di{Dord-k};
            if k ~= 1
                SS_A( du*(k-1) + (1:du),  du*(k-2) + (1:du) ) = Iu;
            end
            SS_C( (1:dy), du*(k-1) + (1:du) ) = Ri{Dord-k};
        end
        SS_B( 1:du, 1:du ) = Iu;
        
    otherwise
        
        error('Option not allowed. Choose between: LMFD/RMFD or obs/ctr');
        
end
