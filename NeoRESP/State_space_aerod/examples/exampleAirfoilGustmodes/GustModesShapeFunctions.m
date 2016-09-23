function [N,xq] = GustModesShapeFunctions(x,xnodes,mode)
% =========================================================================
%                      GUST MODES SHAPE FUNCIONS
% =========================================================================
%
% Description: linear and quadratic gust modes shape functions
%
% -------------------------------------------------------------------------
%
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

M = length(x);
Nnodes = length(xnodes);

if strcmp(mode,'lin')
    N  = zeros(M,Nnodes);
elseif strcmp(mode,'quad')
    N  = zeros(M,2*Nnodes-1);
end
% ATTENTION: in case of quadratic shape functions the nodes are only the
% boundary nodes defining the elements, and not the inerior nodes

for i=1:M
    
    [~, pos] = min(abs(x(i)-xnodes));
    if (pos==1)
        pos_lb = pos;
    elseif (pos==Nnodes)
        pos_lb = pos-1;
    else
        if  x(i) > xnodes(pos) %(abs( xnodes(pos+1) - x(i) ) < abs( xnodes(pos-1) - x(i)))
            pos_lb = pos;
        else
            pos_lb = pos-1;
        end
    end
    
    if strcmp(mode,'lin')
        
        % piecewise linear
        xloc = ( x(i) - xnodes(pos_lb) )./( xnodes(pos_lb+1) - xnodes(pos_lb) ); %Lelem;
        N0 = 1 - xloc;
        N1 = xloc;
        N(i,pos_lb+(0:1))  = [N0 N1];
        
    elseif strcmp(mode,'quad')
        
        % quadratic
        xloc = 2*( x(i) - xnodes(pos_lb) )./( xnodes(pos_lb+1) - xnodes(pos_lb) ) -1; %Lelem;
        N0 = 0.5*xloc*(xloc-1);
        N1 = -xloc^2+1;
        N2 = 0.5*xloc*(xloc+1);
        N(i,2*pos_lb-1+(0:2))  = [N0 N1 N2];
        
    end
    
end

if strcmp(mode,'lin')
    
    xq = xnodes;
    
elseif strcmp(mode,'quad')
    
    xq = xnodes(1);
    for i=1:Nnodes-1
        xq = [xq (xnodes(i)+xnodes(i+1))/2 xnodes(i+1)];
    end
    
end

% for k = 1:length(t)
%     q(:,k) = interp1(x,Vg(:,k),xq,'linear','extrap')';
% end