%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

%
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%  MLS_INTERFACE Mean Least Square conservative inteface matrix assembly
%   
%	Usage:
%		H = MLS_INTERFACE(str_data, aero_data, poly, weight, points, rmax, tcond)
%
%	Input:        
%		str_data	matrix with the structural nodes space position [Node_number x Dimension]
%                   [ x1 y1 z1;
%                     x2 y2 z2;
%                   ...]
%       aero_data	matrix with the aerodynamic nodes space position [Node_number x Dimension]
%                   [ x1 y1 z1;
%                     x2 y2 z2;
%                   ...]
%   
%       poly	order of the polynomial base function 
%               1 (linear), 2 (quadratic)
%
%       weight	Radial Basis Function weight (1:4)
%
%       points	number of points to be included in the support
%
%       rmax	maximum radius for compact suport domain 
%
%       tcond 	maximum condition number accepted for the system matrix. 
%              	Useful for ill-conditioned problems (typical value 1.0E+5)
%
%	Output:        
% 		H	sparse interface matrix 
%
%  	This function requires knearn mex file under Unix OS or interface executable under Windows OS
%	
%	Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************

function H = mls_interface(str_data, aero_data, poly, weight, points, rmax, tcond) 

path = neocass_path;
%interf_path = fullfile(path, 'interface', 'interface.exe'];

DEFAULT_POLY = 2;
DEFAULT_WEIGHT = 2;
DEFAULT_TCOND = realmax;
DEFAULT_RADIUS = realmax;

strN = size(str_data, 1);
aerN = size(aero_data, 1);
dim = size(str_data, 2);

if (dim ~= size(aero_data, 2))
    fprintf('\n Inconsistent data space dimensions.');
    return;
end

if (poly <= 0) || (poly > 2) 
    fprintf('\n Wrong polynomial order. Correct values are 1 (linear) or 2 (parabolic). Value set to %d by default.', DEFAULT_POLY);
	poly = DEFAULT_POLY;
end

if (weight <= 0) || (weight > 4) 
    fprintf('\n Wrong Radial Basis weight order. Correct values are between 1 and 4. Value set to %d by default.', DEFAULT_WEIGHT);
	weight = DEFAULT_WEIGHT;
end

if (rmax == 0)
    fprintf('\n No rmax value required. Value set to Inf.');
	rmax = DEFAULT_RADIUS;
end

if (tcond == 0)
    fprintf('\n No condition value inserted. Value set to %f by default.', DEFAULT_TCOND);
	tcond = DEFAULT_TCOND;
end

switch poly
	
	case 1 
        min_p = 1+dim;
    
    case 2
        min_p = 1+(dim)^2;
end

if (points < min_p) 
	fprintf('\n Required search points too smal. Value set to %d.', min_p);
	points = min_p; 
end

if (points > strN)
	fprintf('\n Required too many search points. Value set to %d.', strN);
	points = strN; 
end

if isunix
	% call ANN library using mex file for Unix OS
	[nnpos,dist] = knearn(str_data,aero_data, dim, points);
else
	% launch executable for Windows OS based on ANN library
	% export parameters for interface
	fp = fopen('param.int','w'); 
	fprintf(fp,'SPACE_DIMENSION: %d\n',dim);
	fprintf(fp,'K_POINTS: %d', points);
	fclose(fp);
	% export basic data
	fp = fopen('basic_input.int','w'); 
	fprintf(fp,'BASIC_NODES: %d\n', length(str_data));
	fprintf(fp,'%g %g %g\n', str_data');
	fclose(fp);
	% export query data
	fp = fopen('query_input.int','w'); 
	fprintf(fp,'QUERY_NODES: %d\n', length(aero_data));
  fprintf(fp,'%g %g %g\n', aero_data');
	fclose(fp);
	% call for ANN routine interface.exe
	system(interf_path); 
	nnpos = load('nnpos.int');
	dist = zeros(aerN,points);
	fp = fopen('dist.int','r');
	dist =(fscanf(fp,'%g ', [points aerN]))';
	fclose(fp);
	% clean working dir 
    system('del nnpos.int');
    system('del dist.int');
    system('del basic_input.int');
    system('del query_input.int');
    system('del param.int');
end

dist = sqrt(dist);

r = zeros(aerN*points,1);
c = zeros(aerN*points,1);
h = zeros(aerN*points,1);
coef = 0;
toll = 1/tcond;

for i = 1:aerN
    
    nloc = sum(dist(i,:) < rmax);
	if ( nloc < min_p) 
		nloc = min_p;
		fprintf('\nInserted radius too small (rmax parameter). Compact suport enlarged to minimum size.');
	end
    
    [P, pp] = mls_poly(str_data(nnpos(i,1:nloc),:), poly, aero_data(i,:));
    W = mls_wgt(dist(i,1:nloc), weight);
    b = P' * W;
    A = P' * W * P;
    r(coef+1:coef+nloc) = i*ones(1, nloc);
    c(coef+1:coef+nloc) = nnpos(i,1:nloc);
%    
    % [u,s,v] = svd(A);
    % s = diag(s);
    % maxs = s(1);
    % get system matrix condition number     
    % s = s./maxs;
    % sv = find(s < toll);
    % nsv = length(sv);
    % build interface coefficients
    % h(coef+1:coef+nloc) = (pp*pinv(A, s(nsv))*b)';
%
    h(coef+1:coef+nloc) = (pp*pinv(A, toll)*b)';
   
    coef = coef + nloc;
end

r = r(1:coef);
c = c(1:coef);
h = h(1:coef);

H = sparse(r, c, h, aerN, strN);

%***********************************************************************************************************************

function [P, pp] = mls_poly(str_data, type, aer_data)
    
N = size(str_data,1);
dim = size(str_data,2);
P = zeros(N, 1 + (dim)^type);
pp = zeros(1, 1 + (dim)^type);

% re-scale nodes coordinates to avoid matrix ill-conditioning
cg = mean(str_data);
str = zeros(N, dim);
for (j=1:N) 
    str(j,:) = str_data(j,:) - cg;
end
aer = aer_data - cg;

switch (type)

    case 1            
        P = [ones(N,1), str];
        pp(1) = 1;
        pp(2:dim+1) = aer; 

    case 2
        P = [ones(N,1), str];
        pp(1) = 1;
        pp(2:dim+1) = aer; 
        k = dim+1;
        for i = 1:dim
            for j = i:dim
                k = k+1;
                P(:,k) = str(:,i) .* str(:,j);
                pp(k) = aer(:,i) * aer(:,j);
            end
        end

end
    
%***********************************************************************************************************************
    
function W = mls_wgt(dist, type)
        
mx = 1.1 * max(dist); 
switch (type)

    case 1 
        W = diag((1 - dist/mx).^2);

    case 2
        W = diag((1 - dist/mx).^4 .* (4*dist/mx+1));

    case 3
        W = diag((1 - dist/mx).^6 .* (35/3*(dist/mx).^2 + 18/3*(dist/mx)+1));

    case 4 
        W = diag((1 - dist/mx).^8 .* (32*(dist/mx).^3+25*(dist/mx).^2 + 8*(dist/mx)+1));

end
