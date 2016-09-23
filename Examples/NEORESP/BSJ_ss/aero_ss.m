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
%   Author: Luca Cavagna
%
% The template of this solver comes from M. Ripepi 
% The code has been adapted to NeoRESP
%
function aero_ss(input, outfile)
%
data = load(input);
klist = data.k;
Qhag = data.Ha;
% -------------------------------------------------------------------------
% Rational Matrix Fraction Approximation Parameters
% -------------------------------------------------------------------------
% Identification options
%
% MOTION
%
opt{1} =3;      % MFD order
opt{2} = 3;      % MFD algorithm
opt{3} = 'rmfd'; % left or right MFD
opt{4} = 2;      % residualization order
opt{5} = 100;      % weight parameter value W^2
%
% GUST
%
%opt{1} = 8;      % MFD order
%opt{2} = 3;      % MFD algorithm
%opt{3} = 'rmfd'; % left or right MFD
%opt{4} = 2;      % residualization order
%opt{5} = 100;      % weight parameter value W^2

% Levenberg-Marquardt parameters
tau     = 1e-3;
tolg    = 1e-4;
tolx    = 1e-6;
maxiter = 100;
optsLM = [tau tolg tolx maxiter];
%
eigsopt.threshold = -5e-2;
eigsopt.type      = 'bound';      % 'bound' or 'flip'
eigsopt.bound     =  -1.5e-1;
eigsopt.method    = 'polesplace'; %'polesplace' or 'eigshift'
% Model reduction algorithm
algROM = 'bst'; % 'schur', 'hankel', 'bst'
%
restart = [];
solution = improvedMFDfun(klist,Qhag,opt,optsLM,eigsopt,algROM,restart);
save(outfile,'solution');
%
end