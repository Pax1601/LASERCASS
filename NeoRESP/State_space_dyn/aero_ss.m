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

data = load(input);
klist = data.k;
Qhag = data.Ha;
% -------------------------------------------------------------------------
% Rational Matrix Fraction Approximation Parameters
% -------------------------------------------------------------------------
% Identification options

opt{1} = 3;      % MFD order
opt{2} = 3;      % MFD algorithm
opt{3} = 'lmfd'; % left or right MFD
opt{4} = 2;      % residualization order
opt{5} = 100;      % weight parameter value W^2

%opt{1} = 4;      % MFD order
%opt{2} = 3;      % MFD algorithm
%opt{3} = 'lmfd'; % left or right MFD
%opt{4} = 2;      % residualization order
%opt{5} = 100;      % weight parameter value W^2
% Levenberg-Marquardt parameters
tau     = 1e-3;
tolg    = 1e-4;
tolx    = 1e-6;
maxiter = 100;
optsLM = [tau tolg tolx maxiter];

% stabilizations parameters ATR
%eigsopt.threshold = -1e-2;
%eigsopt.type      = 'bound';      % 'bound' or 'flip'
%eigsopt.bound     =  -1.5e-1;
%eigsopt.method    = 'polesplace'; %'polesplace' or 'eigshift'


eigsopt.threshold = -1e-3;
eigsopt.type      = 'bound';      % 'bound' or 'flip'
eigsopt.bound     =  -1.5e-1;
eigsopt.method    = 'polesplace'; %'polesplace' or 'eigshift'

% Model reduction algorithm
algROM = 'bst'; % 'schur', 'hankel', 'bst'

restart = [];
%
solution = improvedMFDfun(klist,Qhag,opt,optsLM,eigsopt,algROM,restart);

%W = ones(1,length(klist));
%W(1) = 10
%W(2) = 10

%[D0,D1,D2,A,B,C] = ms_pade(Qhag,1,klist,W,1);
%Qhag(:,:,1,1)

%solution.inoutresid.AA = A;
%solution.inoutresid.BB{1} = B;
%solution.inoutresid.BB{2} = zeros(size(B,1),size(B,2));
%solution.inoutresid.BB{3} = zeros(size(B,1),size(B,2));
%solution.inoutresid.CC = C;
%solution.inoutresid.DD{1} = D0;
%solution.inoutresid.DD{2} = D1;
%solution.inoutresid.DD{3} = D2;
%nXa = length(A)
%for i=1:length(klist)
%p=1i*klist(i);
%EEs = D0 + p*D1 + p^2*D2;
%HHid(:,:,i) = C*( (p*eye(nXa) - A)\ B ) + EEs;
%end
%HHid(:,:,2)
%figure(1); close; figure(1); hold on
%plot(squeeze(real(Qhag(1,1,:))),squeeze(imag(Qhag(1,1,:))),'-ko');
%plot(squeeze(real(HHid(1,1,:))),squeeze(imag(Qhag(1,1,:))),'ro');

%Aid    = solution.inoutresid.AA;
%Bid{1} = solution.inoutresid.BB{1};
%Bid{2} = solution.inoutresid.BB{2};
%Bid{3} = solution.inoutresid.BB{3};
%Cid    = solution.inoutresid.CC;
%Eid{1} = solution.inoutresid.DD{1};
%Eid{2} = solution.inoutresid.DD{2};
%Eid{3} = solution.inoutresid.DD{3};
save(outfile,'solution');
end