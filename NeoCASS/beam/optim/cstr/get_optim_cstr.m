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

function [CIN, CEQ] = get_optim_cstr(X, beam_model)
%
CIN = [];
CEQ = [];
CT_IN = beam_model.Optim.Cstr.In.CT;
CT_EQ = beam_model.Optim.Cstr.Eq.CT;

if (~isempty(beam_model.Optim.DCSTR_IN))
  N = length(beam_model.Optim.CSTR_IN);
  CIN = zeros(N,1);
  for i=1:N
    CIN(i) = beam_model.Optim.CSTR_IN(i) - beam_model.Optim.Cstr.In.Value(i);
    index = find(beam_model.Optim.DCSTR_IN(i,:)>0);
    for k=1:length(index)
      n = index(k);
      CIN(i) = CIN(i) + beam_model.Optim.DCSTR_IN(i,n) * (X(n) - beam_model.Optim.X(n));
    end
    index = find(beam_model.Optim.DCSTR_IN(i,:)<0);
    for k=1:length(index)
      n = index(k);
      CIN(i) = CIN(i) + (beam_model.Optim.X(n)^2) * beam_model.Optim.DCSTR_IN(i,n) * (1/X(n) - 1/beam_model.Optim.X(n));
    end
  end
%
end

%CI1=CIN;
%CIN=[];

if (~isempty(beam_model.Optim.DCSTR_IN))
  CIN = beam_model.Optim.CSTR_IN + beam_model.Optim.DCSTR_IN * (X - beam_model.Optim.X)'...
        - beam_model.Optim.Cstr.In.Value';
end

%CI2=CIN;%

%size(CI2)


%size(CI1)
%a=[CI1,CI2];
%size(a)
%disp(a)


%

if (~isempty(beam_model.Optim.DCSTR_EQ))
  CEQ = beam_model.Optim.CSTR_EQ + beam_model.Optim.DCSTR_EQ * (X - beam_model.Optim.X)' ...
        - beam_model.Optim.Cstr.Eq.Value';
end
%
end
