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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     120210      ?       L.Travaglini     Creation
%
%**************************************************************************
%*****
% this function computes the value of flutter constraint
% IN    --> structure contains solution of flutter for the mach andeigenvalue computed
% curve --> name of file holding spline information on reject curve on Real/Imag*2pi 
function [f,v,IND] = flutter_cstr(IN,curve,NORM)

Sclim = load(curve);
name = fieldnames(Sclim);
Sclim = getfield(Sclim,name{1});
c = [IN.ris(:,2) , IN.ris(:,3)];
clim = ppval(Sclim,c(:,1));

d = (c(:,2)-clim); %normalized

% % % % % % % % il controllo deve essere verificato no no... meglio farlo fuori..
% % % % % % % 
% % % % % % % if length(find(d>=0))>= round(N/3*2) &&  max(abs(diff(c(:,2))./diff(c(:,1)) )) < 2e-3 && max(abs(diff(IN.ris(:,4))./diff(c(:,1)) ))<5e-4
% % % % % % %     f = -1;
% % % % % % % else
% % % % % % %     
% % % % % % % end

% firstly find the zeros (if exist) of d.

d(abs(d)<=1e-10) = 0;

ind = find(diff(sign(d))~=0); % ind where d changes sign

if isempty(ind)
    v = IN.ris(:,2);
    f = (trapz(IN.ris(:,2),d))/NORM;
    IND = [1,length(d)];
    
else
    nz = length(ind);
    dpl = spline(IN.ris(:,2),d);
    zeri = zeros(nz,1);
    zeri2 = zeri;
    fun = @(x)zerofind(x,dpl);
    for i = 1 : nz
        zeri(i) = fzero(fun,IN.ris(ind(i),2));
        zeri2(i) = fzero(fun,IN.ris(ind(i)+1,2));
    end
    zeri = union(zeri,zeri2);
    control = size(zeri);
    if control(2)>1
        zeri = zeri';
    end
    control = find(abs(diff(zeri))>1e-4);
    zeri = [zeri(1);zeri(control+1)];
    % now let compute areas of each part
    % add zero value to vector d
    %     d = union(d,ppval(dpl,zeri));
    [v,sortindv] = sort([IN.ris(:,2);zeri]);
    d2 = [d;ppval(dpl,zeri)] ;
    d = d2(sortindv);
    N = length(d);
    
    d(abs(d)<=1e-6) = 0;
    ind = find(d==0);
    npart = length(ind)+1;
    if ind(1) ==1
        npart = npart-1;
        ind = ind(2:end);
    end
    if ind(end) == N
        npart = npart-1;
        ind = ind(1:end-1);
    end
    if npart == 0        
        f = (trapz(v,d))/NORM;
        IND = [1,N];
    else
        F = zeros(npart,1);
        F(1) = trapz(v(1:ind(1)),d(1:ind(1)));
        
        for i = 2 :npart-1
            F(i) = trapz( v(ind(i-1):ind(i)), d(ind(i-1):ind(i)) );
        end
        F = [F(1:end-1); trapz( v(ind(end):end), d(ind(end):end) )];
%         [f,indf] = max(F);
%         f = f/NORM;
%         if indf == npart
%             IND = [ind(end), N];
%         elseif indf == 1
%             IND = [1,ind(1)];
%         else
%             IND = [ind(indf-1),ind(indf)];
%         end
       indf = find(F>=0);
       f = sum(F(indf))/NORM;
       IND = []; 
       for ij = 1 : length(indf)
           if indf(ij) == npart
               IND = [IND,ind(end), N];
           elseif indf(ij) == 1
               IND = [IND,1,ind(1)];
           else
               IND = [IND,ind(indf(ij)-1),ind(indf(ij))];
           end
       end
    end
    
    
end
% now 

end
function f = zerofind(y,dpl)
f = ppval(dpl,y);
end