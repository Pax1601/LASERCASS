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
% Recover bar/beam stress at recovery points
%
function [CStressesN, CStressesT, SafeM_Tmax_Norm, SafeM_Tmin_Norm, SafeM_SM_Norm, ...
          SafeM_Tmax_Shear, SafeM_Tmin_Shear, SafeM_SM_Shear, SBUCK, PBUCK] = stress_recovery(F, p, PBar, Mat, DP)

	CStressesN = zeros(1,4);
	CStressesT = zeros(1,4);
  SafeM_Tmax_Norm  = 0.0;
  SafeM_Tmin_Norm  = 0.0;
  SafeM_SM_Norm    = 1.0;
  SafeM_Tmax_Shear = 0.0;
  SafeM_Tmin_Shear = 0.0;
  SafeM_SM_Shear   = 1.0;
  CSN = zeros(2,4);
  CST = zeros(2,4);
  SBUCK = 1.0;
  PBUCK = 1.0;

   % Determine stress status
    if (norm(PBar.Str_point(:,:,p)) ~= 0)
      AREA = PBar.A(p);
      Iy = PBar.I(p,2); Iz = PBar.I(p,1); Ixy = PBar.I(p,3); 
      J = PBar.J(p); 
      iyz = Iy-Iz;
      if (iyz)
        alpha = 0.5 * atan(2*Ixy) / iyz;
      else
        alpha = 0;      
      end
      Izp = Iz * cos(alpha)^2 + Iy * sin(alpha)^2 - Ixy * sin(2*alpha);
      Iyp = Iy * cos(alpha)^2 + Iz * sin(alpha)^2 + Ixy * sin(2*alpha);
      R = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)];
      FP = zeros(3,1); MP = zeros(3,1);
      Mcrush = 0;
      Mtmean = 0;
      Tymean = 0;
      Tzmean = 0;
%     Common to all PBAR types: determine normal stresses
      for j=1:2 % loop on collocation points
        offset = 6*(j-1);
        FP(2:3) = R * F([2 3] + offset); FP(1) = F(1+offset); % axial force
        MP(2:3) = R * F([5 6] + offset); MP(1) = F(4+offset); % torsional torque
        % recover normal stress
        for k=1:4 % loop on nodes
          ARM = R * PBar.Str_point(k,1:2,p)';
          CSN(j,k) = FP(1)/AREA + MP(2)*ARM(2)/Iyp - MP(3)*ARM(1)/Izp;       
        end
        Mcrush = Mcrush + MP(3); % used only by nultiweb beam (see type 3 below)
        Mtmean = Mtmean + MP(1);
        Tymean = Tymean + FP(2);
        Tzmean = Tzmean + FP(3);
      end
%
      Mcrush = abs(Mcrush/2);
      Mtmean = Mtmean/2;
      Tymean = Tymean/2;
      Tzmean = Tzmean/2;
%
      CStressesN = mean(CSN);
%     Monocoque sections: determine tangential stresses
      t_av = false;
      switch (PBar.Type(p))

        case {10} % symmetric wing box
          t_av = true;
          si = PBar.SI(p); % access to section data
          data = PBar.Section(si).data;

          Acorner = data(1); tskin = data(2); chord = data(3); h = data(4);
          tweb = tskin;
%
          omega = chord*h; Shweb = tweb*(h/2)*(h/4); Shskin = tskin*(chord/2)*(chord/4);
          Aweb  = tweb * h;
          Askin = tskin * chord;
          for j=1:2 % loop on collocation points
            offset = 6*(j-1);
            FP(2:3) = R * F([2 3] + offset); % shear forces
            MP(1) = F(4+offset); % torsional torque
            torque_sign = [-1 1 1 -1];
            % recover tangential stress on webs. Each web resists to half shear force along y section axis
            for k=1:2:4 % loop on nodes
              ARM = R * PBar.Str_point(k,1:2,p)';
              if (norm(ARM)) % avoid calculation if the point is not given
                CST(j,k) = torque_sign(k) * MP(1)/(2*omega*tweb) + (3/2)*FP(2)/(2*Aweb);     
              end
            end
            % recover tangential stress on skin. Each skin resists to half shear force along z section axis
            for k=2:2:4 % loop on nodes
              ARM = R * PBar.Str_point(k,1:2,p)';
              if (norm(ARM)) % avoid calculation if the point is not given
                CST(j,k) = torque_sign(k) * MP(1)/(2*omega*tskin) + (3/2)*FP(3)/(2*Askin);     
              end
            end
          end
          %
          % BUCKLING safety margin CALCULATION on stringers and skin panels
          %
          if (PBar.Type(p) == 11)
            % stringer buckling for the 
            str_stress = [CStressesN];
            index =find(str_stress<0);
            if (~isempty(index))
              J = min(PBar.Section(si).data(11), PBar.Section(si).data(12));
              mins = min(str_stress(index));
              % Eulerian global buckling
              SBUCK = Mat.E(PBar.Mat(p)) * J * (pi^2) / (AREA * PBar.Section(si).data(8)^2);
              SBUCK = -SBUCK/mins -1;
              % Compressed panel global buckling
              b = PBar.Section(si).data(4)/(PBar.Section(si).data(7)-1);
              KC = get_panel_k(PBar.Section(si).data(8),b);
              PBUCK = Mat.E(PBar.Mat(p)) * KC * ((PBar.Section(si).data(3)/b)^2) * (pi^2) / (12*(1-Mat.nu(PBar.Mat(p)))^2);
              PBUCK = -PBUCK/mins -1;
            end

          end
%-------------------------------------------------------------------------------
% FUSELAGE SECTIONS
%
        case {1,2} % fuselage torque tube framed/unframed using GUESS params
%
          t_av = true;
          si = PBar.SI(p); % access to section data
          if (PBar.Type(p) == 1)
            MR = PBar.Section(si).data(2);     % radius
            BM = PBar.Section(si).data(3);     % skin buckling exponent factor
            BEPS = PBar.Section(si).data(4);   % skin buckling multiply factor
            KPRES = PBar.Section(si).data(6);  % factor for pressure hoop stress
            tskin = PBar.Section(si).data(1)*PBar.Section(si).data(5); % skin equivalent thickness
          else
            AF = PBar.Section(si).data(2);     % frame area
            FSPACE = PBar.Section(si).data(3); % frame spacing
            MR = PBar.Section(si).data(4);     % radius
            BM = PBar.Section(si).data(5);     % skin buckling exponent factor
            BEPS = PBar.Section(si).data(6);   % skin buckling multiply factor
            KPRES = PBar.Section(si).data(8);  % factor for pressure hoop stress
            CRITIC_CF = 1/PBar.Section(si).data(9);  % Shanley's constant inverse
            CKF = PBar.Section(si).data(10);   % frame constant ( Jflex / Area^2)
            EF = PBar.Section(si).data(12);    % frame Young modulus
            tskin = PBar.Section(si).data(1)*PBar.Section(si).data(7); % skin equivalent thickness
          end
%
          omega = pi * MR^2; 
%         Dmax = 2*(MR + tskin/2); Dmin = 2*(MR - tskin/2); Scirc = 1/12 * (Dmax^3 - Dmin^3);
          Scirc = 2 * tskin * MR^2;
          Jsec = PBar.I(p,1);
%
          for j=1:2 % loop on collocation points
            offset = 6*(j-1);
            FP(2:3) = R * F([2 3] + offset); % shear forces
            MP(1) = F(4+offset); % torsional torque
            torque_sign = [-1 1 1 -1];
            % points along y section axis
            for k=1:2:4 % loop on nodes
              ARM = R * PBar.Str_point(k,1:2,p)';
              if (norm(ARM)) % avoid calculation if the point is not given (use Jourawsky's formula)
                CST(j,k) = (torque_sign(k) * MP(1)/(2*omega) + Scirc*FP(2)/(2*Jsec))/tskin;     
              end
            end
            % points along z section axis
            for k=2:2:4 % loop on nodes
              ARM = R * PBar.Str_point(k,1:2,p)';
              if (norm(ARM)) % avoid calculation if the point is not given (use Jourawsky's formula)
                CST(j,k) = (torque_sign(k) * MP(1)/(2*omega) + Scirc*FP(3)/(2*Jsec))/tskin;     
              end
            end
          end
%         relieve normal stresses thanks to cabin pressure
          if (DP >0)
            sigmap = pi*MR*DP/(2*tskin);
            CStressesN = CStressesN + sigmap;
            % calculate hoop stress
            sigmap = MR * DP * KPRES / tskin;
            [r1,c1,v1] = find(CStressesN>0);
            for k=1:length(r1)
              if (CStressesN(r1,c1) < sigmap)
                CStressesN(r1,c1) = sigmap;
              end
            end          
          end
          str_stress = [CStressesN];
          index = find(str_stress<0);
%
          if (PBar.Type(p) == 1) % UNFRAMED BEAM
            if (~isempty(index))
              % skin buckling
              SBUCK = BEPS * Mat.E(PBar.Mat(p)) * tskin^(BM-1) * MR^(1-BM);
              mins = min(str_stress(index));
              SBUCK = -SBUCK/mins -1;
            end
          else                   % FRAMED BEAM
            if (~isempty(index))
              % skin buckling
              mins = min(str_stress(index));
              SBUCK = BEPS * Mat.E(PBar.Mat(p)) * tskin^(BM-1) * FSPACE^(1-BM);
              SBUCK = -SBUCK/mins -1;
              % frame buckling
              maxs = max(abs(str_stress));
              INVCF = (pi * maxs * tskin * 4*MR^4) / (CKF * EF * FSPACE * AF^2); % Actual Shanley's constant inverse
              PBUCK = CRITIC_CF / INVCF -1; % use Shanley's constant for buckling safety margin
            end
          end
%-------------------------------------------------------------------------------
% LIFTING SURRFACE
%
        case {3} % multi-web box beam
%
            si = PBar.SI(p); % access to section data
            str_stress = [CStressesN];
            index = find(str_stress<0);
            Ks = PBar.Section(si).data(7);
            Kw = PBar.Section(si).data(6);
            ts = PBar.Section(si).data(2)/Ks;
            tw = PBar.Section(si).data(1)/Kw;
            dw = PBar.Section(si).data(3);
            h =  PBar.Section(si).data(5);
            chord = PBar.Section(si).data(4);
            expcoeff = PBar.Section(si).data(10);
            vareps = PBar.Section(si).data(11);
            expcoeffw = PBar.Section(si).data(8);
            varepsw = PBar.Section(si).data(9);
            nw = round(chord/dw)+1;
            %
            if (~isempty(index))
              mins = min(str_stress(index));
              % Panel buckling
              SBUCK = (Mat.E(PBar.Mat(p)) * dw * vareps / ts) * (ts/dw)^expcoeff;
              SBUCK = -SBUCK/mins -1;
            end
            % web flexure induced crushing
            expcoeffw = PBar.Section(si).data(8);
            varepsw = PBar.Section(si).data(9);
            PBUCK = Mat.E(PBar.Mat(p))*h*h*chord*(((tw/h)^expcoeffw) / ((2/varepsw)*(vareps*dw/h)^(1/expcoeff)))^(expcoeff/(2*expcoeff-1));
            PBUCK = -Mcrush/PBUCK +1;
            %
            str = smonoq_flux_mweb(h, chord, nw-2, Izp, ts, tw, Mat.G(PBar.Mat(p)), Tymean, Tzmean, Mtmean);
            t_av = false;
            MAXS = Mat.SS(PBar.Mat(p));
            indexu = find(str>0);
            indexl = find(str<0);
            %
            if (isempty(indexu))     
               M1 = realmax;
            else
               SafeM_Tmax_Shear = max(str(indexu));      
               M1 = MAXS/SafeM_Tmax_Shear -1;
            end
            if (isempty(indexl))     
               M2 = realmax;
            else
              SafeM_Tmin_Shear = min(str(indexl));      
              M2 = -MAXS/SafeM_Tmin_Shear -1;
            end
            if (MAXS)
              SafeM_SM_Shear = min(M1, M2);
            end            

          %
          % ADD here other sections
          %
      end
      % set safety margins
      MAXT = Mat.ST(PBar.Mat(p)); MAXC = Mat.SC(PBar.Mat(p)); MAXS = Mat.SS(PBar.Mat(p));
      str = [CStressesN];
      indexu = find(str>0);
      indexl = find(str<0);
        %
      if (isempty(indexu))     
         M1 = realmax;
      else
         SafeM_Tmax_Norm = max(str(indexu));      
         M1 = MAXT/SafeM_Tmax_Norm -1;
      end
      if (isempty(indexl))     
         M2 = realmax;
      else
        SafeM_Tmin_Norm = min(str(indexl));      
        M2 = -MAXC/SafeM_Tmin_Norm -1;
      end
      if (MAXT && MAXC)
        SafeM_SM_Norm = min(M1, M2);
      end            
      %
      if (t_av) % already set if monocoque is used (see type 3)
        CStressesT = mean(CST);
        str = [CStressesT];
        indexu = find(str>0);
        indexl = find(str<0);
        %
        if (isempty(indexu))     
           M1 = realmax;
        else
           SafeM_Tmax_Shear = max(str(indexu));      
           M1 = MAXS/SafeM_Tmax_Shear -1;
        end
        if (isempty(indexl))     
           M2 = realmax;
        else
          SafeM_Tmin_Shear = min(str(indexl));      
          M2 = -MAXS/SafeM_Tmin_Shear -1;
        end
        if (MAXS)
          SafeM_SM_Shear = min(M1, M2);
        end            
      %
      end

    end % end recover stresses

end
%-------------------------------------------------------------------------------
% Monocoque method for multi web      h   c
function [tau] = smonoq_flux_mweb(tbs, Zs, nwebs, Ixx, tC, tW, Gref, Tx, Ty, Mz)
% nwebs: number of internal webs
% number of panels
if (nwebs<1)
  nwebs = 0;
  tW = tC;
end
npans = (nwebs + 1)*2 + nwebs + 2;
% number of stringers
nstrs = (nwebs + 2)*2;
% Inizialize to correct dimension
NODE = zeros(nstrs, 2);
AREA = zeros(nstrs, 1);
BETA = zeros(npans, 2);
T = zeros(npans,1);
G = zeros(npans,1);

% area of each stringer
area = Ixx/(nstrs*(tbs/2)^2);

% X and Y coordinates for the upper nodes
X = [Zs/2 : -Zs/(nwebs+1) : -Zs/2]';
Y = tbs/2*ones(nwebs+2, 1);

% Setup NODE
NODE = [ X,  Y;...
        -X, -Y];
    
% Setup AREA
AREA = area*ones(nstrs, 1);

% Setup BETA
for i = 1:nstrs-1
    BETA(i,:) = [i, i+1];
end
BETA(nstrs,:) = [nstrs, 1]; % close the outer circle
for j = 1:nwebs
    BETA(nstrs+j,:) = [BETA(j+1, 1), BETA(nstrs-j, 1)]; % webs within the section
end

% Setup T
T(1:nstrs) = tC;
T(nstrs+1:end) = tW;

% Setup G and Gref
G = Gref.*ones(npans,1);
%
[fluxes, tetap, CG, SC, Jx, Jy, R] = solve_mono(NODE, AREA, BETA, T, G, Tx, Ty, Mz);
tau = fluxes ./ T;

end
%--------------------------------------------------------------------------------------------------
