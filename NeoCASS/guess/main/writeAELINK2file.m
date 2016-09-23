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
%--------------------------------------------------------------------------------------------------
% Card to define relationship between control surface deflection
%
% Called by:    guess.m
%
% Calls:        BULKdataAELINK.m
%
% CREATED 2008-10-10
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [MASTER] = writeAELINK2file(outf, fid, pdcylin, geo, stick, aircraft, SELECT)
% Modified by Travaglini 19/11/2009, changing the aelink on twintail
% and adding an alelink between htail and canard
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Links aeroelastic variables');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
if (isempty(SELECT))
  INDEX = 1;
else
  INDEX = SELECT;
end
if (outf~=0)
  fprintf(outf, '\n\tExporting links aeroelastic variables...');
end
% counter
ID = 0;
MASTER = [];
MASTER.wing = {};
MASTER.htail = {};
MASTER.vtail = {};
MASTER.vtail2 = {};
MASTER.canard = {};
%--------------------------------------------------------------------------------------------------
% Wing
%--------------------------------------------------------------------------------------------------
if (INDEX == 1 || INDEX ==2)
  cont = 0;
  if isequal(pdcylin.stick.model.winr, 1) && isequal(pdcylin.stick.model.symmXZ, 1)
      nw = length(stick.IDCAERO1.winr);

      for i = 1 : nw

          if ~isequal(geo.wing.CAERO1.sup_control.typ(i), 0)
              ID = ID + 1;
              LABLD = geo.wing.CAERO1.sup_control.nme(i+nw,:);
              LABLI = geo.wing.CAERO1.sup_control.nme(i,:);
              C = geo.wing.CAERO1.sup_control.typ(i);
              % AELINK card
              BULKdataAELINK(fid, ID, LABLD, LABLI, C);
              cont = cont+1; MASTER.wing{cont} = LABLI;
          else
            if ~strcmp(strtok(geo.wing.CAERO1.sup_control.nme(i,:)),'none')
              cont = cont+1; MASTER.wing{cont} = geo.wing.CAERO1.sup_control.nme(i,:);
            end 
          end
      end

  end
end
%--------------------------------------------------------------------------------------------------
% Vertical tail
%--------------------------------------------------------------------------------------------------
if (INDEX == 1 || INDEX ==3)
cont = 0;
  if isequal(pdcylin.stick.model.vert, 1)
    if isequal(aircraft.Vertical_tail.Twin_tail, 0)
      if length(geo.vtail.CAERO1.sup_control.typ) > 1
        nv = length(stick.IDCAERO1.vert);
        if nv > 1 % do it as more than one sector is defined
          for i = 2 : nv
            if ~isequal(geo.vtail.CAERO1.sup_control.typ(i), 0)
              ID = ID + 1;
              LABLD = geo.vtail.CAERO1.sup_control.nme(i,:);
              LABLI = geo.vtail.CAERO1.sup_control.nme(1,:);
              C = geo.vtail.CAERO1.sup_control.typ(i);
              % AELINK card
              BULKdataAELINK(fid, ID, LABLD, LABLI, C);
              cont = cont+1; MASTER.vtail{cont} = LABLI;
            else
              if (strtok(geo.vtail.CAERO1.sup_control.nme(i,:))~='none')
                cont = cont+1; MASTER.vtail{cont} = geo.vtail.CAERO1.sup_control.nme(i,:);
              end 
            end
          end % for
        end % if
      end
    end  % if
  end
end
%--------------------------------------------------------------------------------------------------
% Twin Vertical tail
%--------------------------------------------------------------------------------------------------
if (INDEX == 1 || INDEX ==6)
  cont = 0;
  if isequal(pdcylin.stick.model.vert, 1)
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
      cont = 0;
      % set relationship linking the left with the right control surfaces
      if isequal(pdcylin.stick.model.symmXZ, 1)
        nv2 = size(geo.vtail2.CAERO1.sup_control.nme,1);
        for i = 2 : nv2
            ID = ID + 1;
            LABLD = geo.vtail2.CAERO1.sup_control.nme(i,:);
            LABLI = geo.vtail2.CAERO1.sup_control.nme(1,:);
            C = 1.0;
            % AELINK card
            BULKdataAELINK(fid, ID, LABLD, LABLI, C);
            cont = cont+1; MASTER.vtail2{cont} = LABLI;
        end
      end
    end
  end
end
%--------------------------------------------------------------------------------------------------
% Horizontal tail
%--------------------------------------------------------------------------------------------------
if (INDEX == 1 || INDEX ==4)
  cont = 0;
  if isequal(pdcylin.stick.model.horr, 1)
    if geo.htail.twc >0
        Ms = 2;
    else
        Ms = 1;
    end
    nh = length(stick.IDCAERO1.horr);
    
    %----------------------------------------------------------------------
    % set relationship for the right side control surfaces if more than one
    if nh > 1
        
        for i = Ms+1 : nh
            
            if ~isequal(geo.htail.CAERO1.sup_control.typ(i), 0)
                ID = ID + 1;
                LABLD = geo.htail.CAERO1.sup_control.nme(i,:);
                LABLI = geo.htail.CAERO1.sup_control.nme(Ms,:);
                C = geo.htail.CAERO1.sup_control.typ(i) *geo.htail.CAERO1.sup_control.typ(i); % on the same side
                % AELINK card
                BULKdataAELINK(fid, ID, LABLD, LABLI, C);
                cont = cont+1; MASTER.htail{cont} = LABLI;
            else
              if (strtok(geo.htail.CAERO1.sup_control.nme(i,:))~='none')
                cont = cont+1; MASTER.htail{cont} = geo.htail.CAERO1.sup_control.nme(i,:);
              end 
            end

        end
        
    end
    
    %----------------------------------------------------------------------
    % set relationship linking the left with the right control surfaces
%    if isfield(aircraft.Horizontal_tail,'Vtail')
%        for i = Ms+1 : nh
            
%            if ~isequal(geo.htail.CAERO1.sup_control.typ(i), 0)
%                ID = ID + 1;
%                LABLD = geo.htail.CAERO1.sup_control.nme(i+nh,:);
%                LABLI = geo.htail.CAERO1.sup_control.nme(Ms+nh,:);
%                C = -geo.htail.CAERO1.sup_control.typ(i);
%                % AELINK card
%                BULKdataAELINK(fid, ID, LABLD, LABLI, C);
%                cont = cont+1; MASTER.htail{cont} = LABLI;
%            else
%              if (strtok(geo.htail.CAERO1.sup_control.nme(i,:))~='none')
%                cont = cont+1; MASTER.htail{cont} = geo.htail.CAERO1.sup_control.nme(i,:);
%              end 
%            end
%            
%        end
%    else
        
        if isequal(pdcylin.stick.model.symmXZ, 1)
            
            for i = 1 : nh
                
                if ~isequal(geo.htail.CAERO1.sup_control.typ(i), 0)
                    ID = ID + 1;
                    LABLD = geo.htail.CAERO1.sup_control.nme(i+nh,:);
                    LABLI = geo.htail.CAERO1.sup_control.nme(Ms,:);
                    C = geo.htail.CAERO1.sup_control.typ(i);
                    % AELINK card
                    BULKdataAELINK(fid, ID, LABLD, LABLI, C);
                    cont = cont+1; MASTER.htail{cont} = LABLI;
                else
                  if (strtok(geo.htail.CAERO1.sup_control.nme(i,:))~='none')
                    cont = cont+1; MASTER.htail{cont} = geo.htail.CAERO1.sup_control.nme(i,:);
                  end 

                end
                
            end
            
        end
    %end
    
end
end

%--------------------------------------------------------------------------------------------------
% Canard
%--------------------------------------------------------------------------------------------------
if (INDEX == 1 || INDEX ==5)
cont = 0;
if isequal(pdcylin.stick.model.canr, 1) && aircraft.Strut_wing.present~=1
    if geo.canard.twc >0
        Ms = 2;
    else
        Ms = 1;
    end
    nh = length(stick.IDCAERO1.canr);
    
    %----------------------------------------------------------------------
    % set relationship for the right side control surfaces if more than one
    if nh > 1
        
        for i = Ms+1 : nh
            
            if ~isequal(geo.canard.CAERO1.sup_control.typ(i), 0)
                ID = ID + 1;
                LABLD = geo.canard.CAERO1.sup_control.nme(i,:);
                LABLI = geo.canard.CAERO1.sup_control.nme(Ms,:);
                C = geo.canard.CAERO1.sup_control.typ(i) *geo.canard.CAERO1.sup_control.typ(i); % on the same side
                % AELINK card
                BULKdataAELINK(fid, ID, LABLD, LABLI, C);
                cont = cont+1; MASTER.canard{cont} = LABLI;
              else
                if (strtok(geo.canard.CAERO1.sup_control.nme(i,:))~='none')
                  cont = cont+1; MASTER.canard{cont} = geo.canard.CAERO1.sup_control.nme(i,:);
                end 
            end
            
        end
        
    end
    
    %----------------------------------------------------------------------
    % set relationship linking the left with the right control surfaces
    if isequal(pdcylin.stick.model.symmXZ, 1)
        
        for i = 1 : nh
            
            if ~isequal(geo.canard.CAERO1.sup_control.typ(i), 0)
                ID = ID + 1;
                LABLD = geo.canard.CAERO1.sup_control.nme(i+nh,:);
                LABLI = geo.canard.CAERO1.sup_control.nme(Ms,:);
                C = geo.canard.CAERO1.sup_control.typ(i);
                % AELINK card
                BULKdataAELINK(fid, ID, LABLD, LABLI, C);
                cont = cont+1; MASTER.canard{cont} = LABLI;
            else
              if (strtok(geo.canard.CAERO1.sup_control.nme(i,:))~='none')
                cont = cont+1; MASTER.canard{cont} = geo.canard.CAERO1.sup_control.nme(i,:);
              end 
            end
            
        end
        
    end
    
    %----------------------------------------------------------------------
    % set relationship linking between htail and canard
%     if isequal(pdcylin.stick.model.horr, 1)
%         if isfield(geo.canard.CAERO1.sup_control,'tau')
%             nhh = length(stick.IDCAERO1.horr);
%             ID = ID + 1;
%             LABLD = geo.canard.CAERO1.sup_control.nme(nh,:);
%             LABLI = geo.htail.CAERO1.sup_control.nme(nhh,:);
%             C = geo.canard.CAERO1.sup_control.tau;
%             % AELINK card
%             BULKdataAELINK(fid, ID, LABLD, LABLI, C);
%         end
%     end
end
end

MASTER.wing   = unique(MASTER.wing);
MASTER.vtail  = unique(MASTER.vtail);
MASTER.vtail2 = unique(MASTER.vtail2);
MASTER.htail  = unique(MASTER.htail);
MASTER.canard = unique(MASTER.canard);
MASTER.All = [MASTER.wing, MASTER.wing, MASTER.vtail, MASTER.vtail2, MASTER.htail, MASTER.canard];
MASTER.All = unique(MASTER.All);
if (outf~=0)
  fprintf(outf, 'done.');
end