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
function pdcylin = setup_sma_defaults(pdcylin)
%
% REGRESSION ON/OFF for different components
%
  pdcylin.smartcad.fuse_regr   = 1;
  pdcylin.smartcad.wing_regr   = 1;
  pdcylin.smartcad.wing2_regr  = 1;
  pdcylin.smartcad.ht_regr     = 1;
  pdcylin.smartcad.vt_regr     = 1;
  pdcylin.smartcad.tboom_regr  = 1;
  pdcylin.smartcad.canard_regr = 1;
%
% SAFETY FACTOR (sizing)
%
  pdcylin.smartcad.Ks          = 1.5;
%
% INTERPOLATION PARAMETERS for SPLINE1/2/3 cards
%
  pdcylin.smartcad.spline_type = 1;
% BEAM(1), MLS (3) and RBF (2)
  pdcylin.smartcad.tcond       = 0;
% MLS (3)
  pdcylin.smartcad.weight      = 2;
  pdcylin.smartcad.rmax        = 0;
  pdcylin.smartcad.poly        = 2;
  pdcylin.smartcad.npoints     = 10;
%
% AERO bodies exported (if 1)
%
  pdcylin.smartcad.caerob      = 1;
%
% Look for user defined settings
%
  pdcylin.smartcad = load_user_settings(pdcylin.smartcad, ...
                        'setup_sma_defaults.txt');
%
end
%
function params = load_user_settings(params, filename)
%
%
  fid = 1;
  local_path = pwd;
  if isunix || ismac
    filename = [local_path, '/', filename];
  else
    filename = [local_path, '\', filename];
  end
%
  if exist(filename, 'file')
    fdata = importdata(filename);
    nd = size(fdata.data,1);
    fprintf(fid,'\n\t### Warning: SmartCAD default settings will be overwritten with user-defined values.'); 
  %
    for i=1:nd
      switch(char(fdata.textdata(i,1)))
%
        case {'fuse_regr', 'FUSE_REGR', 'Fuse_Regr'}
          if fdata.data(i)>=0
            params.fuse_regr = fdata.data(i);
            fprintf(fid,'\n\t\tFuselage regression: %d.', params.fuse_regr); 
          end
        case {'wing_regr', 'WING_REGR', 'Wing_Regr'}
          if fdata.data(i)>=0
            params.wing_regr = fdata.data(i);
            fprintf(fid,'\n\t\tWing regression: %d.', params.wing_regr); 
          end
        case {'wing2_regr', 'WING2_REGR', 'Wing2_Regr'}
          if fdata.data(i)>=0
            params.wing2_regr = fdata.data(i);
            fprintf(fid,'\n\t\tWing2 regression: %d.', params.wing2_regr); 
          end
        case {'ht_regr', 'HT_REGR', 'Ht_Regr'}
          if fdata.data(i)>=0
            params.ht_regr = fdata.data(i);
            fprintf(fid,'\n\t\tHorizontal tail regression: %d.', params.ht_regr); 
          end
        case {'vt_regr', 'VT_REGR', 'Vt_Regr'}
          if fdata.data(i)>=0
            params.vt_regr = fdata.data(i);
            fprintf(fid,'\n\t\tVertical tail regression: %d.', params.vt_regr); 
          end
        case {'canard_regr', 'CANARD_REGR', 'Canard_Regr'}
          if fdata.data(i)>=0
            params.canard_regr = fdata.data(i);
            fprintf(fid,'\n\t\tCanard regression: %d.', params.canard_regr); 
          end
%
        case {'Ks', 'ks', 'KS'}
          if fdata.data(i)>=1
            params.Ks = fdata.data(i);
            fprintf(fid,'\n\t\tKs: %d.', params.Ks); 
          end
        case {'spline_type', 'SPLINE_TYPE', 'Spline_Type'}
          if fdata.data(i)>=1 && fdata.data(i)<=3
            params.spline_type = fdata.data(i);
            fprintf(fid,'\n\t\tSpline_type: %d.', params.spline_type); 
          end
        case {'tcond', 'TCOND', 'TCond'}
          params.tcond = fdata.data(i);
          fprintf(fid,'\n\t\tTcond: %d.', params.tcond); 
        case {'weight', 'WEIGHT'}
          if fdata.data(i)>=1 && fdata.data(i)<=4
            params.weight = fdata.data(i);
            fprintf(fid,'\n\t\tMLS weight: %d.', params.weight); 
          end
        case {'rmax', 'RMAX', 'RMax'}
          params.rmax = fdata.data(i);
          fprintf(fid,'\n\t\tMLS radius: %d.', params.rmax); 
        case {'poly', 'POLY'}
          if fdata.data(i)==1 || fdata.data(i)==2
            params.poly = fdata.data(i);
            fprintf(fid,'\n\t\tMLS polynomial order: %d.', params.poly); 
          end
        case {'npoints', 'NPOINTS', 'NPoints'}
          params.npoints = fdata.data(i);
          fprintf(fid,'\n\t\tMLS suport points: %d.', params.npoints); 
        case {'caerob', 'CAEROB', 'CAerob'}
          if fdata.data(i)== 0 || fdata.data(i)== 1
            params.caerob = fdata.data(i);
            if (params.caerob) == 0
              fprintf(fid,'\n\t\tFuselage body: disabled.'); 
            else
              fprintf(fid,'\n\t\tFuselage body: enabled.'); 
            end
          end
      end
    end
  end
end