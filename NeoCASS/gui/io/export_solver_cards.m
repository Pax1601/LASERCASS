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
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080224      1.0     L. Cavagna       Creation
%     090312      1.3     A. De Gaspari    Modification
%     090323      1.3.2   A. De Gaspari    Modification
%
%*******************************************************************************
%
% function export_solver_cards(fp, gui_param_file)
%
%   DESCRIPTION: Appends solver cards to GUESS file or new file
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                fp             pointer    output file pointer
%                gui_param_file string     GUI parameters filename                                      
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************

function export_solver_cards(fp, gui_param_file)

fid = 1;

path = neoguiscratchpath;

command = ['gui_param = load(''', path, gui_param_file,''');'];
eval(command);

NV = gui_param.solver.flutter.NV;
EXPORT_EIG = gui_param.solver.eig.NROOTS + gui_param.solver.eig.MINF;
STATE_MAT = gui_param.solver.aeros.STATE_MAT; 

EIG_AV = gui_param.solver.EIG_AV;
FLUTT_AV = gui_param.solver.FLUTT_AV;
STATIC_AERO_AV = gui_param.solver.STATIC_AERO_AV;

% determine SOL to be given between 144/145/103
SOL = 0;
if FLUTT_AV
  SOL = 145;
elseif EIG_AV
  SOL = 103;
elseif STATIC_AERO_AV
  SOL = 144;
else
  SOL = 101; % default static solver
end

fprintf(fp, '\nSOL %d', SOL);

if (FLUTT_AV)

  if (NV >0)
  %
    fprintf(fid, '\n - Exporting parameters for flutter solver...');
    Mach_list = gui_param.solver.flutter.Mach_list;
    Freq_list = gui_param.solver.flutter.RFreq_list;
    Track_list = gui_param.solver.flutter.FMODES;
    Mode_list = gui_param.solver.flutter.MSELECT;
    CREF = gui_param.solver.aero.CREF;
    VMAX = gui_param.solver.flutter.VMAX;
    RHO = gui_param.solver.flutter.RHO;
    SIMXZ = gui_param.solver.aero.SXZ;
    SIMXY = gui_param.solver.aero.SXY;
    DLM_ORDER = gui_param.solver.aero.DLM_ORDER;
  %
    export_dlmsol_param(fp, Mach_list, Freq_list, Track_list, Mode_list, CREF, VMAX, NV, RHO, SIMXZ, SIMXY, DLM_ORDER);
  %
    fprintf(fid, 'done.\n');
  else
    fprintf(fid, '\n - Number of velocity points (V Step) not given.');
  end
end

if (EIG_AV)
%
  fprintf(fid, '\n - Exporting parameters for eigenvalues solver...');
  MINF = gui_param.solver.eig.MINF;
  MAXF = gui_param.solver.eig.MAXF;
  NROOTS = gui_param.solver.eig.NROOTS;
  POINT = gui_param.solver.eig.REF_GRID;
  CORD = gui_param.solver.eig.REF_GRID_C;
  MET = gui_param.solver.eig.NORM_MET;
%
  export_eigsol_param(fp, MET, MINF, MAXF, NROOTS, POINT, CORD);
%
  fprintf(fid, 'done.\n');
end

if (STATIC_AERO_AV)
%
  fprintf(fid, '\n - Exporting parameters for steady VLM solver...');
  SIMXZ = gui_param.solver.aero.SXZ;
  SIMXY = gui_param.solver.aero.SXY;
  CREF = gui_param.solver.aero.CREF;
  BREF = gui_param.solver.aero.BREF;
  SREF = gui_param.solver.aero.SREF;
  HEIGHT = gui_param.solver.aero.HEIGHT;
%
  export_st_aerosol_param(fp, CREF, BREF, SREF, SIMXZ, SIMXY, HEIGHT);
  %
  suportID = gui_param.solver.suport;
  if ~isempty(suportID),
      [SUPORT] = cnvt2_8chs(suportID);
      fprintf(fp, '\n$');
      fprintf(fp, ['\nSUPORT  1       ', '%s', '123456  '], SUPORT);
      fprintf(fp, '\n$\n');
  end
  %
  writeTRIM2file(fp, STATE_MAT);
  
  fprintf(fid, 'done.\n');
  %
%
%   ns = size(STATE_MAT, 1);
%   if (ns > 1)
%     fprintf(fid, '\n      Warning: only flight state %d is exported.', STATE_MAT(1,1));
%   end
%   export_flight_param(fp, STATE_MAT(1, :));
%
  
%
end
