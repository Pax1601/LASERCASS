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
%
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      1.0     L. Cavagna       Creation
%     090325      1.3     A. De Gaspari    Creation of include_field_parser function
%     090902      1.3.7   A. De Gaspari    Added maneuver label to the TRIM= card
%
%*******************************************************************************
%
% function [INFO, PARAM, COORD, NODE, MAT, BAR, PBAR, BEAM, PBEAM, FORCE,
%           MOMENT, F_FLW, CONM, MASS, SPC, CAERO, OPTIM] = read_nas_file(filename)
%
%   DESCRIPTION: Load NEOCASS nastran-like file and creates database
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                filename       FILE       NEOCASS nastran-like input file
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                INFO           struct     Card counters
%                PARAM          struct     Model parameters
%                COORD          struct     Reference frames data
%                NODE           struct     Node data
%                MAT            struct     Material data
%                BAR            struct     Bar data
%                PBAR           struct     Bar property data
%                BEAM           struct     Beam data
%                PBEAM          struct     Beam property data
%                FORCE          struct     Applied force data
%                MOMENT         struct     Applied moments data
%                F_FLW          struct     Applied follower forces data
%                CONM           struct     Concentrated mass data
%                MASS           struct     Model weight data
%                SPC            struct     Model constraint data
%                CAERO          struct     Model aerodynamic data
%
%    REFERENCES:
%
%*******************************************************************************

function [INFO, PARAM, COORD, NODE, MAT, BAR, PBAR, BEAM, PBEAM, FORCE, MOMENT, ...
           F_FLW, CONM, MASS, SPC, CAERO, OPTIM, CELAS, RBE2, GUST, SET, SURFDEF, ...
           DEXTLOAD, DAMP, DESOPT] = read_nas_file(filename)

fid = 1;
FIELD = 8;
CORDIT = 10;
% define allowed Nastran cards
keyword = { 'CORD1R'
    'GRID'
    'CBAR'
    'PBAR'
    'CORD2R'
    'MAT1'
    'CONM1'
    'CONM2'
    'PARAM'
    'FORCE'
    'FORCE1'
    'FORCE2'
    'MOMENT'
    'MOMENT1'
    'MOMENT2'
    'LOAD='  % case control CARD
    'SPC='   % case control CARD
    'SPC1'
    'SOL'
    'CBEAM'
    'PBEAM'
    'CAERO1'
    'AEROS'
    'AERO'
    'SPLINE3'
    'SET1'
    'RBE0'   % this card is defined for geometric aeropoints input
    'SPLINE2'
    'TRIM'
    'TRIM='  % case control CARD
    'THRUST' % add-on: follower force for thrust modelling
    'AELINK'
    'MKAERO1'
    'EIGR'
    'GRAV'
    'MSELECT'
    'SUPORT'
    'PBARSM1'
    'PBARGMW' % GUESS Multi-Web box beam
    'FMODES'
    'PBARWB1'
    'PBARGFU' % GUESS Fuselage Unframed
    'PBARGFF' % GUESS Fuselage Framed
    'INCLUDE' % include filename
    'CELAS'
    'RBE2'
    'UMODES' % 47
    'GUST'
    'SET'
    'ACCELERATION=' %50 %recovery
    'DISP='
    'VELOCITY='
    'IFORCE='
    'AEROFORCE='
    'HINGEFORCE='
    'SURFDEF'
    'GUST='        % case control CARD
    'SURFDEF='     % case control CARD
    'IFORCEBE='
    'SPLINE1'
    'ITRIM='      % case control CARD
    'DLOAD'
    'SDAMPING='   % case control CARD
    'TABDMP1'
    'CAEROB'
    'TRIMEXT'
    'DEREXT'
    'AERUPD'
    'DESVAR'
    'DLINK'
    'DVPREL'
    'DRESP'
    'TRIMRESP'
    'STABRESP'
    'CAERO0'
    'AESURF'
    'AELIST'
}; 

%

if ~exist(filename)
    
    error('\nUnable to find file %s.', filename);
    
else
    
    % info struct with counters
    INFO = [];
    % MASS struct
    MASS.CG = zeros(1,3);     % center of gravity coords
    MASS.MCG = zeros(6,6);    % mass matrix 6x6 collocated in cg
    MASS.MRP = zeros(6,6);    % mass matrix 6x6 collocated in the grid point required by the user
    MASS.MCG_pa = zeros(6,6); % mass matrix 6x6 collocated in cg along principal axes
    MASS.R_pa = zeros(3,3);   % principal axes cosines
    % SPC struct
    SPC = [];
    SPC.ID = [];
    SPC.DOF = [];
    SPC.Nodes = [];
    % PARAM struct
    PARAM = [];
    % DAMP
    PARAM.SDAMP = 0;
    PARAM.KDAMP = 1;
    % OPTIMIZATION
    PARAM.OBJ = '';
    PARAM.DESVAR = '';
    PARAM.DLINK = '';
    PARAM.ST_CONSTR = '';
    PARAM.DY_CONSTR = '';
    PARAM.DIRDER = 1.0e-5;
    PARAM.OPTDVAR = 1.0;
    % DLM
    PARAM.DLM_ORDER = 2;
    PARAM.DLM_NP    = 12;
    PARAM.DLM_KMAX  = 0;
    PARAM.DLM_AR    = 3;
    % flutter plot
    PARAM.NVSTEP = 50;
    PARAM.VMAX = 0;
    PARAM.RHO_VG = 0;
    PARAM.FMODES = [];
    PARAM.UMODES = [];
    % divergence in SOL 144
    PARAM.DIVERG = 0;
    % external derivatives in SOL 144
    PARAM.DER_FILE = '';
    PARAM.DER_TYPE = 0;
    PARAM.AEROINT = [];
    % eigenvalues
    PARAM.EIG_FILE = '';
    PARAM.MSELECT = [];
    PARAM.SUPORT = [];
    PARAM.MINF = 0;
    PARAM.MAXF = 0;
    PARAM.NROOTS = 0;
    PARAM.MSCALE = 'MASS';
    PARAM.MG = 0;
    PARAM.MC = 0;
    PARAM.SUP_MAMPL = 1.0;
    %
    PARAM.FUSE_DP = 0;
    %
    PARAM.FILE = filename;
    PARAM.INCLUDE = {};
    PARAM.INCLUDE{1} = filename; % dummy
    PARAM.AUTOPLOT = false;
    PARAM.GRDPNT = int32(0);
    PARAM.LOAD = int32(0);
    PARAM.GUST = [];
    PARAM.SURFDEF = [];
    PARAM.SPC = int32(0);
    PARAM.SOL = int32(0);
    PARAM.MSOL = [];
    % landing gear
    PARAM.LANDG = []; % node IDs
    % non linear beam settings
    PARAM.EPS = 1.0e-3;
    PARAM.NSTEP = 10;
    PARAM.NITER = 5;
    PARAM.RES_TOL = 1.0e-6;
    % aerodynamic relax factor
    PARAM.REL_FAC = 0.5;
    %
    PARAM.WTMASS = 1.0;
    PARAM.GRAV = zeros(3,1);
    PARAM.G = 9.81;
    GRAV = []; % dummy variable
    PARAM.FID = fid;
    % responces settings
    PARAM.RHOREF = 0.0;
    PARAM.MACH   = 0.0;
    PARAM.VREF   = 0.0;
    PARAM.MODACC = -1;
    PARAM.BCOU = 1;
    PARAM.ACCELERATION = [];
    PARAM.VELOCITY = [];
    PARAM.DISP = [];
    PARAM.IFORCE = [];      % internal loads on BAR
    PARAM.IFORCEBE = [];    % internal loads on BEAM
    PARAM.AEROFORCE = [];   % Aerodinamic Loads
    PARAM.HINGEFORCE = [];  % Hinge Moments
    % FORCE struct
    FORCE = [];
    FORCE.ID =   [];
    FORCE.Type = [];
    FORCE.Node = [];
    FORCE.Mag = [];
    FORCE.Orient = [];
    FORCE.CID = [];
    % FORCE struct
    MOMENT = [];
    MOMENT.ID =   [];
    MOMENT.Type = [];
    MOMENT.Node = [];
    MOMENT.Mag = [];
    MOMENT.Orient = [];
    MOMENT.CID = [];
    % FOLLOWER struct
    F_FLW = [];
    F_FLW.ID =   [];
    F_FLW.Node = [];
    F_FLW.Mag = [];
    F_FLW.Orient = [];
    F_FLW.CID = [];
    % dummy COORD1 struct
    COORD1 = [];
    COORD1.ID = [];
    COORD1.Nodes = [];
    COORD1.R = [];
    COORD1.Origin = [];
    % dummy COORD2 struct
    COORD2 = [];
    COORD2.ID = [];
    COORD2.RID = [];
    COORD2.Nodes = [];
    COORD2.R = [];
    COORD2.Origin = [];
    % COORD struct
    COORD = [];
    COORD.ID = [];
    COORD.Origin = [];
    COORD.R = [];
    % NODE struct
    NODE  = [];
    NODE.ID = [];
    NODE.CS = [];
    NODE.Coord = [];
    NODE.CD = [];
    %NODE.PS = [];
    NODE.Index = [];
    NODE.R = [];
    NODE.DOF = [];
    NODE.Aero = [];
    % BEAM struct
    BEAM  = [];
    BEAM.ID = [];       % element ID
    BEAM.PID = [];      % Property ID
    BEAM.Conn = [];     % Connectivity
    BEAM.Orient = [];   % Orientation vector
    BEAM.OffsetT = [];  % Offset type
    BEAM.Offset = [];   % offset values
    BEAM.Colloc = [];   % collocation point global coordinates
    BEAM.R = [];        % 5 Reference frame matrices
    BEAM.D = [];        % 2 section stiffness matrices
    BEAM.M = [];        % 3 nodes mass matrix
    beamg0 = [];        % dummy variable
    % PBAR struct
    PBEAM = [];
    PBEAM.ID = [];
    PBEAM.Mat = [];
    PBEAM.A = [];
    PBEAM.I = [];
    PBEAM.J = [];
    PBEAM.RhoNS = [];
    PBEAM.Kshear = [];
    PBEAM.X_L = [];
    PBEAM.NSI = [];
    PBEAM.NSCG = [];
    PBEAM.NA = [];
    PBEAM.DA = [];
    PBEAM.DI = [];
    PBEAM.DJ = [];
    PBEAM.DRhoNS = [];
    % BAR struct
    BAR  = [];
    BAR.ID = [];       % element ID
    BAR.PID = [];      % Property ID
    BAR.Conn = [];     % Connectivity
    BAR.Orient = [];   % Orientation vector
    BAR.OffsetT = [];  % Offset type
    BAR.Offset = [];   % offset values
    BAR.Colloc = [];   % collocation point global coordinates
    BAR.R = [];        % 5 Reference frame matrices
    BAR.D = [];        % 2 section stiffness matrices
    BAR.M = [];        % 3 nodes mass matrix
    BAR.barg0 = [];    % dummy variable
    % PBAR struct
    PBAR = [];
    PBAR.ID = [];
    PBAR.Mat = [];
    PBAR.A = [];
    PBAR.I = [];
    PBAR.J = [];
    PBAR.RhoNS = [];
    PBAR.Kshear = [];
    PBAR.Str_point = [];
    PBAR.Type = [];
    PBAR.Section = [];
    PBAR.SI = [];
    % MAT1 struct
    MAT = [];
    MAT.ID = [];
    MAT.E = [];
    MAT.G = [];
    MAT.nu = [];
    MAT.Rho = [];
    MAT.ST = [];
    MAT.SC = [];
    MAT.SS = [];
    % CONM1 struct
    CONM1 = [];
    CONM1.ID = [];
    CONM1.Node = [];
    CONM1.CID = [];
    CONM1.M = [];
    % CONM2 struct
    CONM2 = [];
    CONM2.ID = [];
    CONM2.Node = [];
    CONM2.CID = [];
    CONM2.M = [];
    CONM2.Offset = [];
    % CAERO struct
    CAERO.ID = [];
    CAERO.CP = [];
    CAERO.IS = [];
    CAERO.INT = [];
    CAERO.AESURF = [];
    CAERO.AESURF.ID = [];
    CAERO.AESURF.Name = {};
    CAERO.AESURF.CID = [];
    CAERO.AESURF.AERID = [];
    % BAERO struct
    BAERO.ID = [];
    BAERO.CP = [];
    BAERO.SET = [];
    BAERO.Interp = [];
    AEROLE = [];
    %
    CAERO.geo.ny = [];
    CAERO.geo.nx = [];
    CAERO.geo.startx = [];
    CAERO.geo.starty = [];
    CAERO.geo.startz = [];
    CAERO.geo.c = [];
    CAERO.geo.foil = cell(0);
    CAERO.geo.b = [];
    CAERO.geo.T = [];
    CAERO.geo.SW = [];
    CAERO.geo.TW = [];
    CAERO.geo.dihed = [];
    CAERO.geo.meshtype = [];
    CAERO.geo.flapped = [];
    CAERO.geo.nwing = [];
    CAERO.geo.nelem = [];
    CAERO.geo.CG = [];
    CAERO.geo.ref_point = [];
    CAERO.geo.symetric = [];
    CAERO.geo.fc = [];
    CAERO.geo.fnx = [];
    CAERO.geo.fsym = [];
    CAERO.geo.flap_vector = [];
    CAERO.geo.nc = 0;
    %
    CAERO.state.AS = 1;
    CAERO.state.alpha = 0;
    CAERO.state.betha =0;
    CAERO.state.P = 0;
    CAERO.state.Q = 0;
    CAERO.state.R = 0;
    CAERO.state.ALT = 0;
    CAERO.state.rho = 0;
    CAERO.state.pgcorr = 1; % set Prandtl-Glauert correction
    AERUPD = [];
    %  
    BAERO.geo.ref_point = [];
    BAERO.geo.fs    = [];
    BAERO.geo.Rs    = [];
    BAERO.geo.L     = [];
    BAERO.geo.R     = [];
    BAERO.geo.Rx    = [];
    BAERO.geo. x    = [];
    BAERO.geo.Nelem = [];
    BAERO.geo.CAERO_INT = [];
   %
    % dummy variables to overwrite Tornado's reference values
    CREF_VLM = 1;
    BREF_VLM = 1;
    SREF_VLM = 1;
    %
    CREF_DLM = 1;
    BREF_DLM = 1;
    SREF_DLM = 1;
    %
    SIMXZ_VLM = 0;
    SIMXY_VLM = 0;
    %
    SIMXZ_DLM = 0;
    SIMXY_DLM = 0;
    %
    Param.data = cell(0);
    Value.data = [];
    TRIM_PARAM.data = cell(0);
    TRIM_VALUE.data = [];
    USER_HINGE = [];
    % addition to Tornado
    CAERO.state.SIMXZ = 0;
    CAERO.state.SIMXY = 0;
    CAERO.state.Mach     = [];
    CAERO.state.Mach_qhh = [];
    CAERO.state.Kfreq = [];
    %
    CAERO.ref.S_ref = 0;
    CAERO.ref.C_mac = 0;
    %CAERO.ref.mac_pos = 0;
    CAERO.ref.C_mgc = 0;
    CAERO.ref.b_ref = 0;
    CAERO.lattice = [];
    CAERO.lattice_defo = [];
    CAERO.lattice_vlm = [];
    CAERO.lattice_dlm = [];
    % interpolation params
    CAERO.Interp = [];
    CAERO.Interp.ID = [];
    CAERO.Interp.Patch = [];
    CAERO.Interp.Index = [];
    CAERO.Interp.Param = [];
    CAERO.Interp.Type = [];
    % strctural interpolation set
    CAERO.Set = [];
    CAERO.Set.ID = [];
    CAERO.Set.Node = [];
    % TRIM section
    CAERO.Trim = [];
    CAERO.Trim.ID = [];
    CAERO.Trim.Type = [];
    CAERO.Trim.Select = -1;
    CAERO.Trim.Man_index = [];
    CAERO.Trim.CID = [];
    CAERO.Trim.Mach = [];
    CAERO.Trim.ALT = [];
    CAERO.Trim.FM = [];
    CAERO.Trim.CS = [];
    CAERO.Trim.FM.Fixed = [];
    CAERO.Trim.FM.Value = [];
    CAERO.Trim.CS.Fixed = [];
    CAERO.Trim.CS.Value = [];
    CAERO.Trim.Link = [];
    CAERO.Trim.NC = [];
    CAERO.Trim.Symm = [];
    CAERO.Trim.Param = [];
    CAERO.Trim.Value = [];
    CAERO.Trim.Ext = [];
    CAERO.Trim.MINDEX = [];
    %
    RBE0 = [];
    RBE0.ID = [];
    RBE0.Master = [];
    RBE0.Node = [];
    % dummy control surface cell variables
    CONTROL_NAME = cell(0);
    % OPTIMIZATION section
    OPTIM = [];
    OPTIM.Desvar = [];
    OPTIM.Desvar.n_desvar = 0;
    OPTIM.Desvar.Name = {};
    %
    OPTIM.Cstr = [];
    OPTIM.Cstr.In.Name = {};
    OPTIM.Cstr.In.Value = {};
    OPTIM.Cstr.In.CT = {};
    OPTIM.Cstr.Eq.Name = {};
    OPTIM.Cstr.Eq.Value = {};
    OPTIM.Cstr.Eq.CT = {};
    OPTIM.Cstr.n_in = 0;
    OPTIM.Cstr.n_eq = 0;
    %
    OPTIM.Xnorm = [];
    OPTIM.X     = [];
    OPTIM.X0    = [];
    OPTIM.XL    = [];
    OPTIM.XU    = [];
    OPTIM.OBJ   = [];
    OPTIM.DOBJ   = [];
    OPTIM.CSTR_IN   = [];
    OPTIM.CSTR_EQ   = [];
    OPTIM.DCSTR_IN   = [];
    OPTIM.DCSTR_EQ   = [];
    %
    DESOPT = [];
    DESOPT.DESVAR = [];
    DESOPT.DESVAR.ID = [];
    DESOPT.DESVAR.NAME = {};
    DESOPT.DESVAR.X0 = [];
    DESOPT.DESVAR.XL = [];
    DESOPT.DESVAR.XU = [];
    DESOPT.DESVAR.DELTA = [];
%
    DESOPT.DLINK.ID = [];
    DESOPT.DLINK.FUN = {};
    DESOPT.DLINK.XL = [];
    DESOPT.DLINK.XU = [];
%
    DESOPT.DRESP.ID = [];
    DESOPT.DRESP.NAME = [];
    DESOPT.DRESP.Type = [];
    DESOPT.DRESP.Set = [];
    DESOPT.DRESP.Comp = [];
    DESOPT.DRESP.Index = [];
    DESOPT.DRESP.XL = [];
    DESOPT.DRESP.XU = [];
%
    DESOPT.TRIMRESP.ID = [];
    DESOPT.TRIMRESP.Index = [];
    DESOPT.TRIMRESP.NAME = [];
    DESOPT.TRIMRESP.Comp = [];
    DESOPT.TRIMRESP.XL = [];
    DESOPT.TRIMRESP.XU = [];
%
    DESOPT.STABRESP.ID = [];
    DESOPT.STABRESP.NAME = [];
    DESOPT.STABRESP.Comp = [];
    DESOPT.STABRESP.Set = [];
    DESOPT.STABRESP.XL = [];
    DESOPT.STABRESP.XU = [];
%
    DESOPT.DVPREL.ID = [];
    DESOPT.DVPREL.PID = [];
    DESOPT.DVPREL.Type = [];
    DESOPT.DVPREL.Index = [];
    DESOPT.DVPREL.FUN = {};
    DESOPT.DVPREL.XL = [];
    DESOPT.DVPREL.XU = [];
    DESOPT.DVPREL.Bar = {};
    %
    %CELAS
    CELAS.ID = [];
    CELAS.Node =[];
    % CELAS.DOF
    CELAS.STiff = [];
    
    %RBE2
    RBE2 = [];
%    RBE2.ID = [];
%    RBE2.IDM =[];
    % GUST
    GUST.ID = [];
    GUST.DIR = [];
    GUST.Amp = [];
    GUST.Tmax = [];
    GUST.X0 = [];
    GUST.funs = {}; % gust space profile
    GUST.fun = {};  % gust time profile
    % SURFDEF
    SURFDEF.ID = [];
    SURFDEF.Label = {};
    SURFDEF.Amp = [];
    SURFDEF.Tmax = [];
    SURFDEF.X0 = [];
    SURFDEF.fun = {};
    SURFDEF.Ti = [];
    SURFDEF.Npiece = [];
   % DEXTLOAD
    DEXTLOAD.ID = [];
    DEXTLOAD.Node = [];
    DEXTLOAD.NDOF = [];
    DEXTLOAD.Amp = [];
    DEXTLOAD.Tmax = [];
    DEXTLOAD.X0 = [];
    DEXTLOAD.fun = {};
    % SET
    SET.ID = [];
    SET.Val = [];
    % DAMP
    DAMP.ID = [];
    DAMP.Type = [];
    DAMP.Freq = [];
    DAMP.g = [];
    % counters
    ndamp = 0;
    nsuport = 0;
    ngrav = 0;
    ngrid = 0;
    ncord =  0;
    ncord1 = 0;
    ncord2 = 0;
    ncbar  = 0;
    ninclude = 1;
    ncbeam = 0;
    npbar  = 0;
    npbeam = 0;
    ncoord = 0;
    nmat   = 0;
    ncom1 = 0;
    ncom2 = 0;
    nforce = 0;
    nmom = 0;
    mtot = 0;
    nload = 0;
    nspc = 0;
    cc_nspc = 0;
    cc_trim = 0;
    i_trim = 0;
    i_damp = 0;
    ndof = 0;
    ncaero = 0;
    ncaero0 = 0;
    naesurf = 0;
    nbaero = 0;
    ninterp = 0;
    nset = 0;
    nrbe0 = 0;
    naero = 0;
    naeros = 0;
    ntrim = 0;
    mindex = 0;
    ntrimext = 0;
    nderext = 0;
    nflw = 0;
    nlink = 0;
    nmkaero = 0;
    neig = 0;
    npbarsm = 0;
    ncelas = 0;
    nrbe2 = 0;
    ngust = 0;
    nSET = 0;
    nsurfdef = 0;
    ndload = 0;
    naerupd = 0;
    ndesvar = 0;
    ndlink = 0;
    ndvprel = 0;
    ndresp = 0;
    ntrimresp = 0;
    nstabresp = 0;
    %
    ngustl = 0;
    nsurfdefl =0;
    %*******************************************************************************
    READ_INCLUDE = true;
    NFILE = 0;
    
    while (READ_INCLUDE)
        
        NFILE = NFILE + 1;
        fp = fopen(PARAM.INCLUDE{NFILE}, 'r');
        
        skip_line = false;
        fprintf(fid,'\nReading %s file...', PARAM.INCLUDE{NFILE});
        
        while ~feof(fp)
            
            if ~skip_line
                
                tline = fgetl(fp);
                
            else
                
                skip_line = false;
                
            end
            
            CARD = strtok(tline);
            
            switch CARD
                
                % CORD1R card
                % assume the 3 grid points are given in the GLOBAL reference frame
                case keyword{1}
                    
                    fprintf(fid, '\n\t\t### Warning: COORD1R card will assume nodes are given in the BASIC reference frame.');
                    
                    ncord1 = ncord1 +1;
                    COORD1.ID(ncord1) = int32(num_field_parser(tline, 2));
                    if COORD1.ID(ncord1) < 1
                        fclose(fp);
                        error('Coordinate system ID must be greater than zero.');
                        
                    end
                    COORD1.Nodes(ncord1,1) = int32(num_field_parser(tline, 3));
                    COORD1.Nodes(ncord1,2) = int32(num_field_parser(tline, 4));
                    COORD1.Nodes(ncord1,3) = int32(num_field_parser(tline, 5));
                    
                    if length(tline) > 6 * FIELD
                        
                        ncord1 = ncord1 +1;
                        COORD1.ID(ncord1) = int32(num_field_parser(tline, 6));
                        COORD1.Nodes(ncord1,1) = int32(num_field_parser(tline, 7));
                        COORD1.Nodes(ncord1,2) = int32(num_field_parser(tline, 8));
                        COORD1.Nodes(ncord1,3) = int32(num_field_parser(tline, 9));
                        
                    end
                    
                    skip_line = false;
                    
                    % CORD2R card
                case keyword{5}
                    
                    ncord2 = ncord2 +1;
                    COORD2.ID(ncord2) = int32(num_field_parser(tline, 2));
                    
                    if COORD2.ID(ncord2) < 1
                        fclose(fp);
                        error('Coordinate system ID must be greater than zero.');
                        
                    end
                    
                    COORD2.RID(ncord2) = int32(num_field_parser(tline, 3));
                    
%                    if COORD2.RID(ncord2) ~= 0
%                        fclose(fp)
%                        error('Allowed to read only points declared in the BASIC reference frame. Please contact DIAPM.');
%                    end
                    
                    COORD2.Nodes(1,1,ncord2) = num_field_parser(tline, 4);
                    COORD2.Nodes(1,2,ncord2) = num_field_parser(tline, 5);
                    COORD2.Nodes(1,3,ncord2) = num_field_parser(tline, 6);
                    COORD2.Nodes(2,1,ncord2) = num_field_parser(tline, 7);
                    COORD2.Nodes(2,2,ncord2) = num_field_parser(tline, 8);
                    COORD2.Nodes(2,3,ncord2) = num_field_parser(tline, 9);
                    COORD2.Nodes(3,1:3,ncord2) =[0 0 0];
                    % try to read continuation
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        
                        for n=1:3
                            
                            COORD2.Nodes(3,n,ncord2) = num_field_parser(tline, 1+n);
                            
                        end
                        
                        skip_line = false;
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                    % GRID
                case keyword{2} % GRID card
                    
                    ngrid = ngrid +1;
                    
                    NODE.ID(ngrid) = int32(num_field_parser(tline, 2));
                    NODE.CS(ngrid) = int32(num_field_parser(tline, 3));
                    NODE.Coord(ngrid, 1) = num_field_parser(tline, 4);
                    NODE.Coord(ngrid, 2) = num_field_parser(tline, 5);
                    NODE.Coord(ngrid, 3) = num_field_parser(tline, 6);
                    NODE.CD(ngrid) = int32(num_field_parser(tline, 7));
                    %NODE.PS(ngrid) = int32(num_field_parser(tline, 8));
                    % skip by default the superelement index field 9
                    skip_line = false;
                    
                    % CBAR
                case keyword{3}
                    
                    ncbar = ncbar +1;
                    
                    BAR.ID(ncbar) = int32(num_field_parser(tline, 2));
                    if ~BAR.ID(ncbar)
                        fclose(fp)
                        error('Given null Bar ID.');
                        
                    end
                    BAR.PID(ncbar) = int32(num_field_parser(tline, 3));
                    BAR.Conn(ncbar,1) = int32(num_field_parser(tline, 4));
                    BAR.Conn(ncbar,2) = int32(num_field_parser(tline, 5));
                    BAR.Conn(ncbar,3) = 0; % create the third field
                    
                    g0 = string_field_parser(tline, 6);
                    i = find(g0  == '.'); % check if real or integer
                    
                    if isempty(i)
                        % integer
                        BAR.barg0(ncbar) = true;
                        
                    else
                        % real
                        BAR.barg0(ncbar) = false;
                        
                    end
                    
                    BAR.Orient(ncbar,1) = num_field_parser(tline, 6);
                    BAR.Orient(ncbar,2) = num_field_parser(tline, 7);
                    BAR.Orient(ncbar,3) = num_field_parser(tline, 8);
                    
                    BAR.OffsetT(ncbar) = 1; % set to default value
                    type = string_field_parser(tline, 9);
                    
                    if ~isempty(type)
                        
                        switch(type)
                            
                            case 'GGG'
                                
                                BAR.OffsetT(ncbar) = 1;
                                
                            case 'BGG'
                                
                                BAR.OffsetT(ncbar) = 2;
                                
                            case 'GGO'
                                
                                BAR.OffsetT(ncbar) = 3;
                                
                            case 'BGO'
                                
                                BAR.OffsetT(ncbar) = 4;
                                
                            case 'GOG'
                                
                                BAR.OffsetT(ncbar) = 5;
                                
                            case 'BOG'
                                
                                BAR.OffsetT(ncbar) = 6;
                                
                            case 'GOO'
                                
                                BAR.OffsetT(ncbar) = 7;
                                
                            case 'BOO'
                                
                                BAR.OffsetT(ncbar) = 8;
                                
                            otherwise
                                fclose(fp)
                                error('Unknown offset option for CBAR %d.', BAR.ID(ncbar));
                                
                        end
                    end
                    
                    BAR.Offset(ncbar,1:9) = zeros(1,9);
                    
                    % try to read continuation
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        
                        for n=1:3
                            
                            BAR.Offset(ncbar,n) = num_field_parser(tline, 3+n);
                            
                        end
                        for n=7:9
                            
                            BAR.Offset(ncbar,n) = num_field_parser(tline, n);
                            
                        end
                        
                        
                        skip_line = false;
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                    % PBAR
                case keyword{4} % PBAR card
                    
                    npbar = npbar +1;
                    
                    PBAR.ID(npbar) = int32(num_field_parser(tline, 2));
                    PBAR.SI(npbar) = 0;
                    PBAR.Type(npbar) = 0;
                    PBAR.Mat(npbar) = int32(num_field_parser(tline, 3));
                    PBAR.A(npbar) = num_field_parser(tline, 4);
                    PBAR.I(npbar,1) = num_field_parser(tline, 5);
                    PBAR.I(npbar,2) = num_field_parser(tline, 6);
                    PBAR.I(npbar,3) = 0.0;
                    PBAR.J(npbar) = num_field_parser(tline, 7);
                    PBAR.RhoNS(npbar) = num_field_parser(tline, 8);
                    
                    PBAR.Kshear(npbar,1:2) = zeros(1,2);
                    PBAR.Str_point(:,:,npbar) = zeros(4,2);
                    
                    % try to read continuation
                    skip_line = false;
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        % store stress recovery points
                        r = 2;
                        for n=1:4
                            
                            PBAR.Str_point(n,1,npbar) = num_field_parser(tline, r);
                            PBAR.Str_point(n,2,npbar) = num_field_parser(tline, r+1);
                            r = r+2;
                        end
                        
                        tline = fgetl(fp);
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            
                            skip_line = false;
                            
                            PBAR.Kshear(npbar,1) = num_field_parser(tline, 2);
                            PBAR.Kshear(npbar,2) = num_field_parser(tline, 3);
                            
                            PBAR.I(npbar,3) = num_field_parser(tline, 4);
                            
                            % check inertia properties
                            
                            if (PBAR.I(npbar,1) * PBAR.I(npbar,2) < PBAR.I(npbar,3))
                                fclose(fp);
                                error('Wrong Bar property %d inertia definition.', PBAR.ID(npbar));
                                
                            end
                        else
                            skip_line = true;
                        end
                    else
                        skip_line = true;
                    end
                    
                    % MAT1 card
                case keyword{6}
                    
                    nmat = nmat +1;
                    
                    MAT.ID(nmat) = int32(num_field_parser(tline, 2));
                    MAT.E(nmat) = num_field_parser(tline, 3);
                    MAT.G(nmat) = num_field_parser(tline, 4);
                    MAT.nu(nmat) = num_field_parser(tline, 5);
                    MAT.Rho(nmat) = num_field_parser(tline, 6);
                    MAT.ST(nmat) = 0;
                    MAT.SC(nmat) = 0;
                    MAT.SS(nmat) = 0;
                    
                    % try to read continuation
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        MAT.ST(nmat) = num_field_parser(tline, 2);
                        MAT.SC(nmat) = num_field_parser(tline, 3);
                        MAT.SS(nmat) = num_field_parser(tline, 4);
                        
                    else
                        
                        skip_line = true;
                    end
                    
                    % check for consistent database
                    if ( (MAT.E(nmat)==0) && (MAT.G(nmat)==0) )
                        fclose(fp);
                        error('Inconsistent definition of material property material %d. Both E and G are null.', MAT.ID(nmat));
                        
                    end
                    
                    if (MAT.E(nmat)==0)
                        
                        MAT.E(nmat) = 2 * (1 + MAT.nu(nmat)) * MAT.G(nmat);
                        
                    end
                    
                    if (MAT.G(nmat)==0)
                        
                        MAT.G(nmat) = MAT.E(nmat) / (2 * (1 + MAT.nu(nmat)));
                        
                    end
                    
                    if (MAT.nu(nmat)==0) % nu is not used for beam formulation but is anyway set
                        
                        MAT.nu(nmat) = MAT.E(nmat) / (2 * MAT.G(nmat)) -1;
                        
                    end
                    
                    % CONM1 card
                case keyword{7}
                    
                    ncom1 = ncom1 +1;
                    
                    CONM1.ID(ncom1) = int32(num_field_parser(tline, 2));
                    CONM1.Node(ncom1) = int32(num_field_parser(tline, 3));
                    CONM1.CID(ncom1) = int32(num_field_parser(tline, 4));
                    CONM1.M(:,:,ncom1) = zeros(6,6);
                    
                    CONM1.M(1,1,ncom1) = num_field_parser(tline, 5);
                    CONM1.M(2,1,ncom1) = num_field_parser(tline, 6);
                    CONM1.M(2,2,ncom1) = num_field_parser(tline, 7);
                    CONM1.M(3,1,ncom1) = num_field_parser(tline, 8);
                    CONM1.M(3,2,ncom1) = num_field_parser(tline, 9);
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        
                        CONM1.M(3,3,ncom1) = num_field_parser(tline, 2);
                        CONM1.M(4,1,ncom1) = num_field_parser(tline, 3);
                        CONM1.M(4,2,ncom1) = num_field_parser(tline, 4);
                        CONM1.M(4,3,ncom1) = num_field_parser(tline, 5);
                        CONM1.M(4,4,ncom1) = num_field_parser(tline, 6);
                        CONM1.M(5,1,ncom1) = num_field_parser(tline, 7);
                        CONM1.M(5,2,ncom1) = num_field_parser(tline, 8);
                        CONM1.M(5,3,ncom1) = num_field_parser(tline, 9);
                        
                        tline = fgetl(fp);
                        
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            
                            skip_line = false;
                            
                            CONM1.M(5,4,ncom1) = num_field_parser(tline, 2);
                            CONM1.M(5,5,ncom1) = num_field_parser(tline, 3);
                            CONM1.M(6,1,ncom1) = num_field_parser(tline, 4);
                            CONM1.M(6,2,ncom1) = num_field_parser(tline, 5);
                            CONM1.M(6,3,ncom1) = num_field_parser(tline, 6);
                            CONM1.M(6,4,ncom1) = num_field_parser(tline, 7);
                            CONM1.M(6,5,ncom1) = num_field_parser(tline, 8);
                            CONM1.M(6,6,ncom1) = num_field_parser(tline, 9);
                            
                        else
                            
                            skip_line = true;
                            
                        end
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                    
                    for c=1:5 % c index
                        for r = (c+1):6 % r index
                            CONM1.M(c,r,ncom1) = CONM1.M(r,c,ncom1);
                        end
                    end
                    
                    % CONM2 card
                case keyword{8}
                    
                    ncom2 = ncom2 +1;
                    
                    CONM2.ID(ncom2) = int32(num_field_parser(tline, 2));
                    CONM2.Node(ncom2) = int32(num_field_parser(tline, 3));
                    CONM2.CID(ncom2) = int32(num_field_parser(tline, 4));
                    CONM2.M(:,:,ncom2) = zeros(6,6);
                    CONM2.M(1,1,ncom2) = num_field_parser(tline, 5);
                    CONM2.M(2,2,ncom2) = CONM2.M(1,1,ncom2);
                    CONM2.M(3,3,ncom2) = CONM2.M(1,1,ncom2);
                    
                    CONM2.Offset(ncom2,1:3) = zeros(1,3);
                    CONM2.Offset(ncom2,1) = num_field_parser(tline, 6);
                    CONM2.Offset(ncom2,2) = num_field_parser(tline, 7);
                    CONM2.Offset(ncom2,3) = num_field_parser(tline, 8);
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        
                        CONM2.M(4,4,ncom2) = num_field_parser(tline, 2);
                        CONM2.M(5,4,ncom2) = -num_field_parser(tline, 3);
                        CONM2.M(5,5,ncom2) = num_field_parser(tline, 4);
                        CONM2.M(6,4,ncom2) = -num_field_parser(tline, 5);
                        CONM2.M(6,5,ncom2) = -num_field_parser(tline, 6);
                        CONM2.M(6,6,ncom2) = num_field_parser(tline, 7);
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                    for c=4:5 % c index
                        for r = (c+1):6 % r index
                            CONM2.M(c,r,ncom2) = CONM2.M(r,c,ncom2);
                        end
                    end
                    
                    % PARAM card
                case keyword{9}
                    
                    param = string_field_parser(tline, 2);
                    
                    switch param
                        
                        case 'DLM_KMAX'
                            PARAM.DLM_KMAX = num_field_parser(tline, 3);

                        case 'DLM_AR'
                            PARAM.DLM_AR = num_field_parser(tline, 3);

                        case 'DLM_NP'
                            PARAM.DLM_NP = num_field_parser(tline, 3);

                        case 'KDAMP'

                            PARAM.KDAMP = num_field_parser(tline, 3);

                        case 'VREF'

                            PARAM.VREF = num_field_parser(tline, 3);

                        case 'MACH'

                            PARAM.MACH = num_field_parser(tline, 3);

                        case 'RHOREF'

                            PARAM.RHOREF = num_field_parser(tline, 3);

                        case 'MODACC'

                            PARAM.MODACC = num_field_parser(tline, 3);

                        case 'DIRDER'
                            
                            PARAM.DIRDER = num_field_parser(tline, 3);
                            
                        case 'EPS'
                            
                            PARAM.EPS = num_field_parser(tline, 3);
                            
                        case 'NSTEP'
                            
                            PARAM.NSTEP = abs(num_field_parser(tline, 3));
                            
                        case 'NITER'
                            
                            PARAM.NITER = abs(num_field_parser(tline, 3));
                            
                        case 'RES_TOL'
                            
                            PARAM.RES_TOL = num_field_parser(tline, 3);
                            
                        case 'REL_FAC'
                            
                            PARAM.REL_FAC = abs(num_field_parser(tline, 3));
                            if (PARAM.REL_FAC>1.0)
                                fclose(fp);
                                error('Wrong REF_FAC parameter value (greater than 1.0).');
                            end
                            
                        case 'GRDPNT'
                            
                            PARAM.GRDPNT = int32(num_field_parser(tline, 3));
                            
                        case 'WTMASS'
                            
                            PARAM.WTMASS = num_field_parser(tline, 3);
                            if (PARAM.WTMASS==0)
                                fclose(fp);
                                error('Null WTMASS parameter given.');
                            end
                            
                        case 'EIG_FILE'
                            
                            tline = tline(2*FIELD+1:end);
                            index = find(tline == '''');
                            if isempty(index)
                                fclose(fp);error('Wrong head filename form for EIG_FILE.');
                            end
                            PARAM.EIG_FILE = tline(index(1)+1:index(2)-1);
                            
                        case 'DIVERG'
                            
                            PARAM.DIVERG = num_field_parser(tline, 3);
                            
                        case 'OBJ'
                            
                            PARAM.OBJ = eval(['@', string_field_parser(tline, 3)]);
                            
                        case 'DESVAR'
                            
                            PARAM.DESVAR = eval(['@', string_field_parser(tline, 3)]);
                            
                        case 'DLINK'
                            
                            PARAM.DLINK = eval(['@', string_field_parser(tline, 3)]);
                            
                        case 'FUSE_DP'
                            
                            PARAM.FUSE_DP = num_field_parser(tline, 3);
                            
                        case 'AUTOPLOT'
                            
                            PARAM.AUTOPLOT = num_field_parser(tline, 3);
                            
                        case 'OPTDVAR'
                            
                            PARAM.DIVERG = num_field_parser(tline, 3);

                        case 'LANDG'

                            LANDG = num_field_parser(tline, 3);
                            PARAM.LANDG = [PARAM.LANDG; LANDG];

                        case 'DER_FILE'
                            
                            tline = tline(2*FIELD+1:end);
                            index = find(tline == '''');
                            if isempty(index)
                                fclose(fp);error('Wrong head filename form for DER_FILE.');
                            end
                            PARAM.DER_FILE = tline(index(1)+1:index(2)-1);

                        case 'DER_TYPE'

                            PARAM.DER_TYPE = num_field_parser(tline, 3);
                            if PARAM.DER_TYPE>2 || PARAM.DER_TYPE<0
                              error('Wrong value given for PARAM DER_TYPE.');
                            end

                        case 'AEROINT'

                        LINE = [];
                        [tline, rem] = strtok(tline);
                        [tline, rem] = strtok(rem);
                        LINE = [LINE, rem];
                        skip_line = false;
                        while ~skip_line
                            tline = fgetl(fp);
                            if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                              LINE = [LINE, tline];
                            else
                                skip_line = true;
                            end
                        end
                        AEROINT = unique(read_set_file(LINE,'AEROINT'));
                        index = find(AEROINT);
                        PARAM.AEROINT = [PARAM.AEROINT, unique(AEROINT(index))];

                    end % end param
                    
                    % all FORCES cards have the same struct
                    % FORCE card
                case keyword{10}
                    
                    nforce = nforce +1;
                    
                    FORCE.Type(nforce) = uint8(0);
                    FORCE.ID(nforce) = int32(num_field_parser(tline, 2));
                    FORCE.Node(nforce) = int32(num_field_parser(tline, 3));
                    FORCE.CID(nforce) = int32(num_field_parser(tline, 4));
                    FORCE.Mag(nforce) = num_field_parser(tline, 5);
                    FORCE.Orient(nforce,:) = zeros(1,3);
                    FORCE.Orient(nforce, 1) = num_field_parser(tline, 6);
                    FORCE.Orient(nforce, 2) = num_field_parser(tline, 7);
                    FORCE.Orient(nforce, 3) = num_field_parser(tline, 8);
                    
                    % FORCE1 card
                case keyword{11}
                    
                    nforce = nforce +1;
                    
                    FORCE.Type(nforce) = uint8(1);
                    FORCE.ID(nforce) = int32(num_field_parser(tline, 2));
                    FORCE.Node(nforce) = int32(num_field_parser(tline, 3));
                    FORCE.Mag(nforce) = num_field_parser(tline, 4);
                    FORCE.Orient(nforce,:) = zeros(1,3);
                    FORCE.Orient(nforce, 1) = num_field_parser(tline, 5);
                    FORCE.Orient(nforce, 2) = num_field_parser(tline, 6);
                    FORCE.CID(nforce) = int32(0);
                    % FORCE2 card
                case keyword{12}
                    
                    nforce = nforce +1;
                    
                    FORCE.Type(nforce) = uint8(2);
                    FORCE.ID(nforce) = int32(num_field_parser(tline, 2));
                    FORCE.Node(nforce) = int32(num_field_parser(tline, 3));
                    FORCE.Mag(nforce) = num_field_parser(tline, 4);
                    FORCE.Orient(nforce,:) = zeros(1,3);
                    FORCE.Orient(nforce, 1) = num_field_parser(tline, 5);
                    FORCE.Orient(nforce, 2) = num_field_parser(tline, 6);
                    FORCE.Orient(nforce, 3) = num_field_parser(tline, 7);
                    % dummy storage for the fourth point
                    FORCE.CID(nforce) = int32(num_field_parser(tline, 8));
                    
                    % MOMENT card
                case keyword{13}
                    
                    nmom = nmom +1;
                    
                    MOMENT.Type(nmom) = uint8(0);
                    MOMENT.ID(nmom) = int32(num_field_parser(tline, 2));
                    MOMENT.Node(nmom) = int32(num_field_parser(tline, 3));
                    MOMENT.CID(nmom) = int32(num_field_parser(tline, 4));
                    MOMENT.Mag(nmom) = num_field_parser(tline, 5);
                    MOMENT.Orient(nmom,:) = zeros(1,3);
                    MOMENT.Orient(nmom, 1) = num_field_parser(tline, 6);
                    MOMENT.Orient(nmom, 2) = num_field_parser(tline, 7);
                    MOMENT.Orient(nmom, 3) = num_field_parser(tline, 8);
                    
                    % MOMENT1 card
                case keyword{14}
                    
                    nmom = nmom +1;
                    
                    MOMENT.Type(nmom) = uint8(1);
                    MOMENT.ID(nmom) = int32(num_field_parser(tline, 2));
                    MOMENT.Node(nmom) = int32(num_field_parser(tline, 3));
                    MOMENT.Mag(nmom) = num_field_parser(tline, 4);
                    MOMENT.Orient(nmom,:) = zeros(1,3);
                    MOMENT.Orient(nmom, 1) = num_field_parser(tline, 5);
                    MOMENT.Orient(nmom, 2) = num_field_parser(tline, 6);
                    MOMENT.CID(nmom) = int32(0);
                    % MOMENT2 card
                case keyword{15}
                    
                    nmom = nmom +1;
                    
                    MOMENT.Type(nmom) = uint8(2);
                    MOMENT.ID(nmom) = int32(num_field_parser(tline, 2));
                    MOMENT.Node(nmom) = int32(num_field_parser(tline, 3));
                    MOMENT.Mag(nmom) = num_field_parser(tline, 4);
                    MOMENT.Orient(nmom,:) = zeros(1,3);
                    MOMENT.Orient(nmom, 1) = num_field_parser(tline, 5);
                    MOMENT.Orient(nmom, 2) = num_field_parser(tline, 6);
                    MOMENT.Orient(nmom, 3) = num_field_parser(tline, 7);
                    % dummy storage for the fourth point
                    MOMENT.CID(nmom) = int32(num_field_parser(tline, 8));
                    
                case keyword{16} % LOAD= case control card
                    
                    [tline, rem] = strtok(tline);
                    
                    PARAM.LOAD = int32(str2num(rem));
                    if (PARAM.LOAD == 0)
                        fclose(fp);error('Load set must be different from zero.');
                    end
                    nload = nload +1;
                    if nload > 1
                        fclose(fp);error('Multiple LOAD requests. Please check.');
                    end
                    
                case keyword{17} % SPC= case control card
                    
                    [tline, rem] = strtok(tline);
                    
                    PARAM.SPC = int32(str2num(rem));
                    cc_nspc = cc_nspc +1;
                    if cc_nspc > 1
                        fclose(fp);error('Multiple SPC requests. Please check.');
                    end
                    
                case keyword{18} % SPC1 card
                    
                    nspc = nspc +1;
                    
                    SPC.ID(nspc) = int32(num_field_parser(tline, 2));
                    constr = string_field_parser(tline, 3);
                    SPC.DOF(nspc).list = zeros(1, length(constr));
                    if length(constr) > 6
                        fclose(fp);error('Constrained too many dofs in SPC set %d.', SPC.ID(nspc));
                    end
                    
                    for i = 1:length(constr)
                        SPC.DOF(nspc).list(i) = int32(str2num(constr(i)));
                    end
                    i = 0;
                    for j= 4:9
                        i = i+1;
                        SPC.Nodes(nspc).list(i) = int32(num_field_parser(tline, j));
                    end
                    
                    SPC.Nodes(nspc).list = sort(SPC.Nodes(nspc).list);
                    j = find(SPC.Nodes(nspc).list);
                    SPC.Nodes(nspc).list = SPC.Nodes(nspc).list(j);
                    i = length(j);
                    
                    skip_line = false;
                    
                    while ~skip_line
                        
                        tline = fgetl(fp);
                        
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            
                            skip_line = false;
                            for j= 2:9
                                i = i+1;
                                SPC.Nodes(nspc).list(i) = int32(num_field_parser(tline, j));
                            end
                            
                        else
                            
                            skip_line = true;
                            
                        end
                        
                        SPC.Nodes(nspc).list = sort(SPC.Nodes(nspc).list);
                        j = find(SPC.Nodes(nspc).list);
                        SPC.Nodes(nspc).list = SPC.Nodes(nspc).list(j);
                        i = length(j);
                        
                    end
                    
                case keyword{19} % SOL card
                    
                    [rem, sol] = strtok(tline);
                    [sol, rem] = strtok(sol);
                    
                    PARAM.SOL = str2num(sol);
                    
                case keyword{20} % BEAM struct
                    
                    ncbeam = ncbeam +1;
                    
                    BEAM.ID(ncbeam) = int32(num_field_parser(tline, 2));
                    if ~BEAM.ID(ncbeam)
                        fclose(fp);error('Given null Beam ID.');
                    end
                    BEAM.PID(ncbeam) = int32(num_field_parser(tline, 3));
                    BEAM.Conn(ncbeam,1) = int32(num_field_parser(tline, 4));
                    BEAM.Conn(ncbeam,2) = int32(num_field_parser(tline, 5));
                    BEAM.Conn(ncbeam,3) = 0; % create the third field
                    
                    g0 = string_field_parser(tline, 6);
                    i = find(g0  == '.'); % check if real or integer
                    
                    if isempty(i)
                        % integer
                        beamg0(ncbeam) = true;
                        
                    else
                        % real
                        beamg0(ncbeam) = false;
                        
                    end
                    
                    BEAM.Orient(ncbeam,1) = num_field_parser(tline, 6);
                    BEAM.Orient(ncbeam,2) = num_field_parser(tline, 7);
                    BEAM.Orient(ncbeam,3) = num_field_parser(tline, 8);
                    
                    BEAM.OffsetT(ncbeam) = 1; % set to default value
                    type = string_field_parser(tline, 9);
                    
                    if ~isempty(type)
                        
                        switch(type)
                            
                            case 'GGG'
                                
                                BEAM.OffsetT(ncbeam) = 1;
                                
                            case 'BGG'
                                
                                BEAM.OffsetT(ncbeam) = 2;
                                
                            case 'GGO'
                                
                                BEAM.OffsetT(ncbeam) = 3;
                                
                            case 'BGO'
                                
                                BEAM.OffsetT(ncbeam) = 4;
                                
                            case 'GOG'
                                
                                BEAM.OffsetT(ncbeam) = 5;
                                
                            case 'BOG'
                                
                                BEAM.OffsetT(ncbeam) = 6;
                                
                            case 'GOO'
                                
                                BEAM.OffsetT(ncbeam) = 7;
                                
                            case 'BOO'
                                
                                BEAM.OffsetT(ncbeam) = 8;
                                
                            otherwise
                                
                                fclose(fp);error('Unknown offset option for CBEAM %d.', BEAM.ID(ncbeam));
                                
                        end
                    end
                    
                    BEAM.Offset(ncbeam,1:9) = zeros(1,9);
                    
                    % try to read continuation
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        
                        for n=1:3
                            
                            BEAM.Offset(ncbeam,n) = num_field_parser(tline, 3+n);
                            
                        end
                        for n=7:9
                            
                            BEAM.Offset(ncbeam,n) = num_field_parser(tline, n);
                            
                        end
                        
                        
                        skip_line = false;
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                    tline = fgetl(fp); % skip the third cbeam line
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                case keyword{21} % PBEAM struct
                    
                    npbeam = npbeam +1;
                    
                    PBEAM.ID(npbeam) = int32(num_field_parser(tline, 2));
                    PBEAM.Mat(npbeam) = int32(num_field_parser(tline, 3));
                    PBEAM.A(npbeam).data(1) = num_field_parser(tline, 4);
                    PBEAM.I(npbeam).data(1,1) = num_field_parser(tline, 5);
                    PBEAM.I(npbeam).data(2,1) = num_field_parser(tline, 6);
                    PBEAM.I(npbeam).data(3,1) = num_field_parser(tline, 7);
                    PBEAM.J(npbeam).data(1) = num_field_parser(tline, 8);
                    PBEAM.RhoNS(npbeam).data(1) = num_field_parser(tline, 9);
                    
                    PBEAM.X_L(npbeam).data(1) = [0.0];
                    PBEAM.Kshear(npbeam, 1:2) = zeros(1,2);
                    PBEAM.NSI(npbeam,1:2) = zeros(1,2);
                    PBEAM.NSCG(npbeam,1:4) = zeros(1,4);
                    PBEAM.NA(npbeam,1:4) = zeros(1,4);
                    
                    % try to read continuation
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        
                    else
                        
                        fclose(fp);error('Missing continuation card in Beam property %d.', PBEAM.ID(npbeam));
                        
                    end
                    % skip all the previous line which containt stress recovery points
                    
                    ENDB_FOUND = false;
                    
                    % read midsection properties
                    i = 1;
                    while ~ENDB_FOUND
                        
                        tline = fgetl(fp);
                        
                        i = i+1;
                        PBEAM.X_L(npbeam).data(i) = num_field_parser(tline, 3);
                        PBEAM.A(npbeam).data(i) = num_field_parser(tline, 4);
                        PBEAM.I(npbeam).data(1,i) = num_field_parser(tline, 5);
                        PBEAM.I(npbeam).data(2,i) = num_field_parser(tline, 6);
                        PBEAM.I(npbeam).data(3,i) = num_field_parser(tline, 7);
                        PBEAM.J(npbeam).data(i) = num_field_parser(tline, 8);
                        PBEAM.RhoNS(npbeam).data(i) = num_field_parser(tline, 9);
                        
                        if PBEAM.X_L(npbeam).data(i) >= 1.0
                            
                            ENDB_FOUND = true;
                            
                        end
                        
                        tline = fgetl(fp); % skip stresss recovery line
                    end
                    % check for end B data
                    
                    if PBEAM.A(npbeam).data(end) == 0
                        PBEAM.A(npbeam).data(end) = PBEAM.A(npbeam).data(1);
                    end
                    if PBEAM.I(npbeam).data(1, end) == 0
                        PBEAM.I(npbeam).data(1, end) = PBEAM.I(npbeam).data(1, 1);
                    end
                    if PBEAM.I(npbeam).data(2, end) == 0
                        PBEAM.I(npbeam).data(2, end) = PBEAM.I(npbeam).data(2, 1);
                    end
                    if PBEAM.I(npbeam).data(3, end) == 0
                        PBEAM.I(npbeam).data(3, end) = PBEAM.I(npbeam).data(3, 1);
                    end
                    if PBEAM.J(npbeam).data(end) == 0
                        PBEAM.J(npbeam).data(end) = PBEAM.J(npbeam).data(1);
                    end
                    if PBEAM.RhoNS(npbeam).data(end) == 0
                        PBEAM.RhoNS(npbeam).data(end) = PBEAM.RhoNS(npbeam).data(1);
                    end
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        
                        PBEAM.Kshear(npbeam,1) = num_field_parser_1(tline, 2);
                        PBEAM.Kshear(npbeam,2) = num_field_parser_1(tline, 3);
                        PBEAM.NSI(npbeam,1) = num_field_parser(tline, 6);
                        PBEAM.NSI(npbeam,2) = num_field_parser(tline, 7);
                        
                        tline = fgetl(fp);
                        
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            
                            skip_line = false;
                            
                            PBEAM.NSCG(npbeam,1) = num_field_parser(tline, 2);
                            PBEAM.NSCG(npbeam,2) = num_field_parser(tline, 3);
                            PBEAM.NSCG(npbeam,3) = num_field_parser(tline, 4);
                            PBEAM.NSCG(npbeam,4) = num_field_parser(tline, 5);
                            
                            PBEAM.NA(npbeam,1) = num_field_parser(tline, 6);
                            PBEAM.NA(npbeam,2) = num_field_parser(tline, 7);
                            PBEAM.NA(npbeam,3) = num_field_parser(tline, 8);
                            PBEAM.NA(npbeam,4) = num_field_parser(tline, 9);
                            
                        else
                            
                            skip_line = true;
                            
                        end
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                    % check for midsection missing data and interpolate them
                    ns = i;
                    
                    PBEAM.A(npbeam).data      = ...
                        interp_tapered_beam_data(ns, PBEAM.A(npbeam).data,      PBEAM.X_L(npbeam).data);
                    PBEAM.I(npbeam).data(1,:) = ...
                        interp_tapered_beam_data(ns, PBEAM.I(npbeam).data(1,:), PBEAM.X_L(npbeam).data);
                    PBEAM.I(npbeam).data(2,:) = ...
                        interp_tapered_beam_data(ns, PBEAM.I(npbeam).data(2,:), PBEAM.X_L(npbeam).data);
                    PBEAM.I(npbeam).data(3,:) = ...
                        interp_tapered_beam_data(ns, PBEAM.I(npbeam).data(3,:), PBEAM.X_L(npbeam).data);
                    PBEAM.J(npbeam).data      = ...
                        interp_tapered_beam_data(ns, PBEAM.J(npbeam).data,      PBEAM.X_L(npbeam).data);
                    PBEAM.RhoNS(npbeam).data  = ...
                        interp_tapered_beam_data(ns, PBEAM.RhoNS(npbeam).data,  PBEAM.X_L(npbeam).data);
                    
                    for i=1:ns
                        
                        if (PBEAM.I(npbeam).data(1, i) * PBEAM.I(npbeam).data(1, i)) < (PBEAM.I(npbeam).data(3, i))
                            
                            fclose(fp);error('Wrong Beam property %d inertia definition.', PBEAM.ID(npbeam));
                            
                        end
                    end
                    
                    % interpolate structural properties on collocation points
                    
                    PBEAM.DA(:, :, npbeam) = zeros(1,2);
                    PBEAM.DI(:, :, npbeam) = zeros(3,2);
                    PBEAM.DJ(:, :, npbeam) =  zeros(1,2);
                    PBEAM.DRhoNS(:, :, npbeam) =  zeros(1,2);
                    
                    PBEAM.DA(1, 1:2, npbeam) = interp1(PBEAM.X_L(npbeam).data,...
                        PBEAM.A(npbeam).data,	  [0.5 - 1/(2*sqrt(3)), 0.5 + 1/(2*sqrt(3))]);
                    PBEAM.DI(1, 1:2, npbeam) = interp1(PBEAM.X_L(npbeam).data,...
                        PBEAM.I(npbeam).data(1,:), [0.5 - 1/(2*sqrt(3)), 0.5 + 1/(2*sqrt(3))]);
                    PBEAM.DI(2, 1:2, npbeam) = interp1(PBEAM.X_L(npbeam).data,...
                        PBEAM.I(npbeam).data(2,:), [0.5 - 1/(2*sqrt(3)), 0.5 + 1/(2*sqrt(3))]);
                    PBEAM.DI(3, 1:2, npbeam) = interp1(PBEAM.X_L(npbeam).data,...
                        PBEAM.I(npbeam).data(3,:), [0.5 - 1/(2*sqrt(3)), 0.5 + 1/(2*sqrt(3))]);
                    PBEAM.DJ(1, 1:2, npbeam) = interp1(PBEAM.X_L(npbeam).data,...
                        PBEAM.J(npbeam).data,	  [0.5 - 1/(2*sqrt(3)), 0.5 + 1/(2*sqrt(3))]);
                    PBEAM.DRhoNS(1, 1:2, npbeam) = interp1(PBEAM.X_L(npbeam).data,...
                        PBEAM.RhoNS(npbeam).data,  [0.5 - 1/(2*sqrt(3)), 0.5 + 1/(2*sqrt(3))]);
                    
                case keyword{22} % CAERO1 struct
                    
                    ncaero = ncaero +1;
                    t = 1;
                    
                    CAERO.geo.nelem(ncaero) = 1;
                    CAERO.ID(ncaero) = int32(num_field_parser(tline, 2));
                    
                    %CAERO.EID(ncaero) = int32(num_field_parser(tline, 3)); eliminated
                    CAERO.geo.dihed(ncaero, 1) = D2R(num_field_parser(tline, 3));
                    
                    CAERO.CP(ncaero) = int32(num_field_parser(tline, 4));
                    CAERO.geo.ny(ncaero,1) = int32(num_field_parser(tline, 5));
                    if (CAERO.geo.ny(ncaero,1) == 0)
                        fclose(fp);error('Read null span panel numbers for CAERO entry %d.', CAERO.ID(ncaero));
                    end
                    
                    CAERO.geo.nx(ncaero,1) = int32(num_field_parser(tline, 6));
                    if (CAERO.geo.nx(ncaero,1) == 0)
                        fclose(fp);error('Read null chord panel numbers for CAERO entry %d.', CAERO.ID(ncaero));
                    end
                    
                    foil = string_field_parser(tline, 7);
                    fprintf(fid,'\n\tCAERO1 %d airfoils:', CAERO.ID(ncaero));
                    if isempty(foil)
                        CAERO.geo.foil(ncaero, 1, 1) = {'0'};
                        fprintf(fid, '\n\t\t### Warning: no root foil provided. Flat plate assumed.');
                    else
                        
                      if (~isempty(str2num(foil(1:4))) && length(deblank(foil) == 4)) % check if 4 digits provided
                            CAERO.geo.foil(ncaero, 1, 1) = {foil};
                            fprintf(fid, '\n\t\tRoot: 4 digits NACA %s.', foil);
                      else
                        if (exist(strcat(foil, '.dat'), 'file'))
                          foil = strcat(foil, '.dat');
                        elseif (exist(strcat(foil, '.DAT'), 'file'))
                          foil = strcat(foil, '.DAT');
                        else
                          msg = ['Unable to find airfoil ', foil];
                          error(msg);
                        end
                        CAERO.geo.foil(ncaero, 1, 1) = {foil};
                        fprintf(fid, '\n\t\tRoot: %s.', foil);
                      end
                    end

                    foil = string_field_parser(tline, 8);
                    if isempty(foil)
                        CAERO.geo.foil(ncaero, 1, 2) = {'0'};
                        fprintf(fid, '\n\t\t### Warning: no tip foil provided.  Flat plate assumed.');
                    else
                        
                      if (~isempty(str2num(foil(1:4))) && length(deblank(foil) == 4)) % check if 4 digits provided
                            CAERO.geo.foil(ncaero, 1, 2) = {foil};
                            fprintf(fid, '\n\t\tTip: 4 digits NACA %s.', foil);
                      else
                        if (exist(strcat(foil, '.dat'), 'file'))
                          foil = strcat(foil, '.dat');
                        elseif (exist(strcat(foil, '.DAT'), 'file'))
                          foil = strcat(foil, '.DAT');
                        else
                          msg = ['Unable to find airfoil ', foil];
                          error(msg);
                        end
                        CAERO.geo.foil(ncaero, 1, 2) = {foil};
                        fprintf(fid, '\n\t\tTip: %s.', foil);
                      end
                    end
                    
                    CAERO.geo.meshtype(ncaero,1) = int32(num_field_parser(tline, 9));
                    if ((CAERO.geo.meshtype(ncaero,t) <1) || (CAERO.geo.meshtype(ncaero,t) >4) && (CAERO.geo.meshtype(ncaero,t) <6))
                        
                        CAERO.geo.meshtype(ncaero,1)  = 1;
                        
                    end
                    %
                    CAERO.INT(ncaero) = int32(num_field_parser(tline, 10));
                    %
                    % try to read continuation
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        CAERO.geo.startx(ncaero) = num_field_parser(tline, 2);
                        CAERO.geo.starty(ncaero) = num_field_parser(tline, 3);
                        CAERO.geo.startz(ncaero) = num_field_parser(tline, 4);
                        % WARNING: the meaning of the following fields is changed for Tornado compatibility
                        CAERO.geo.c(ncaero)       = num_field_parser(tline, 5);
                        CAERO.geo.b(ncaero,1)     = num_field_parser(tline, 6);
                        CAERO.geo.T(ncaero,1)     = num_field_parser(tline, 7);
                        CAERO.geo.SW(ncaero,1)    = D2R(num_field_parser(tline, 8));
                        CAERO.geo.TW(ncaero,1,1)  = D2R(num_field_parser(tline, 9));
                        CAERO.geo.TW(ncaero,1,2)  = D2R(num_field_parser(tline, 10));
                        
                        skip_line = false;
                        
                    else
                        
                        fclose(fp);error('Unable to find continuation card for CAERO entry %d.', CAERO.geo.ID(ncaero));
                        
                    end
                    
                    CAERO.geo.flapped(ncaero,1) = 0;
                    CAERO.geo.fc(ncaero,1,1) = 0;
                    CAERO.geo.fc(ncaero,1,2) = 0;
                    CAERO.geo.fnx(ncaero,1) = 0;
                    CAERO.geo.fsym(ncaero,1) = 0;
                    
                    % try to read continuation with FLAP data
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        CAERO.geo.flapped(ncaero,1) = int8(num_field_parser(tline, 2));
                        if CAERO.geo.flapped(ncaero,1)
                            
                            CAERO.geo.nc = CAERO.geo.nc +1;
                            CAERO.geo.fc(ncaero,1,1) =  num_field_parser(tline, 3);
                            CAERO.geo.fc(ncaero,1,2) =  num_field_parser(tline, 4);
                            CAERO.geo.fnx(ncaero,1) = num_field_parser(tline, 5);
                            CONTROL_NAME(CAERO.geo.nc) = {name_field_parser(tline, 6)};
                            if ~length( cell2mat(CONTROL_NAME(CAERO.geo.nc)))
                                fclose(fp);error('No name given for control surface belonging to patch %d.', CAERO.ID(ncaero));
                            end
                            %CAERO.geo.fsym(ncaero,1) = int32(num_field_parser(tline, 4)); always zero.
                            % this feature is never used and is substitute by AELINK card
                        end
                    else
                        
                        skip_line = true;
                        
                    end
                    
                    % try to read partitions
                    
                    if ~skip_line
                        
                        skip_line = false;
                        
                        while ~skip_line
                            
                            tline = fgetl(fp);
                            
                            if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                                
                                skip_line = false;
                                t = t+1;
                                CAERO.geo.nelem(ncaero) = t;
                                CAERO.geo.nx(ncaero,t) = CAERO.geo.nx(ncaero,1);
                                
                                CAERO.geo.dihed(ncaero, t) = D2R(num_field_parser(tline, 2));
                                CAERO.geo.ny(ncaero, t) = int32(num_field_parser(tline, 3));
                                CAERO.geo.b(ncaero,t) = num_field_parser(tline, 4);
                                CAERO.geo.T(ncaero,t) = num_field_parser(tline, 5);
                                CAERO.geo.foil(ncaero,t,1) = CAERO.geo.foil(ncaero,t-1,2);
                                
                                foil = string_field_parser(tline, 6);
                                if isempty(foil)
                                    CAERO.geo.foil(ncaero, t, 2) = {'0'};
                                else
                                    
                                    if length(foil) > 1
                                        if (isempty(str2num(foil(1))))
                                          if (exist(strcat(foil, '.dat'), 'file'))
                                            foil = strcat(foil, '.dat');
                                          elseif (exist(strcat(foil, '.DAT'), 'file'))
                                            foil = strcat(foil, '.DAT');
                                          else
                                            msg = ['Unable to find airfoil ', foil];
                                            error(msg);
                                          end
                                        end
                                        CAERO.geo.foil(ncaero, t, 2) = {foil};
                                    else
                                        CAERO.geo.foil(ncaero, t, 2) = {'0'};
                                    end
                                end
                                
                                CAERO.geo.SW(ncaero,t)    = D2R(num_field_parser(tline, 7));
                                CAERO.geo.TW(ncaero,t,1)  = CAERO.geo.TW(ncaero,t-1,2);
                                CAERO.geo.TW(ncaero,t,2)  = D2R(num_field_parser(tline, 8));
                                CAERO.geo.meshtype(ncaero,t) = int32(num_field_parser(tline, 9));
                                if ((CAERO.geo.meshtype(ncaero,t) <1) || (CAERO.geo.meshtype(ncaero,t) >4) && (CAERO.geo.meshtype(ncaero,t) <6))
                                    
                                    CAERO.geo.meshtype(ncaero,t)  = 1;
                                    
                                end
                                
                                CAERO.geo.flapped(ncaero,t) = 0;
                                CAERO.geo.fc(ncaero,t,1) = 0;
                                CAERO.geo.fc(ncaero,t,2) = 0;
                                CAERO.geo.fnx(ncaero,t) = 0;
                                CAERO.geo.fsym(ncaero,t) = 0;
                                
                                % try to read continuation with flap data
                                tline = fgetl(fp);
                                
                                if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                                    
                                    skip_line = false;
                                    CAERO.geo.flapped(ncaero,t) = int8(num_field_parser(tline, 2));
                                    if CAERO.geo.flapped(ncaero,t)
                                        
                                        CAERO.geo.nc = CAERO.geo.nc +1;
                                        CAERO.geo.fc(ncaero,t,1) =  num_field_parser(tline, 3);
                                        CAERO.geo.fc(ncaero,t,2) =  num_field_parser(tline, 4);
                                        CAERO.geo.fnx(ncaero,t) = num_field_parser(tline, 5);
                                        CONTROL_NAME(CAERO.geo.nc) = {name_field_parser(tline, 6)};
                                        if ~length( cell2mat(CONTROL_NAME(CAERO.geo.nc)))
                                            fclose(fp);error('No name given for control surface belonging to partition %d on patch %d.', t, CAERO.ID(ncaero));
                                        end
                                        %CAERO.geo.fsym(ncaero,1) = int32(num_field_parser(tline, 4)); always zero
                                    end
                                else
                                    
                                    skip_line = true;
                                    
                                end
                                
                            else
                                skip_line = true;
                            end
                            
                        end
                        
                    end
                    
                case keyword{23} % AEROS card (steady VLM)
                    naeros
                    if naeros
                        fclose(fp);error('Multiple AEROS cards given.');
                    end
                    naeros = naeros + 1;
                    CAERO.Trim.CID = int32(num_field_parser(tline, 3)); % if equal to -1 use body principal axis
                    if ( (CAERO.Trim.CID~=0) && ((CAERO.Trim.CID~=-1)) )
                        fclose(fp);error('Coordinate system for trim analysis must always be the 0 global reference frame.');
                    end
                    CREF_VLM = num_field_parser(tline, 4);
                    BREF_VLM = num_field_parser(tline, 5);
                    SREF_VLM = num_field_parser(tline, 6);
                    SIMXZ_VLM = int8(num_field_parser(tline, 7));
                    if (SIMXZ_VLM < 0)
                        fclose(fp);error('Antisymmetric boundary conditions not available for VLM.');
                    end
                    SIMXY_VLM = int8(num_field_parser(tline, 8));
                    if (SIMXY_VLM < 0)
                        fclose(fp);error('Antisymmetric boundary conditions not available for VLM.');
                    end
                    if (SIMXY_VLM > 0)
                        % get ground height and store it in its variable
                        SIMXY_VLM = abs(num_field_parser(tline, 9));
                    end
                    
                case keyword{24} % AERO card (unsteady DLM)
                    
                    if naero
                        fclose(fp);error('Multiple AERO cards given.');
                    end
                    naero = naero + 1;
                    SREF_DLM =  num_field_parser(tline, 2);
                    PARAM.VMAX = num_field_parser(tline, 3);
                    CREF_DLM = num_field_parser(tline, 4);
                    PARAM.RHO_VG = num_field_parser(tline, 5);
                    if (PARAM.RHO_VG == 0)
                        PARAM.RHO_VG = 1.225;
                    end
                    SIMXZ_DLM = int8(num_field_parser(tline, 6));
                    SIMXY_DLM = int8(num_field_parser(tline, 7));
                    if (SIMXY_DLM ~= 0)
                        fclose(fp);error('Ground effect for DLM not available yet.');
                    end
                    n = int8(num_field_parser(tline, 8));
                    if (n >0)
                        PARAM.DLM_ORDER = n;
                    end
                    n = num_field_parser(tline, 9);
                    if (n >0)
                        PARAM.NVSTEP = n;
                    end
                    BREF_DLM =  num_field_parser(tline, 10);
                    
                    
                case keyword{25} % SPLINE3 card for MLS interpolation
                    
                    ninterp = ninterp +1;
                    CAERO.Interp.Param(ninterp, :) = zeros(1, 5);
                    CAERO.Interp.Type(ninterp) = uint8(1);
                    
                    CAERO.Interp.ID(ninterp) = int32(num_field_parser(tline, 2));
                    CAERO.Interp.Patch(ninterp) = int32(num_field_parser(tline, 3));
                    CAERO.Interp.Index(ninterp,1) = int32(num_field_parser(tline, 4));
                    if CAERO.Interp.Index(ninterp,1) < 1
                        
                        fclose(fp);error('Wrong index for first panel in interpolation set %d.', CAERO.Interp.ID(ninterp));
                        
                    end
                    CAERO.Interp.Index(ninterp,2) = int32(num_field_parser(tline, 5));
                    CAERO.Interp.Set(ninterp) = int32(num_field_parser(tline, 6));
                    
                    for i=1:3
                        
                        CAERO.Interp.Param(ninterp, i) = num_field_parser(tline, 6+i); % poly, weight, points: parameter for MLS interface
                        
                    end
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        for i=2:3
                            
                            CAERO.Interp.Param(ninterp, i+2) = num_field_parser(tline, i); %rmax, tcond: parameter for MLS interface
                            
                        end
                        
                    else
                        
                        fclose(fp);error('Unable to read element list for SPLINE3 card %d.', CAERO.Interp.ID(ninterp));
                        
                    end
                    
                case {keyword{26},keyword{77}} % SET1 card
                    
                    nset = nset +1;
                    
                    CAERO.Set.ID(nset) = int32(num_field_parser(tline, 2));
                    [tline, rem] = strtok(tline);
                    [tline, rem] = strtok(rem);
                    LINE = [];
                    LINE = [LINE, rem];
                    skip_line = false;
                    while ~skip_line
                        tline = fgetl(fp);
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                          LINE = [LINE, tline];
                        else
                            skip_line = true;
                        end
                    end
                    CAERO.Set.Node(nset).data = unique(read_set_file(LINE,'SET1'));
                    
                case keyword{27} % RBE0 card for aeropoints
                    
                    nrbe0 = nrbe0 +1;
                    ne = 0;
                    data = [];
                    
                    RBE0.ID(nrbe0) = int32(num_field_parser(tline, 2));
                    RBE0.Master(nrbe0) = int32(num_field_parser(tline, 3));
                    
                    for i=4:9
                        
                        ne = ne+1;
                        data(ne) = int32(num_field_parser(tline, i));
                        
                    end
                    
                    skip_line = false;
                    
                    while ~skip_line
                        
                        tline = fgetl(fp);
                        
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            
                            skip_line = false;
                            for i= 2:9
                                
                                ne = ne+1;
                                data(ne) = int32(num_field_parser(tline, i));
                                
                            end
                            
                        else
                            
                            skip_line = true;
                            
                        end
                        
                    end
                    
                    index = find(data);
                    RBE0.Node(nrbe0).data = unique(data(index));
                    
                case keyword{28} % SPLINE2 card for RBF interpolation
                    
                    ninterp = ninterp +1;
                    CAERO.Interp.Param(ninterp, :) = zeros(1, 5);
                    CAERO.Interp.Type(ninterp) = uint8(2);
                    
                    CAERO.Interp.ID(ninterp) = int32(num_field_parser(tline, 2));
                    CAERO.Interp.Patch(ninterp) = int32(num_field_parser(tline, 3));
                    CAERO.Interp.Index(ninterp,1) = int32(num_field_parser(tline, 4));
                    if CAERO.Interp.Index(ninterp,1) < 1
                        
                        fclose(fp);error('Wrong index for first panel in interpolation set %d.', CAERO.Interp.ID(ninterp));
                        
                    end
                    CAERO.Interp.Index(ninterp,2) = int32(num_field_parser(tline, 5));
                    CAERO.Interp.Set(ninterp) = int32(num_field_parser(tline, 6));
                    CAERO.Interp.Param(ninterp, 1) = uint8(1);
                    for i=2:4
                        
                        CAERO.Interp.Param(ninterp, i) = num_field_parser(tline, 5+i); % weight, rmax, tcond parameter for RBF interface
                        
                    end
                    CAERO.Interp.Param(ninterp, 5) = CAERO.Interp.Param(ninterp, 4);
                    CAERO.Interp.Param(ninterp, 4) = CAERO.Interp.Param(ninterp, 3);
                    CAERO.Interp.Param(ninterp, 3) = 0;
                    
                case keyword{29} % TRIM card for static aeroelasticity
                    
                    ntrim = ntrim +1;
                    
                    CAERO.Trim.ID(ntrim)  = int32(num_field_parser(tline, 2));
                    CAERO.Trim.Type(ntrim) = int8(num_field_parser(tline, 3));
                    CAERO.Trim.Mach(ntrim)  = num_field_parser(tline, 4);
                    CAERO.Trim.ALT(ntrim) = num_field_parser(tline, 5);
                    Param(ntrim).data(1) = {name_field_parser(tline, 6)};
                    Value(ntrim).data(1) = num_field_parser(tline, 7);
                    Param(ntrim).data(2) = {name_field_parser(tline, 8)};
                    Value(ntrim).data(2) = num_field_parser(tline, 9);
                    index = find(CAERO.Trim.Mach(1:ntrim-1) == CAERO.Trim.Mach(ntrim));                
                    if isempty(index)
                      mindex = mindex+1;
                      CAERO.Trim.MINDEX(ntrim) = mindex;
                    else
                      CAERO.Trim.MINDEX(ntrim) = CAERO.Trim.MINDEX(index(1));
                    end
                    skip_line = false;
                    n = 2;
                    
                    while ~skip_line
                        
                        tline = fgetl(fp);
                        
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            
                            skip_line = false;
                            
                            for k=2:2:8
                                
                                n = n+1;
                                Param(ntrim).data(n) =  {name_field_parser(tline, k)};
                                Value(ntrim).data(n) = num_field_parser(tline, k+1);
                                
                            end
                            
                        else
                            
                            skip_line = true;
                        end
                        
                    end
                    
                    i = 0;
                    
                    for k=1:n
                        
                        if length(cell2mat(Param(ntrim).data(k)))
                            
                            i = i+1;
                            TRIM_PARAM(ntrim).data(i) = Param(ntrim).data(k);
                            TRIM_VALUE(ntrim).data(i) = Value(ntrim).data(k);
                            
                        end
                        
                    end
                    
                    CAERO.Trim.NC(ntrim) = i;
                    
                case keyword{30} % TRIM= case control card
                    
                    cc_trim = cc_trim +1;
                    
                    [tline, rem] = strtok(tline);
                    [id, lab_rem] = strtok(rem, ',');
                    
                    CAERO.Trim.Label_Select{cc_trim} = strtrim(lab_rem(2:end));
                    CAERO.Trim.Select(cc_trim) = int32(str2num(id));
                    
                    % THRUST card
                case keyword{31}
                    
                    nflw = nflw +1;
                    
                    F_FLW.ID(nflw) = int32(num_field_parser(tline, 2));
                    F_FLW.Node(nflw) = int32(num_field_parser(tline, 3));
                    F_FLW.CID(nflw) = int32(num_field_parser(tline, 4));
                    F_FLW.Orient(nflw,:) = zeros(1,3);
                    F_FLW.Offset(nflw,:) = zeros(1,3);
                    F_FLW.Orient(nflw, 1) = num_field_parser(tline, 5);
                    F_FLW.Orient(nflw, 2) = num_field_parser(tline, 6);
                    F_FLW.Orient(nflw, 3) = num_field_parser(tline, 7);
                    if (norm(F_FLW.Orient(nflw,:)) == 0)
                        
                        fclose(fp);error('Follower force %d has null orientation vector.', F_FLW.ID(nflw));
                        
                    end
                    F_FLW.Mag(nflw) = norm(F_FLW.Orient(nflw,:));
                    F_FLW.Orient(nflw,:) = F_FLW.Orient(nflw,:) ./ F_FLW.Mag(nflw);
                    F_FLW.Offset(nflw, 1) = num_field_parser(tline, 8);
                    F_FLW.Offset(nflw, 2) = num_field_parser(tline, 9);
                    F_FLW.Offset(nflw, 3) = num_field_parser(tline, 10);
                    
                case keyword{32} % AELINK used to set control surface constraints
                    
                    nlink = nlink +1;
                    
                    CAERO.Trim.Link.ID(nlink) = int32(num_field_parser(tline, 2));
                    name_s = name_field_parser(tline, 3);
                    if length(name_s) == 0
                        fclose(fp);error('No variable name given in dependent AELINK card %d.', CAERO.Trim.Link.ID(nlink));
                    end
                    CAERO.Trim.Link.Slave(nlink) = {name_s};
                    name_m = name_field_parser(tline, 4);
                    if length(name_m) == 0
                        fclose(fp);error('No variable name given in independent AELINK card %d.', CAERO.Trim.Link.ID(nlink));
                    end
                    CAERO.Trim.Link.Master(nlink) = {name_m};
                    if strcmp(name_s, name_m)
                        fclose(fp);error('The same control surface is declared both as independent and dependent in AELINK card %d.',...
                            CAERO.Trim.Link.ID(nlink));
                    end
                    CAERO.Trim.Link.Coeff(nlink) = num_field_parser(tline, 5);
                    if length(name_field_parser(tline, 6))
                        fclose(fp);error('AELINK card provides only two parameters to be given.');
                    end
                    
                case keyword{33} % MKAERO1
                    
                    nmkaero = nmkaero +1;
                    if (nmkaero>1)
                      error('Multiple MKAERO1 card given.');
                    end
                    n = length(CAERO.state.Mach_qhh);
                    for i=1:8
                        
                        CAERO.state.Mach_qhh(n+i) = num_field_parser(tline, i+1);
                        
                    end
                    
                    tline = fgetl(fp);
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        n = length(CAERO.state.Kfreq);
                        for i=1:8
                            
                            CAERO.state.Kfreq(n+i) = num_field_parser(tline, i+1);
                            
                        end
                        
                    else
                        
                        fclose(fp);error('MKAERO1 missing reduced frequency list.');
                        
                    end
                    
                    tline = fgetl(fp);
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        n = length(CAERO.state.Kfreq);
                        for i=1:8
                            
                            CAERO.state.Kfreq(n+i) = num_field_parser(tline, i+1);
                            
                        end
                    end
                    
                case keyword{34} % EIGR
                    
                    neig = neig +1;
                    
                    if (neig >1)
                        fclose(fp);error('Multiple EIGR cards given.');
                    end
                    
                    PARAM.MINF = num_field_parser(tline, 4);
                    PARAM.MAXF = num_field_parser(tline, 5);
                    if (PARAM.MINF > PARAM.MAXF)
                        fclose(fp);error('Wrong frequency range given in EIGR card.');
                    end
                    PARAM.NROOTS = int32(abs(num_field_parser(tline, 7)));
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        mode_scale = string_field_parser(tline, 1);
                        if (length(mode_scale) >0)
                            PARAM.MSCALE = mode_scale;
                        end
                        PARAM.MG =  int32(num_field_parser(tline, 3));
                        PARAM.MC =  int32(num_field_parser(tline, 4));
                        if ((PARAM.MC<0) || (PARAM.MC>6))
                            fclose(fp);error('Wrong component number in EIGR card.');
                        end
                        
                    else
                        
                        skip_line = true;
                        
                    end
                    
                case keyword{35} % GRAV
                    
                    ngrav = ngrav + 1;
                    GRAV.ID(ngrav) = int32(num_field_parser(tline, 2));
                    GRAV.CID(ngrav) = int32(num_field_parser(tline, 3));
                    GRAV.Mag(ngrav) = num_field_parser(tline, 4);
                    GRAV.Orient(1, ngrav) = num_field_parser(tline, 5);
                    GRAV.Orient(2, ngrav) = num_field_parser(tline, 6);
                    GRAV.Orient(3, ngrav) = num_field_parser(tline, 7);
                    
                case keyword{36} % MSELECT
                    
                    if (isempty(PARAM.MSELECT))
                        LINE = [];
                        [tline, rem] = strtok(tline);
                        LINE = [LINE, rem];
                        skip_line = false;
                        while ~skip_line
                            tline = fgetl(fp);
                            if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                              LINE = [LINE, tline];
                            else
                                skip_line = true;
                            end
                        end
                        PARAM.MSELECT = unique(read_set_file(LINE,'MSELECT'));
                        index = find(PARAM.MSELECT);
                        PARAM.MSELECT = unique(PARAM.MSELECT(index));
                    else
                        fclose(fp);error('MSELECT card already given.');
                    end
                    
                case keyword{37} % SUPORT
                    
                    nsuport = nsuport +1;
                    if (nsuport >1)
                        fclose(fp);error('Multiple SUPORT cards given.');
                    end
                    
                    PARAM.SUPORT = [];
                    ne = 0;
                    data = [];
                    
                    for i=3:9
                        
                        ne = ne+1;
                        data(ne) = int32(num_field_parser(tline, i));
                        
                    end
                    
                    skip_line = false;
                    
                    while ~skip_line
                        
                        tline = fgetl(fp);
                        
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            
                            skip_line = false;
                            for i= 2:9
                                
                                ne = ne+1;
                                data(ne) = int32(num_field_parser(tline, i));
                                
                            end
                            
                        else
                            
                            skip_line = true;
                            
                        end
                        
                    end
                    %
                    index = find(data);
                    data = data(index);
                    ne = length(data);
                    if (mod(ne,2))
                        fclose(fp);error('Odd entries given in SUPORT card.');
                    end
                    %
                    PARAM.SUPORT(:,1) = data(1:2:ne); PARAM.SUPORT(:,2) = data(2:2:ne);
                    %
                    % PBARSM1
                case keyword{38} % PBAR card for single cell semimonocoque
                    
                    npbarsm = npbarsm +1;
                    npbar = npbar +1;
                    PBAR.SI(npbar) = npbarsm;
                    PBAR.ID(npbar) = int32(num_field_parser(tline, 2));
                    PBAR.Mat(npbar) = int32(num_field_parser(tline, 3));
                    PBAR.Type(npbar) = 10;
                    PBAR.Section(npbarsm).data = [];
                    for r = 4:7 % Acorner, tskin, chord (=z length), height(=y length)
                        PBAR.Section(npbarsm).data(r-3) = abs(num_field_parser(tline, r));
                    end
                    PBAR.RhoNS(npbar) = num_field_parser(tline, 8);
                    %
                    schord = PBAR.Section(npbarsm).data(3);
                    sheight = PBAR.Section(npbarsm).data(4);
                    %
                    PBAR.A(npbar) = 0.0;
                    PBAR.I(npbar,1) = 0.0;
                    PBAR.I(npbar,2) = 0.0;
                    PBAR.I(npbar,3) = 0.0;
                    PBAR.J(npbar) = 0.0;
                    PBAR.Kshear(npbar,1:2) = zeros(1,2);
                    PBAR.Str_point(:,:,npbar) = zeros(4,2);
                    % try to read continuation
                    skip_line = false;
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        skip_line = false;
                        % store stress recovery points
                        r = 2;
                        for n=1:4
                            
                            PBAR.Str_point(n,1,npbar) = num_field_parser(tline, r);
                            PBAR.Str_point(n,2,npbar) = num_field_parser(tline, r+1);
                            r = r+2;
                        end
                    else
                        skip_line =true;
                        %     automatically add stress recovery points
                        PBAR.Str_point(1,1,npbar) = 0.0; PBAR.Str_point(1,2,npbar) = schord/2;
                        PBAR.Str_point(3,1,npbar) = 0.0; PBAR.Str_point(3,2,npbar) = -schord/2;
                        %
                        PBAR.Str_point(2,1,npbar) = sheight/2;  PBAR.Str_point(2,2,npbar) = 0.0;
                        PBAR.Str_point(4,1,npbar) = -sheight/2; PBAR.Str_point(4,2,npbar) = 0.0;
                    end
                    
                    % PBARGMW
                case keyword{39} % PBAR card for multi-web box beam
                    
                    npbarsm = npbarsm +1;
                    npbar = npbar +1;
                    PBAR.SI(npbar) = npbarsm;
                    PBAR.ID(npbar) = int32(num_field_parser(tline, 2));
                    PBAR.Mat(npbar) = int32(num_field_parser(tline, 3));
                    PBAR.Type(npbar) = 3;
                    PBAR.Section(npbarsm).data = [];
                    for r=4:8 % web thick, cover thick, web spacing, chord (=z length), height(=y length)
                        PBAR.Section(npbarsm).data(r-3) = num_field_parser(tline, r);
                    end
                    %
                    schord = PBAR.Section(npbarsm).data(4);
                    sheight = PBAR.Section(npbarsm).data(5);
                    %
                    tline = fgetl(fp);
                    for r=2:7 % Kweb, Kcover, ExpW, EPSW, ExpC, EPSC,
                        PBAR.Section(npbarsm).data(r+4) = num_field_parser(tline, r);
                    end
                    PBAR.RhoNS(npbar) = num_field_parser(tline, 8);
                    
                    PBAR.A(npbar) = 0.0;
                    PBAR.I(npbar,1) = 0.0;
                    PBAR.I(npbar,2) = 0.0;
                    PBAR.I(npbar,3) = 0.0;
                    PBAR.J(npbar) = 0.0;
                    PBAR.Kshear(npbar,1:2) = zeros(1,2);
                    PBAR.Str_point(:,:,npbar) = zeros(4,2);
                    
                    % try to read continuation
                    skip_line = false;
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        skip_line = false;
                        % store stress recovery points
                        r = 2;
                        for n=1:4
                            
                            PBAR.Str_point(n,1,npbar) = num_field_parser(tline, r);
                            PBAR.Str_point(n,2,npbar) = num_field_parser(tline, r+1);
                            r = r+2;
                        end
                    else
                        skip_line =true;
                        %     automatically add stress recovery points
                        PBAR.Str_point(1,1,npbar) = 0.0; PBAR.Str_point(1,2,npbar) = schord/2;
                        PBAR.Str_point(3,1,npbar) = 0.0; PBAR.Str_point(3,2,npbar) = -schord/2;
                        %
                        PBAR.Str_point(2,1,npbar) = sheight/2;  PBAR.Str_point(2,2,npbar) = 0.0;
                        PBAR.Str_point(4,1,npbar) = -sheight/2; PBAR.Str_point(4,2,npbar) = 0.0;
                    end
                    
                case keyword{40} % FMODES specify modes to be followed in flutter tracking
                    
                    if (isempty(PARAM.FMODES))
                        LINE = [];
                        [tline, rem] = strtok(tline);
                        LINE = [LINE, rem];
                        skip_line = false;
                        while ~skip_line
                            tline = fgetl(fp);
                            if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                              LINE = [LINE, tline];
                            else
                                skip_line = true;
                            end
                        end
                        PARAM.FMODES = unique(read_set_file(LINE,'FMODES'));
                        index = find(PARAM.FMODES);
                        PARAM.FMODES = unique(PARAM.FMODES(index));
                    else
                        fclose(fp);error('FMODES card already given.');
                    end
                    
                case keyword{47} % UMODES specify modes to be used in dynamic solution
                    
                    if (isempty(PARAM.UMODES))
                        LINE = [];
                        [tline, rem] = strtok(tline);
                        LINE = [LINE, rem];
                        skip_line = false;
                        while ~skip_line
                            tline = fgetl(fp);
                            if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                              LINE = [LINE, tline];
                            else
                                skip_line = true;
                            end
                        end
                        PARAM.UMODES = unique(read_set_file(LINE,'UMODES'));
                        index = find(PARAM.UMODES);
                        PARAM.UMODES = unique(PARAM.UMODES(index));
                    else
                        fclose(fp);error('UMODES card already given.');
                    end
                    
                    % PBARWB1
                case keyword{41} % PBAR card for symmetric wing box definition
                    
                    npbarsm = npbarsm +1;
                    npbar = npbar +1;
                    PBAR.SI(npbar) = npbarsm;
                    PBAR.ID(npbar) = int32(num_field_parser(tline, 2));
                    PBAR.Mat(npbar) = int32(num_field_parser(tline, 3));
                    PBAR.Type(npbar) = 11;
                    PBAR.Section(npbarsm).data = [];
                    for r = 4:8 % Atot, tweb, tskin, chord (=z length), height(=y length)
                        PBAR.Section(npbarsm).data(r-3) = abs(num_field_parser(tline, r));
                    end
                    PBAR.RhoNS(npbar) = num_field_parser(tline, 9);
                    schord = PBAR.Section(npbarsm).data(4);
                    sheight = PBAR.Section(npbarsm).data(5);
                    %
                    PBAR.A(npbar) = 0.0;
                    PBAR.I(npbar,1) = 0.0;
                    PBAR.I(npbar,2) = 0.0;
                    PBAR.I(npbar,3) = 0.0;
                    PBAR.J(npbar) = 0.0;
                    PBAR.Kshear(npbar,1:2) = zeros(1,2);
                    PBAR.Str_point(:,:,npbar) = zeros(4,2);
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        skip_line = false;
                        
                    else
                        
                        fclose(fp);error('Missing continuation card in PBARWB1 %d property definition.', PBAR.ID(npbar));
                        
                    end
                    
                    Stype = string_field_parser(tline, 2);
                    
                    switch(Stype)
                        
                        case {'I', 'i'}
                            PBAR.Section(npbarsm).data(6) = 1;
                            
                        case {'Z', 'z'}
                            PBAR.Section(npbarsm).data(6) = 2;
                            
                        case {'C', 'c'}
                            PBAR.Section(npbarsm).data(6) = 3;
                            
                        case {'L', 'l'}
                            PBAR.Section(npbarsm).data(6) = 4;
                            
                        case {'T', 't'}
                            PBAR.Section(npbarsm).data(6) = 5;
                            
                        otherwise
                            fclose(fp);error('Unknown stringer type in PBARWB1 %d.', PBAR.ID(npbar));
                    end
                    
                    for r = 3:6 % chordwise stringers, ribs pitch, horiz flange/thickness ratio, vert flange/thickness ratio
                        PBAR.Section(npbarsm).data(4+r) = abs(num_field_parser(tline, r));
                    end
                    
                    % try to read continuation
                    skip_line = false;
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        skip_line = false;
                        % store stress recovery points
                        r = 2;
                        for n=1:4
                            
                            PBAR.Str_point(n,1,npbar) = num_field_parser(tline, r);
                            PBAR.Str_point(n,2,npbar) = num_field_parser(tline, r+1);
                            r = r+2;
                        end
                    else
                        skip_line =true;
                        %     automatically add stress recovery points
                        PBAR.Str_point(1,1,npbar) = 0.0; PBAR.Str_point(1,2,npbar) = schord/2;
                        PBAR.Str_point(3,1,npbar) = 0.0; PBAR.Str_point(3,2,npbar) = -schord/2;
                        %
                        PBAR.Str_point(2,1,npbar) = sheight/2;  PBAR.Str_point(2,2,npbar) = 0.0;
                        PBAR.Str_point(4,1,npbar) = -sheight/2; PBAR.Str_point(4,2,npbar) = 0.0;
                    end
                    
                    % PBARGFU
                case keyword{42} % PBAR card fuselage MDO from GUESS solver
                    
                    npbarsm = npbarsm +1;
                    npbar = npbar +1;
                    PBAR.SI(npbar) = npbarsm;
                    PBAR.ID(npbar) = int32(num_field_parser(tline, 2));
                    PBAR.Mat(npbar) = int32(num_field_parser(tline, 3));
                    PBAR.Type(npbar) = 1;
                    PBAR.Section(npbarsm).data = [];
                    for r = 4:7 % Skin_t, R, BM, BEXP
                        PBAR.Section(npbarsm).data(r-3) = abs(num_field_parser(tline, r));
                    end
                    PBAR.RhoNS(npbar) = num_field_parser(tline, 8);
                    %
                    tline = fgetl(fp);
                    for r = 2:3 % KMG, KP
                        PBAR.Section(npbarsm).data(3+r) = abs(num_field_parser(tline, r));
                    end
                    %
                    shradius = PBAR.Section(npbarsm).data(2);
                    PBAR.A(npbar) = 0.0;
                    PBAR.I(npbar,1) = 0.0;
                    PBAR.I(npbar,2) = 0.0;
                    PBAR.I(npbar,3) = 0.0;
                    PBAR.J(npbar) = 0.0;
                    PBAR.Kshear(npbar,1:2) = zeros(1,2);
                    PBAR.Str_point(:,:,npbar) = zeros(4,2);
                    % try to read continuation
                    skip_line = false;
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        skip_line = false;
                        % store stress recovery points
                        r = 2;
                        for n=1:4
                            
                            PBAR.Str_point(n,1,npbar) = num_field_parser(tline, r);
                            PBAR.Str_point(n,2,npbar) = num_field_parser(tline, r+1);
                            r = r+2;
                        end
                    else
                        skip_line =true;
                        PBAR.Str_point(1,1,npbar) = 0.0; PBAR.Str_point(1,2,npbar) = shradius;
                        PBAR.Str_point(3,1,npbar) = 0.0; PBAR.Str_point(3,2,npbar) = -shradius;
                        %
                        PBAR.Str_point(2,1,npbar) = shradius;  PBAR.Str_point(2,2,npbar) = 0.0;
                        PBAR.Str_point(4,1,npbar) = -shradius; PBAR.Str_point(4,2,npbar) = 0.0;
                    end
                    
                    % PBARGFF
                case keyword{43} % PBAR card fuselage MDO from GUESS solver
                    
                    npbarsm = npbarsm +1;
                    npbar = npbar +1;
                    PBAR.SI(npbar) = npbarsm;
                    PBAR.ID(npbar) = int32(num_field_parser(tline, 2));
                    PBAR.Mat(npbar) = int32(num_field_parser(tline, 3));
                    PBAR.Type(npbar) = 2;
                    PBAR.Section(npbarsm).data = [];
                    for r = 4:7 % Skin_t, Area_Frame, F_spacing, R
                        PBAR.Section(npbarsm).data(r-3) = abs(num_field_parser(tline, r));
                    end
                    PBAR.RhoNS(npbar) = num_field_parser(tline, 8);
                    %
                    tline = fgetl(fp);
                    for r = 2:9 % BM, BEXP (buckling exponent), KMG, KP, CF (Shanley's constant), CKF, FDENS, FE
                        PBAR.Section(npbarsm).data(3+r) = abs(num_field_parser(tline, r));
                    end
                    %
                    shradius = PBAR.Section(npbarsm).data(4);
                    PBAR.A(npbar) = 0.0;
                    PBAR.I(npbar,1) = 0.0;
                    PBAR.I(npbar,2) = 0.0;
                    PBAR.I(npbar,3) = 0.0;
                    PBAR.J(npbar) = 0.0;
                    PBAR.Kshear(npbar,1:2) = zeros(1,2);
                    PBAR.Str_point(:,:,npbar) = zeros(4,2);
                    % try to read continuation
                    skip_line = false;
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        skip_line = false;
                        % store stress recovery points
                        r = 2;
                        for n=1:4
                            
                            PBAR.Str_point(n,1,npbar) = num_field_parser(tline, r);
                            PBAR.Str_point(n,2,npbar) = num_field_parser(tline, r+1);
                            r = r+2;
                        end
                    else
                        skip_line =true;
                        PBAR.Str_point(1,1,npbar) = 0.0; PBAR.Str_point(1,2,npbar) = shradius;
                        PBAR.Str_point(3,1,npbar) = 0.0; PBAR.Str_point(3,2,npbar) = -shradius;
                        %
                        PBAR.Str_point(2,1,npbar) = shradius;  PBAR.Str_point(2,2,npbar) = 0.0;
                        PBAR.Str_point(4,1,npbar) = -shradius; PBAR.Str_point(4,2,npbar) = 0.0;
                    end
                    
                    % INCLUDE
                case keyword{44} % INCLUDE card for secondary files
                    
                    ninclude = ninclude +1;
                    %       name_assembly = '';
                    %       for k=2:20
                    %         name_assembly = [name_assembly, include_field_parser(tline, k)];
                    %       end
                    name_assembly = tline(9:end);
                    PARAM.INCLUDE{ninclude} = name_assembly;

                    % CELAS
                case keyword{45} 
                    DOF1 = string_field_parser(tline, 5);
                    DOF2 = string_field_parser(tline, 7);
                    Cnode1 = num_field_parser(tline, 4);
                    Cnode2 = num_field_parser(tline, 6);
                    
                    if isempty(DOF1) && isempty(DOF2)
                        fclose(fp);error('\nUnable to find dof in celas %d.', num_field_parser(tline, 2));
                    elseif Cnode1==0 && Cnode2==0
                        fclose(fp);error('\nUnable to find node in celas %d.', num_field_parser(tline, 2));
                    elseif Cnode1 ==0 && ~isempty(DOF1)
                        fclose(fp);error('\nUnable to find node 1 in celas %d.', num_field_parser(tline, 2));
                    elseif Cnode2 ==0 && ~isempty(DOF2)
                        fclose(fp);error('\nUnable to find node 2 in celas %d.', num_field_parser(tline, 2));
                    elseif Cnode1 ~=0 && isempty(DOF1)
                        fclose(fp);error('\nUnable to find DOF of node 1 in celas %d.', num_field_parser(tline, 2));
                    elseif Cnode2 ~=0 && isempty(DOF2)
                        fclose(fp);error('\nUnable to find DOF of node 2 in celas %d.', num_field_parser(tline, 2));
                    else
                        ncelas = ncelas +1;
                        CELAS.ID(ncelas) = num_field_parser(tline, 2);
                        CELAS.STiff(ncelas) = num_field_parser(tline, 3);
                        CELAS.Node(ncelas,1) = Cnode1;
                        CELAS.Node(ncelas,2) = Cnode2;
                        CELAS.DOF(ncelas,1).data = DOF1;
                        CELAS.DOF(ncelas,2).data = DOF2;
                        
                    end
                  % RBE2
                case keyword{46}
                    DOF = string_field_parser(tline, 4);
                    if isempty(DOF) && ischar(DOF)
                        fclose(fp);error('\nUnable to find dof in RBE2 %d.', num_field_parser(tline, 2));
                    else
                        nrbe2 = nrbe2 +1;
                        RBE2.ID(nrbe2) = num_field_parser(tline, 2);
                        RBE2.IDM(nrbe2) = num_field_parser(tline, 3);
                        RBE2.GDL(nrbe2).data = DOF;
                        
                        ne = 0;
                        data = [];
                        for i=5:9
                            
                            ne = ne+1;
                            data(ne) = int32(num_field_parser(tline, i));
                            
                        end
                        
                        skip_line = false;
                        
                        while ~skip_line
                            
                            tline = fgetl(fp);
                            
                            if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                                
                                skip_line = false;
                                for i= 2:9
                                    
                                    ne = ne+1;
                                    data(ne) = int32(num_field_parser(tline, i));
                                    
                                end
                                
                            else
                                
                                skip_line = true;
                                
                            end
                            
                        end
                        
                        index = find(data);
                        RBE2.IDS(nrbe2).data = unique(data(index));
                        
                    end
  
                  % GUST                    
                case keyword{48} 
                    
                    ngust  = ngust +1;
                    
                    GUST.ID(ngust) = int32(num_field_parser(tline, 2));
                    if ~GUST.ID(ngust)
                        
                        fclose(fp);error('Given null Gust ID.');
                        
                    end
                    GUST.Amp(ngust) = num_field_parser(tline, 3);
                    GUST.Tmax(ngust) = num_field_parser(tline, 4);
                    GUST.X0(ngust) = num_field_parser(tline, 5);
                    GUST.DIR(ngust) = num_field_parser(tline, 6);
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        GUST.funs{ngust} = tline(9:end);
                    else
                        
                        fclose(fp);error('Given no Gust spatial distribution.');
                        
                    end
                    
                    tline = fgetl(fp);
                    
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        
                        GUST.fun{ngust} = tline(9:end);
                    else
                        
                        fclose(fp);error('Given no Gust function.');
                        
                    end

                  % SET                    
                case keyword{49} 
                    [token, remain] = strtok(tline);
                    nSET = nSET+1;
                    indeq = find(remain == '=');
                    if isempty(indeq)
                        fclose(fp);error('error SET card')
                    else
                        idSET = str2double(remain(indeq+1:end))
                        if  isnan(idSET)
                            fclose(fp);error('error in SET card, no ID')
                        elseif ~isempty(find(idSET == SET.ID,1))
                            fclose(fp);error('error in SET card, duplicated ID')
                        else
                            SET.ID(nSET) = idSET;
                            remain = remain(indeq+1:end);
                            if isempty(remain)
                                fclose(fp);error(['No value given in SET: ',num2str(SET.ID(nSET))])
                            else
                                indTHRU = strfind(remain,'THRU');
                                if isempty(indTHRU)
                                    SET.Val(nSET).data = str2num(remain);
                                else
                                    indEXCEPT = strfind(remain,'EXCEPT');
                                    if isempty(indEXCEPT)
                                        
                                        SET.Val(nSET).data = str2num(remain(1:indTHRU(1)-1));
                                        for iTH = 1 : length(indTHRU)-1
                                            dataTHRU = str2num(remain(indTHRU(iTH)+4:indTHRU(iTH+1)-1));
                                            SET.Val(nSET).data = [SET.Val(nSET).data,SET.Val(nSET).data(end)+1:dataTHRU(1),dataTHRU(2:end)];
                                        end
                                        dataTHRU = str2num(remain(indTHRU(end)+4:end));
                                        SET.Val(nSET).data = unique([SET.Val(nSET).data,SET.Val(nSET).data(end)+1:dataTHRU(1),dataTHRU(2:end)]);
                                    else
                                        if length(indEXCEPT) >  length(indTHRU)
                                            fclose(fp);error(['Too many EXCEPT: ',num2str(SET.ID(nSET))])
                                        else
                                            SET.Val(nSET).data = str2num(remain(1:indTHRU(1)-1));
                                            for iTH = 1 : length(indTHRU)-1
                                                if isempty(find(indEXCEPT>indTHRU(iTH) & indEXCEPT<indTHRU(iTH+1),1))
                                                    dataTHRU = str2num(remain(indTHRU(iTH)+4:indTHRU(iTH+1)-1));
                                                    SET.Val(nSET).data = [SET.Val(nSET).data,SET.Val(nSET).data(end)+1:dataTHRU(1),dataTHRU(2:end)];
                                                else
                                                    indEXCEPTloc = indEXCEPT(indEXCEPT>indTHRU(iTH) & indEXCEPT<indTHRU(iTH+1));
                                                    dataEXCEPT = str2num(remain(indEXCEPTloc+6:indTHRU(iTH+1)-1));
                                                    dataTHRU = str2double(remain(indTHRU(iTH)+4:indEXCEPTloc-1));
                                                    SET.Val(nSET).data = [SET.Val(nSET).data,setdiff(SET.Val(nSET).data(end)+1:dataTHRU(1),dataEXCEPT(dataEXCEPT<=dataTHRU(1))),dataEXCEPT(dataEXCEPT>dataTHRU(1))];
                                                end
                                            end
                                            if isempty(find(indEXCEPT>indTHRU(end),1))
                                                dataTHRU = str2num(remain(indTHRU(end)+4:end));
                                                SET.Val(nSET).data = unique([SET.Val(nSET).data,SET.Val(nSET).data(end)+1:dataTHRU(1),dataTHRU(2:end)]);
                                            else
                                                indEXCEPTloc = indEXCEPT(indEXCEPT>indTHRU(end) );
                                                dataEXCEPT = str2num(remain(indEXCEPTloc+6:end));
                                                dataTHRU = str2double(remain(indTHRU(end)+4:indEXCEPTloc-1));
                                                SET.Val(nSET).data = unique([SET.Val(nSET).data,setdiff(SET.Val(nSET).data(end)+1:dataTHRU(1),dataEXCEPT(dataEXCEPT<=dataTHRU(1))),dataEXCEPT(dataEXCEPT>dataTHRU(1))]);
                                            end
                                            
                                        end
                                    end
                                end
                            end
                        end
                        
                    end

                  % ACCELERATION                    
                case keyword{50}
                    [token, remain] = strtok(tline);
                    ACCELERATION = str2double(remain);
                    if isnan(ACCELERATION)
                        indACC = strfind(remain,'ALL');
                        if isempty(indACC)
                            fclose(fp);error('error in ACCELERATION card')
                        else
                            PARAM.ACCELERATION = 'ALL';
                        end
                    else
                        PARAM.ACCELERATION = ACCELERATION;
                    end

                  % DISP
                case keyword{51}
                    [token, remain] = strtok(tline);
                    DISP = str2double(remain);
                    if isnan(DISP)
                        indDISP = strfind(remain,'ALL');
                        if isempty(indDISP)
                            fclose(fp);error('error in DISP card')
                        else
                            PARAM.DISP = 'ALL';
                        end
                    else
                        PARAM.DISP = DISP;
                    end
                     
                  % VELOCITY
                case keyword{52}
                    [token, remain] = strtok(tline);
                    VELOCITY = str2double(remain);
                    if isnan(VELOCITY)
                        indVELOCITY = strfind(remain,'ALL');
                        if isempty(indVELOCITY)
                            fclose(fp);error('error in VELOCITY card')
                        else
                            PARAM.VELOCITY = 'ALL';
                        end
                    else
                        PARAM.VELOCITY = VELOCITY;
                    end

                  % IFORCE
                case keyword{53}
                    [token, remain] = strtok(tline);
                    IFORCE = str2double(remain);
                    if isnan(IFORCE)
                        indIFORCE = strfind(remain,'ALL');
                        if isempty(indIFORCE)
                            fclose(fp);error('error in IFORCE card')
                        else
                            PARAM.IFORCE = 'ALL';
                        end
                    else
                        PARAM.IFORCE = IFORCE;
                    end

                  % AEROFORCE
                case keyword{54}
                    [token, remain] = strtok(tline);
                    AEROFORCE = str2double(remain);
                    if isnan(AEROFORCE)
                        indAEROFORCE = strfind(remain,'ALL');
                        if isempty(indAEROFORCE)
                            fclose(fp);error('error in AEROFORCE card')
                        else
                            PARAM.AEROFORCE = 'ALL';
                        end
                    else
                        PARAM.AEROFORCE = AEROFORCE;
                    end
                    
                  % HINGEFORCE
                case keyword{55}
                    [token, remain] = strtok(tline);
                    HINGEFORCE = str2double(remain);
                    if isnan(HINGEFORCE)
                        indHINGEFORCE = strfind(remain,'ALL');
                        if isempty(indHINGEFORCE)
                            fclose(fp);error('error in HINGEFORCE card')
                        else
                            PARAM.HINGEFORCE = 'ALL';
                        end
                    else
                        PARAM.HINGEFORCE = HINGEFORCE;
                    end

                  % SURFDEF
                case keyword{56} 
                    nsurfdef  = nsurfdef +1;
                    SURFDEF.ID(nsurfdef) = int32(num_field_parser(tline, 2));
                    if ~SURFDEF.ID(nsurfdef)
                        fclose(fp); error('Null SURFDEF ID given.');
                    end
                    SURFDEF.Label{nsurfdef} = string_field_parser(tline,3);
                    SURFDEF.Amp(nsurfdef) = num_field_parser(tline, 4);
                    SURFDEF.Tmax(nsurfdef) = num_field_parser(tline, 5);
                    SURFDEF.X0(nsurfdef) = num_field_parser(tline, 6);
                    if (SURFDEF.X0(nsurfdef)<0)
                      error('Negative delay given in SURFDEF card');
                    end

                    % check format
                    if SURFDEF.Tmax(nsurfdef) == 0
                      SURFDEF.Npiece(nsurfdef) = SURFDEF.X0(nsurfdef);
                      for k=1:SURFDEF.Npiece(nsurfdef)
                        tline = fgetl(fp);
                        if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                            SURFDEF.Ti(nsurfdef).data(k,1) =  num_field_parser(tline,2);
                            SURFDEF.Ti(nsurfdef).data(k,2) =  num_field_parser(tline,3);
                            SURFDEF.fun{nsurfdef,k} = tline(25:end);
                        else
                            fclose(fp);error('No SURFDEF time function given.');
                        end
                      end
%
                      SURFDEF.X0(nsurfdef) = SURFDEF.Ti(nsurfdef).data(1,1);
                      Ttot = SURFDEF.Ti(nsurfdef).data(1,2);
                      for k=2:SURFDEF.Npiece(nsurfdef)
                        addp = SURFDEF.Ti(nsurfdef).data(k,1) - (SURFDEF.Ti(nsurfdef).data(k-1,1) + SURFDEF.Ti(nsurfdef).data(k-1,2));
                        if addp < 0
                          error(['Piecewise functions %d - %d overlap.', num2str(k-1), num2str(k)]);
                        else
                          Ttot = Ttot + SURFDEF.Ti(nsurfdef).data(k,1) - (SURFDEF.Ti(nsurfdef).data(k,1) + SURFDEF.Ti(nsurfdef).data(k,2));
                          if addp >0
                            fprintf(fid, '\n### Warning: piecewise functions %d-%d for AESURF %d do not match in time. Null values will appear in between.', k-1, k, SURFDEF.ID(nsurfdef));
                          end
                        end
                      end
                      SURFDEF.Tmax(nsurfdef) = Ttot;
%
                    else
                      SURFDEF.Npiece(nsurfdef) = 1;
                      tline = fgetl(fp);
                      if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                          SURFDEF.fun{nsurfdef} = tline(9:end);
                      else
                          fclose(fp);error('No SURFDEF time function given.');
                      end
                      SURFDEF.Ti(nsurfdef).data(1,1) = SURFDEF.X0(nsurfdef);
                      SURFDEF.Ti(nsurfdef).data(1,2) = SURFDEF.Tmax(nsurfdef);
                    end


                    % GUST
                case keyword{57}
                    [tline, rem] = strtok(tline);
                    PARAM.GUST = int32(str2num(rem));
                    if (PARAM.GUST == 0)
                        fclose(fp);error('Gust set must be different from zero.');
                    end
                    ngustl = ngustl +1;
                    if ngustl > 1
                        fclose(fp);error('Multiple GUST loads required. Please check.');
                    end

                  % SURFDEF=
                case keyword{58}
                    [tline, rem] = strtok(tline);
                    
                    PARAM.SURFDEF = int32(str2num(rem));
                    if (PARAM.SURFDEF == 0)
                        fclose(fp);error('SURFDEF set must be different from zero.');
                    end
                    nsurfdefl = nsurfdefl +1;
                    if nsurfdefl > 1
                        fclose(fp);error('Multiple SURFDEF loads required. Please check.');
                    end

                  % IFORCEBE
                case keyword{59}
                    [token, remain] = strtok(tline);
                    IFORCEBE = str2double(remain);
                    if isnan(IFORCEBE)
                        indIFORCEBE = strfind(remain,'ALL');
                        if isempty(indIFORCEBE)
                            fclose(fp); error('error in IFORCEBE card')
                        else
                            PARAM.IFORCEBE = 'ALL';
                        end
                    else
                        PARAM.IFORCEBE = IFORCEBE;
                    end

                  % SPLINE1
                case keyword{60} 
                    ninterp = ninterp +1;
                    CAERO.Interp.Param(ninterp, :) = zeros(1, 5);
                    CAERO.Interp.Type(ninterp) = uint8(1);
                    
                    CAERO.Interp.ID(ninterp) = int32(num_field_parser(tline, 2));
                    CAERO.Interp.Patch(ninterp) = int32(num_field_parser(tline, 3));
                    CAERO.Interp.Index(ninterp,1) = int32(num_field_parser(tline, 4));
                    if CAERO.Interp.Index(ninterp,1) < 1
                        
                        fclose(fp);error('Wrong index for first panel in interpolation set %d.', CAERO.Interp.ID(ninterp));
                        
                    end
                    CAERO.Interp.Index(ninterp,2) = int32(num_field_parser(tline, 5));
                    CAERO.Interp.Set(ninterp) = int32(num_field_parser(tline, 6));
                    CAERO.Interp.Param(ninterp, 1) = int32(num_field_parser(tline, 7));
                    CAERO.Interp.Param(ninterp, 2) = int32(num_field_parser(tline, 8));
                    CAERO.Interp.Param(ninterp, 3) = int32(num_field_parser(tline, 9));

                case keyword{61} % ITRIM= case control card
                    
                    i_trim = i_trim +1;
                    [tline, rem] = strtok(tline);
                    [id, lab_rem] = strtok(rem, ',');
                    CAERO.Trim.Man_index(i_trim) = int32(str2num(id));

                  % DLOAD dynamic external load
                case keyword{62} 
                    ndload  = ndload +1;
                    DEXTLOAD.ID(ndload) = int32(num_field_parser(tline, 2));
                    if ~DEXTLOAD.ID(ndload)
                        fclose(fp); error('Null DEXTLOAD ID given.');
                    end
                    DEXTLOAD.Node(ndload) = num_field_parser(tline, 3);
                    DEXTLOAD.NDOF(ndload) = num_field_parser(tline, 4);
                    if (DEXTLOAD.NDOF(ndload)>6 || DEXTLOAD.NDOF(ndload)<1)
                      error('Nodal DOF in DEXTLOAD card ',num2str(DEXTLOAD.ID(ndload)), ' must be between 1 and 6.');
                    end
                    DEXTLOAD.Amp(ndload) = num_field_parser(tline, 5);
                    DEXTLOAD.Tmax(ndload) = num_field_parser(tline, 6);
                    DEXTLOAD.X0(ndload) = num_field_parser(tline, 7);
                    if (DEXTLOAD.X0(ndload)<0)
                      error('Negative delay given in DEXTLOAD card.');
                    end
                    tline = fgetl(fp);
                    if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        DEXTLOAD.fun{ndload} = tline(9:end);
                    else
                        fclose(fp);error('No DEXTLOAD time function given.');
                    end

                case keyword{63} % SDAMPING= case control card
                    
                    i_damp = i_damp +1;
                    [tline, rem] = strtok(tline);
                    PARAM.SDAMP = int32(str2num(rem));
                    if (PARAM.SDAMP == 0)
                        fclose(fp);error('SDAMPING table ID set must be different from zero.');
                    end
                    if i_damp > 1
                        fclose(fp);error('Multiple SDAMPING cards. Please check.');
                    end

                case keyword{64} % TABDMP1

                    ndamp = ndamp + 1;
                    DAMP.ID(ndamp) = int32(num_field_parser(tline, 2));
                    type = string_field_parser(tline, 3);
                    switch(type)
                      case 'G'
                        DAMP.Type(ndamp) = 1;
                      case 'CRIT'
                        DAMP.Type(ndamp) = 2;
                      case 'Q'
                        DAMP.Type(ndamp) = 3;
                      otherwise
                        error(['Unknown damping definition type in TABDMP1 ', num2str(DAMP.ID(ndamp)),'.']);
                    end
                    ne = 0;
                    skip_line = false;
                    data = [];
                    while ~skip_line
                      tline = fgetl(fp);
                      if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                        skip_line = false;
                        for i= 2:9
                          if (strcmp(strtok(string_field_parser(tline, i)), 'ENDT'))
                            break;
                          end
                          ne = ne+1;
                          data(ne) = num_field_parser(tline, i);
                        end
                      else
                        skip_line = true;
                      end
                    end
                    if (mod(ne,2)) 
                      error(['Unpaired values of frequency and damping in TABDMP1 ', num2str(DAMP.ID(ndamp)),'.']);
                    end                    
                    DAMP.Freq{ndamp} = data(1:2:end);
                    DAMP.g{ndamp} = data(2:2:end);
                    [DAMP.Freq{ndamp}, index] = sort(DAMP.Freq{ndamp});
                    DAMP.g{ndamp} = DAMP.g{ndamp}(index);
                    if (DAMP.Type(ndamp)==3)
                      index = find(DAMP.g{ndamp}<eps);
                      DAMP.g{ndamp}(index) = eps;
                    end

               case keyword{65} % CAEROB

                nbaero = nbaero +1;
                BAERO.ID(nbaero)    = int32(num_field_parser(tline, 2));
                for i=1:3
                  BAERO.geo.ref_point(nbaero,i) = num_field_parser(tline, i+2);
                end
                BAERO.CP(nbaero)    = int32(num_field_parser(tline, 6));
                BAERO.geo.L(nbaero) = num_field_parser(tline, 7);
                if (BAERO.geo.L(nbaero)) == 0
                  error(['Null length for body ', num2str(BAERO.ID(nbaero))]);
                end
                BAERO.geo.Nelem(nbaero)= int32(num_field_parser(tline, 8));
                if (BAERO.geo.Nelem(nbaero)) == 0
                  error(['Null value of mesh points for body ', num2str(BAERO.ID(nbaero)), '.']);
                end
                BAERO.SET(nbaero) = int32(num_field_parser(tline, 9));
                skip_line = false;
                data = []; ne = 0; offset = 0; found = 0;
                while ~skip_line
                  tline = fgetl(fp);
                  if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                    skip_line = false;
                    for i= 1:4
                      if (strcmp(strtok(string_field_parser(tline, 2*i)), 'ENDT'))
                        found = 1;
                        skip_line = true;
                        break;
                      end
                      ne = ne+1;
                      data(ne,1) = num_field_parser(tline, 2*i); % store section fraction
                      data(ne,2) = num_field_parser(tline, 2*i+1); % store section radius
                    end
                  else
                    skip_line = true;
                  end
                end
                if (found==0)
                  error(['No ENDT found in CAEROB card ', num2str(BAERO.ID(nbaero)),'.']);
                end
                if (ne==0)
                  error(['No section data provided for body ', num2str(BAERO.ID(nbaero)), '.']);
                end
                [BAERO.geo.fs{nbaero}, index] = sort(data(:,1)); 
                BAERO.geo.Rs{nbaero}           = data(index,2); 
                index = find(BAERO.geo.fs{nbaero}>1);
                if (~isempty(index))
                  error(['Lenght fraction points greater than 1 for body ', num2str(BAERO.ID(nbaero)), '.']);
                end

              case keyword{66} % TRIMEXT

                ntrimext = ntrimext+1;
                CAERO.Trim.Ext.ID(ntrimext) = int32(num_field_parser(tline, 2));
                tline = tline(2*FIELD+1:end); % filename
                index = find(tline == '''');
                CAERO.Trim.Ext.File{ntrimext}.data = {};
                for i=1:length(index)/2
                  i1 = (i-1)*2+1;
                  i2 = i*2;
                  CAERO.Trim.Ext.File{ntrimext}.data(i) = {tline(index(i1)+1:index(i2)-1)};
                end
                data = [];
                tline = fgetl(fp);
                CAERO.Trim.Ext.Type(ntrimext) = int32(num_field_parser(tline, 2));
                CAERO.Trim.Ext.Coord(ntrimext) = int32(num_field_parser(tline, 3));
                CAERO.Trim.Ext.Symmetry(ntrimext) = int32(num_field_parser(tline, 4));
                CAERO.Trim.Ext.DExt(ntrimext) = int32(num_field_parser(tline, 5));
                CAERO.Trim.Ext.Set(ntrimext) = int32(num_field_parser(tline, 6));
                CAERO.Trim.Ext.Index(ntrimext,1) = int32(num_field_parser(tline, 7));
                CAERO.Trim.Ext.Index(ntrimext,2) = int32(num_field_parser(tline, 8));
                CAERO.Trim.Ext.Toll(ntrimext) = int32(num_field_parser(tline, 9));
                tline = fgetl(fp);
                for r=2:9
                  data(r-1) = int32(num_field_parser(tline, r));
                end
                skip_line = false;
                ne = length(data);
                while ~skip_line
                  tline = fgetl(fp);
                  if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                    skip_line = false;
                    for i= 2:9
                      ne = ne+1;
                      data(ne) = int32(num_field_parser(tline, i));
                    end
                  else
                    skip_line = true;
                  end
                end
                CAERO.Trim.Ext.CAERO(ntrimext).data = unique(data(data>0));
%
              case keyword{67} % DEREXT
                
                nderext = nderext +1;
                CAERO.Trim.DExt.ID(nderext) = int32(num_field_parser(tline, 2));
                skip_line = false;
                ne = 0;
                while ~skip_line
                  tline = fgetl(fp);
                  if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                    skip_line = false;
                    if length(string_field_parser(tline, 2))
                      ne = ne +1;
                      CAERO.Trim.DExt.DOF(nderext).data(ne) = {string_field_parser(tline, 2)};
                      CAERO.Trim.DExt.Pert(nderext).data(ne) = num_field_parser(tline, 3);
                      if CAERO.Trim.DExt.Pert(nderext).data(ne) == 0
                        error(['Null pertubation value for DEREXT card ', num2str(CAERO.Trim.DExt.ID(nderext)),'.']);  
                      end
                      tline = tline(3*FIELD+1:end); % filename
                      index = find(tline == '''');
                      for i=1:length(index)/2
                        i1 = (i-1)*2+1;
                        i2 = i*2;
                       CAERO.Trim.DExt.File{nderext}.data{ne,i} = tline(index(i1)+1:index(i2)-1);
                      end

                    end
                  else
                    skip_line = true;
                  end
                end
%
%
              case keyword{68} % AERUPD
                naerupd = naerupd +1;
                AERUPD.ID(naerupd) = int32(num_field_parser(tline, 2));  
                AERUPD.PART(naerupd) = int32(num_field_parser(tline, 3));  
                AERUPD.NX(naerupd) = int32(num_field_parser(tline, 4));  
                AERUPD.NY(naerupd) = int32(num_field_parser(tline, 5));  
                AERUPD.FNX(naerupd) = int32(num_field_parser(tline, 6));  

              case keyword{69} % DESVAR
                ndesvar = ndesvar +1;
                DESOPT.DESVAR.ID(ndesvar) = int32(num_field_parser(tline, 2));  
                DESOPT.DESVAR.NAME{ndesvar} = name_field_parser(tline, 3);
                DESOPT.DESVAR.X0(ndesvar) = num_field_parser(tline, 4);  
                DESOPT.DESVAR.XL(ndesvar) = num_field_parser(tline, 5);  
                DESOPT.DESVAR.XU(ndesvar) = num_field_parser(tline, 6);
                DESOPT.DESVAR.DELTA(ndesvar) = abs(num_field_parser(tline, 7));
                if DESOPT.DESVAR.DELTA(ndesvar)== 0
                  DESOPT.DESVAR.DELTA(ndesvar) = 0.5;
                end


              case keyword{70} % DLINK
                ndlink = ndlink +1;
                data = [];
                DESOPT.DLINK.ID(ndlink) = int32(num_field_parser(tline, 2));  
                DESOPT.DLINK.FUN{ndlink} = tline(17:end);

              case keyword{71} % DVPREL
                ndvprel = ndvprel  +1;
                DESOPT.DVPREL.ID(ndvprel) = int32(num_field_parser(tline, 2));
                DESOPT.DVPREL.PID(ndvprel) = int32(num_field_parser(tline, 4));
                DESOPT.DVPREL.XL(ndvprel) = int32(num_field_parser(tline, 6));
                DESOPT.DVPREL.XU(ndvprel) = int32(num_field_parser(tline, 7));
                if DESOPT.DVPREL.XL(ndvprel)<0
                  DESOPT.DVPREL.XL(ndvprel) = 0;
                end
                if DESOPT.DVPREL.XU(ndvprel)==0
                  DESOPT.DVPREL.XU(ndvprel) = realmax;
                end
%
                switch(deblank(name_field_parser(tline, 3))) 
                  case 'PBAR'
                    DESOPT.DVPREL.Type(ndvprel) = 1;

                    switch(deblank(name_field_parser(tline, 5)))
                      case {'A', 'a'}
                        index = 1;
                      case {'I1', 'i1'}
                        index = 2;
                      case {'I2', 'i2'}
                        index = 3;
                      case {'J', 'j'}
                        index = 4;
                      case {'I12', 'i12'}
                        index = 5;
                      otherwise
                        error(['Unknown property name for PBAR ', num2str(DESOPT.DVPREL.PID(ndvprel))]);
                    end
                    DESOPT.DVPREL.Index(ndvprel) = index;
%
                  otherwise
                    error('Only PBAR can be used as design property.');
                end  

                skip_line = false;
                while ~skip_line
                  tline = fgetl(fp);
                  if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                    skip_line = false;
                     DESOPT.DVPREL.FUN{ndvprel} = tline(9:end);
                  else
                    skip_line = true;
                  end
                end


               case keyword{72} % DRESP
                ndresp = ndresp  +1;
                DESOPT.DRESP.ID(ndresp) = int32(num_field_parser(tline, 2));
                DESOPT.DRESP.NAME{ndesvar} = name_field_parser(tline, 3);
                switch(deblank(name_field_parser(tline, 4))) 
                  case {'DISP', 'disp'}
                    type = 1;
                  case {'STRAIN', 'strain'}
                    type = 2;
                  case {'FORCE', 'force'}
                    type = 3;
                  case {'STRESS', 'stress'}
                    type = 4;
                end
                DESOPT.DRESP.Type(ndresp) = type;
                DESOPT.DRESP.Index(ndresp) = 0;
                switch(deblank(name_field_parser(tline, 5))) 
                  case {'BAR', 'bar'}
                    DESOPT.DRESP.Index(ndresp) = 1;
                end
                DESOPT.DRESP.Comp(ndresp) = int32(num_field_parser(tline, 6));
                DESOPT.DRESP.Set(ndresp) = int32(num_field_parser(tline, 7));
                DESOPT.DRESP.XL(ndresp) = int32(num_field_parser(tline, 8));
                DESOPT.DRESP.XU(ndresp) = int32(num_field_parser(tline, 9));


               case keyword{73} % TRIMRESP
                ntrimresp = ntrimresp  +1;
                DESOPT.TRIMRESP.ID(ntrimresp) = int32(num_field_parser(tline, 2));
                DESOPT.TRIMRESP.NAME{ntrimresp} = name_field_parser(tline, 3);
                DESOPT.TRIMRESP.Comp{ntrimresp} = name_field_parser(tline, 4);
                DESOPT.TRIMRESP.XL(ntrimresp) = int32(num_field_parser(tline, 5));
                DESOPT.TRIMRESP.XU(ntrimresp) = int32(num_field_parser(tline, 6));
                DESOPT.TRIMRESP.Index(ntrimresp) = 0;

               case keyword{74} % STABRESP
                nstabresp = nstabresp  +1;
                DESOPT.STABRESP.ID(nstabresp) = int32(num_field_parser(tline, 2));
                DESOPT.STABRESP.NAME{nstabresp} = name_field_parser(tline, 3);
                DESOPT.STABRESP.Set{nstabresp}  = name_field_parser(tline, 4);
                DESOPT.STABRESP.Comp(nstabresp, 1) = int32(num_field_parser(tline, 5));
                DESOPT.STABRESP.XL(nstabresp) = int32(num_field_parser(tline, 6));
                DESOPT.STABRESP.XU(nstabresp) = int32(num_field_parser(tline, 7));

                case keyword{75} % CAERO0 nastran CAERO1 format
                  ncaero = ncaero +1;
                  ncaero0 = ncaero0 +1;
                  CAERO.geo.nelem(ncaero) = -1;
                  CAERO.geo.dihed(ncaero, 1) = 0;
                  CAERO.geo.foil(ncaero, 1, 1) = {'0'};
                  CAERO.geo.foil(ncaero, 1, 2) = {'0'};
                  CAERO.ID(ncaero) = int32(num_field_parser(tline, 2));
                  CAERO.CP(ncaero) = int32(num_field_parser(tline, 4));
                  CAERO.geo.ny(ncaero,1) = int32(num_field_parser(tline, 5));
                  if (CAERO.geo.ny(ncaero,1) == 0)
                      fclose(fp);error('Read null span panel numbers for CAERO entry %d.', CAERO.ID(ncaero));
                  end
                  CAERO.geo.nx(ncaero,1) = int32(num_field_parser(tline, 6));
                  if (CAERO.geo.nx(ncaero,1) == 0)
                      fclose(fp);error('Read null chord panel numbers for CAERO entry %d.', CAERO.ID(ncaero));
                  end
                  if (int32(num_field_parser(tline, 7))>0) || (int32(num_field_parser(tline, 8)))
                    error('No AEFACT card allowed for CAERO0 definition.');
                  end
                  CAERO.geo.meshtype(ncaero,1) = int32(num_field_parser(tline, 9));
                  if ((CAERO.geo.meshtype(ncaero,1) <1) || (CAERO.geo.meshtype(ncaero,1) >4) && (CAERO.geo.meshtype(ncaero,1) <6))
                    CAERO.geo.meshtype(ncaero,1)  = 1;
                  end
                  CAERO.INT(ncaero) = int32(num_field_parser(tline, 10));
                  skip_line = false;
                  tline = fgetl(fp);
                  if ( (length(tline) > 1) && (isequal(tline(1),' ') || isequal(tline(1),'+'))) % continuation detected
                    skip_line = false;
                    CAERO.geo.startx(ncaero) = num_field_parser(tline, 2);
                    CAERO.geo.starty(ncaero) = num_field_parser(tline, 3);
                    CAERO.geo.startz(ncaero) = num_field_parser(tline, 4);
                    CAERO.geo.c(ncaero)       = num_field_parser(tline, 5);
                    P1 = zeros(3,1); P2 = zeros(3,1);
                    P3 = zeros(3,1); P4 = zeros(3,1);
                    P1(1) = CAERO.geo.startx(ncaero);
                    P1(2) = CAERO.geo.starty(ncaero);
                    P1(3) = CAERO.geo.startz(ncaero);
                    P2(1) = num_field_parser(tline, 6);
                    P2(2) = num_field_parser(tline, 7);
                    P2(3) = num_field_parser(tline, 8);
                    P3 = P1; P3(1) = P3(1) + CAERO.geo.c(ncaero)/4;
                    P4 = P2; P4(1) = P4(1) + num_field_parser(tline, 9)/4;
                    X = P3 - P1; X = X ./ norm(X);
                    Y = P2 - P1; Y = Y ./ norm(Y);
                    Z = crossm(X)*Y; Z = Z ./ norm(Z);
                    V = P4-P3;
                    Y2 = crossm(Z)*X;Y2 = Y2 ./ norm(Y2);
                    AEROLE(ncaero0,:) = Y2; 
                    CAERO.geo.b(ncaero,1) = dot(Y2, V); %sqrt(V(2)^2 + V(3)^2)
                    V = V ./norm(V);
                    CAERO.geo.SW(ncaero,1) = acos(dot(Y2,V));
                    if (P2(1)<P1(1))
                      CAERO.geo.SW(ncaero,1) = -CAERO.geo.SW(ncaero,1);
                    end
                    CAERO.geo.TW(ncaero,1,1)  = 0;
                    CAERO.geo.TW(ncaero,1,2)  = 0;
%                    CAERO.geo.b(ncaero,1) = norm(P2-P1);
                    if (P2(2)<P1(2))
                      CAERO.geo.b(ncaero,1) = -CAERO.geo.b(ncaero,1);
                    end
                    CAERO.geo.T(ncaero,1)   = num_field_parser(tline, 9) / CAERO.geo.c(ncaero);
                    CAERO.geo.flapped(ncaero,1) = 0;
                    CAERO.geo.fc(ncaero,1,1) = 0;
                    CAERO.geo.fc(ncaero,1,2) = 0;
                    CAERO.geo.fnx(ncaero,1) = 0;
                    CAERO.geo.fsym(ncaero,1) = 0;
                  else
                    skip_line = true;
                  end

                case keyword{76} % AESURF
                  naesurf = naesurf +1;
                  CAERO.AESURF.ID(naesurf) = int32(num_field_parser(tline, 2));
                  CAERO.AESURF.Name{naesurf} = name_field_parser(tline, 3);
                  CAERO.AESURF.CID(naesurf) = int32(num_field_parser(tline, 4));
                  CAERO.AESURF.AERID(naesurf) = int32(num_field_parser(tline, 5));
                  if int32(num_field_parser(tline, 6))>0 || int32(num_field_parser(tline, 7))>0
                    error('Only one control surface can be defined in AESURF card.');
                  end
                    

            end % end of switch
            
        end % end of while
        fclose(fp);
        %
        if (cc_trim>0 && i_trim>0)
          if (cc_trim~=i_trim)  
            error('Number of parameters ITRIM= differs from TRIM=');
          end
        end
        %
        % check for other files to read
        %
        if (length(PARAM.INCLUDE) == NFILE)
            READ_INCLUDE = false;
        else
            if ~exist(PARAM.INCLUDE{NFILE+1}, 'file')
                error(['Unable to find file %s.', PARAM.INCLUDE{NFILE+1},'.']);
            end
        end
        %
    end % INCLUDE
    % clean data
    ninclude = ninclude -1;
    if (ninclude>0)
        PARAM.INCLUDE = PARAM.INCLUDE(2:end);
    end
    %*******************************************************************************
    %
    %       update geo mesh and spline sets 
    %
        for i=1:naerupd
          id = AERUPD.ID(i);
          nx = AERUPD.NX(i); ny = AERUPD.NY(i); part = AERUPD.PART(i); fnx = AERUPD.FNX(i);
          index = find(id ==  CAERO.ID);
          if ~isempty(index)
            fprintf(fid, '\n\t### Warning: CAERO1 %d mesh will be updated at partition %d with nx=%d, ny=%d, fnx=%d.', id, part, nx, ny, fnx);
            if nx>0, CAERO.geo.nx(index,part) = nx; end
            if ny>0, CAERO.geo.ny(index,part) = ny; end
            if (fnx>0 && CAERO.geo.fnx(index,part)>0), CAERO.geo.fnx(index,part) = fnx; end
          else
            error(['Card AERUPD refer to unknown CAERO1 ', num2str(id),'.']);
          end
          if ninterp
            indexs = find(id ==  CAERO.Interp.Patch);
            if ~isempty(indexs)
              CAERO.Interp.Index(indexs,2) = (sum(CAERO.geo.nx(index,:),2) + sum(CAERO.geo.fnx(index,:),2)) * CAERO.geo.ny(index,:);
              fprintf(fid, '\n\t### Warning: second SPLINE interpolation index for CAERO %d set to %d.', id, CAERO.Interp.Index(indexs,2));
            end
          end
        end

    %*******************************************************************************
    % sort NODE database
    
    if ngrid >0
        
        fprintf(fid, '\n\tSorting Node database...');
        
        [NODE.ID, index] = sort(NODE.ID);
        [labels, i] = unique(NODE.ID);
        % check for duplicated labels
        if (length(labels) ~= ngrid)
            
            n = [1 : ngrid];
            dof = NODE.ID(setdiff(n, i));
            
            for k=1:length(dof)
                
                fprintf(fid, '\n\t### Warning: duplicated labels for grid: %d.', NODE.ID(dof(k)));
                
            end
            
            error('Grid entries have duplicated labels.');
            
        end
        
        NODE.CS = NODE.CS(index);
        NODE.Coord = NODE.Coord(index,1:3);
        NODE.CD = NODE.CD(index);
        %NODE.PS = NODE.PS(index);
        
        fprintf(fid, 'done.');
        
        
    end
    %*******************************************************************************
    % set COORD1 database
    if ncord1>0
        
        [COORD1.ID, index] = sort(COORD1.ID);
        [labels, i] = unique(COORD1.ID);
        
        if (length(labels) ~= ncord1)
            
            n = [1 : ncord1];
            dof = COORD1.ID(setdiff(n, i));
            for k=1:length(dof)
                
                fprintf(fid, '\n\tCOORD1 entries have duplicated labels: %d', COORD1.ID(dof(k)));
                
            end
            error('COORD1 entries have duplicated labels.');
        end
        
        COORD1.Nodes = COORD1.Nodes(index,1:3);
        % add two fields
        COORD1.R = zeros(3,3, ncord1);
        COORD1.Origin = zeros(ncord1,3);
        
        for n=1:ncord1
            
            index = [];
            
            for i=1:3
                
                index(i) = find(NODE.ID == COORD1.Nodes(n,i));
                
                if isempty(index(i))
                    error('Node %d not declared for COORD1 %d definition.', COORD1.Nodes(n,i), COORD1.ID(n));
                end
                
            end
            
            COORD1.Origin(n,1:3) = NODE.Coord(index(1),1:3);
            x3 = NODE.Coord(index(2),1:3) - NODE.Coord(index(1),1:3);
            x1 = NODE.Coord(index(3),1:3) - NODE.Coord(index(1),1:3);
            x2 = cross(x3, x1);
            
            if det([x1; x2; x3]) == 0
                
                error('COORD1 %d definition has two collinear points.', COORD1.ID(n));
                
            end
            
            x3 = x3 ./ norm(x3);
            x2 = x2 ./ norm(x2);
            x1 = cross(x2, x3);
            x1 = x1 ./ norm(x1);
            
            COORD1.R(:,:,n) = [x1', x2', x3'];
            
        end
    end
    %*******************************************************************************
    % set COORD2 database
    
    if ncord2>0
        
        [COORD2.ID, index] = sort(COORD2.ID);
        [labels, i] = unique(COORD2.ID);
        if (length(labels) ~= ncord2)
            n = [1 : ncord2];
            error(['CORD2R entries have duplicated labels: ', num2str(COORD2.ID(setdiff(n, i)))]);
        end
        COORD2.Nodes = COORD2.Nodes(:,:,index);
        COORD2.RID = COORD2.RID(index);
        for n=1:ncord2
          if COORD2.RID(n)~=0
            index = find(COORD2.ID == COORD2.RID(n));
            if isempty(index)
              error(['Unable to find CORD2R ', num2str(COORD2.RID(n)), ' declared in CORD2R ', num2str(COORD2.ID(n))]);
            else
              COORD2.RID(n) = index;
            end
          end
        end
        % add two fields
        COORD2.R = zeros(3, 3, ncord2);
        COORD2.Origin = zeros(ncord2,3);
        TAG = zeros(ncord2,1);
        % loop on master COORD        
        for n=1:ncord2
            if COORD2.RID(n) == 0
              TAG(n) = 1;
              [COORD2.Origin(n,1:3), COORD2.R(:,:,n)] = cord2r_data(COORD2, n);
            end
        end
        % loop on 1st and 2nd type dependence
        for k=1:CORDIT
          for n=1:ncord2
              m = COORD2.RID(n);  
              if m ~= 0
                if TAG(m) == 1 && TAG(n) == 0
                  TAG(n) = 1;
                  [COORD2.Origin(n,1:3), COORD2.R(:,:,n)] = cord2r_data(COORD2, n);
                  COORD2.Origin(n,1:3) = COORD2.Origin(m,1:3) + (COORD2.R(:,:,m) * COORD2.Origin(n,1:3)')';
                  COORD2.R(:,:,n) = COORD2.R(:,:,m) * COORD2.R(:,:,n);
                end
              end
          end
          if sum(TAG) == ncord2
            break;
          end
        end
        if k == CORDIT && sum(TAG) < ncord2
          error('Unable to solve CORD2R dependency within 10 iterations. Please change CORD2R dependency.');
        end
    end
    %*******************************************************************************
    % create a unique COORD database
    
    ntot = ncord1 + ncord2;
    ncord = ntot;
    
    if ntot >0
        
        fprintf(fid, '\n\tSorting Coordinate System database...');
        
        COORD.ID = [COORD1.ID, COORD2.ID];
        COORD.Origin = zeros(ntot, 3);
        COORD.R = zeros(3, 3, ntot);
        
        COORD.Origin = [COORD1.Origin;
            COORD2.Origin];
        
        for n=1:ncord1
            
            COORD.R(:,:,n) = COORD1.R(:,:,n);
            
        end
        i = 0;
        for n=ncord1+1:ntot
            i = i + 1;
            COORD.R(:,:,n) = COORD2.R(:,:,i);
            
        end
        
        [COORD.ID, index] = sort(COORD.ID);
        [labels, i] = unique(COORD.ID);
        
        if (length(labels) ~= ntot)
            
            n = [1 : ntot];
            
            error('COORD1 and COORD2 entries have duplicated labels: %d', COORD.ID(setdiff(n, i)));
            
        end
        
        COORD.Origin = COORD.Origin(index,1:3);
        COORD.R = COORD.R(:,:,index);
        
        fprintf(fid, 'done.');
        
    end
    %*******************************************************************************
    % determine dihedral for CAERO0 cards
    n = 0;
    for i=1:ncaero
      if CAERO.geo.nelem(i) <0
        n = n+1; CAERO.geo.nelem(i) = 1;
        CAERO.geo.meshtype(i,1) = 1;
        index = find(COORD.ID == CAERO.CP(i));
        V = COORD.R(:,:,index) * AEROLE(n,:)'; V = V ./norm(V);
        if CAERO.geo.b(i) <0
          CAERO.geo.dihed(i, 1) = -acos(dot(V, [0 -1 0]));
          CAERO.geo.SW(i, 1) = -CAERO.geo.SW(i, 1);
        else
          CAERO.geo.dihed(i, 1) = acos(dot(V, [0 1 0]));
        end
      end
    end
    %*******************************************************************************
    % set AESURF
    if naesurf
      [id, j] = unique(CAERO.AESURF.ID);
      if length(id)~=naesurf
        n = [1:naesurf];
        error(['Duplicated ID for AESURF cards ', num2str(CAERO.AESURF.ID(setdiff(n,j))),'.']);
      end
      for n=1:naesurf
        id = CAERO.AESURF.AERID(n);
        index = find(CAERO.ID == id);
        if isempty(index)
          error(['Unable to find CAERO ', num2str(id), ' referenced by AESURF ', CAERO.AESURF.ID(n),'.']);
        end
        CAERO.geo.flapped(index,1) = 1;
        CAERO.geo.meshtype(index,1) = 6;
        CAERO.geo.nc = CAERO.geo.nc +1;
        CAERO.geo.fc(index,1,1) =  1; CAERO.geo.fc(index,1,2) =  1;
        CAERO.geo.fnx(index,1) = CAERO.geo.nx(index,1);
        CAERO.geo.nx(index,1) = 0;
        CONTROL_NAME(CAERO.geo.nc) = {CAERO.AESURF.Name{n}};
      end
    end
    %*******************************************************************************
    % set FORCE struct
    if nforce >0
        
        fprintf(fid, '\n\tSorting Force database...');
        
        FORCE = get_load_dir(nforce, FORCE, NODE, COORD);
        
        fprintf(fid, 'done.');
    end
    % set MOMENT struct
    
    if nmom >0
        
        fprintf(fid, '\n\tSorting Moment database...');
        
        MOMENT = get_load_dir(nmom, MOMENT, NODE, COORD);
        
        fprintf(fid, 'done.');
    end
    
    % set follower forces
    
    if nflw
        
        fprintf(fid, '\n\tSorting Follower Force database...');
        
        F_FLW = get_flw_load_dir(nflw, F_FLW, NODE, COORD);
        
        fprintf(fid, 'done.');
        
    end
    %*******************************************************************************
    % define GRID in GLOBAL reference frame
    
    index = find (NODE.CS);
    for n=1:length(index)
        
        node = index(n);
        i = find(COORD.ID == NODE.CS(node));
        if isempty(i)
            
            error('Unable to find coordinate system %d for node %d definition.', ...
                NODE.CS(node), NODE.ID(node));
            
        end
        NODE.Coord(node, 1:3) = COORD.Origin(i,1:3) + (COORD.R(:,:,i) * NODE.Coord(node, 1:3)')';
        NODE.CS(node) = i; % overwrite id with index for rapid accessing
        
    end
    
    index = find(NODE.CD);
    for n=1:length(index)
        
        node = index(n);
        i = find(COORD.ID == NODE.CD(node));

        if isempty(i)
            
            error('Unable to find coordinate system %d for node %d definition.', ...
                NODE.CD(node), NODE.ID(node));
            
        end
        
        NODE.CD(node) = i; % overwrite with index for rapid accessing
        
    end
    
    %*******************************************************************************
    % check for LOAD definition
    if nload > 0
        fprintf(fid, '\n\tNumber of load set given: %d.', nload);
        if  PARAM.LOAD
            fprintf(fid, '\n\t\tActive load set: %d.', PARAM.LOAD);
            % define GRAV load
            if (ngrav)
                fprintf(fid, '\n\t\t\tDefining gravity load: %d.', PARAM.LOAD);
                index = find(PARAM.LOAD == GRAV.ID);
                if (length(index) > 1)
                    error('Multiple defined GRAV cards for load set %d.', PARAM.LOAD);
                end
                if (GRAV.CID(index))
                    i = find(COORD.ID == GRAV.CID(index))
                    PARAM.GRAV = GRAV.Mag(index) .* (COORD.R(:,:,i) * GRAV.Orient(:, index));
                else
                    PARAM.GRAV = GRAV.Mag(index) .* GRAV.Orient(:,index);
                end
            end
            % end define GRAV
        end
    else
        fprintf(fid, '\n\tNo load set given.');
    end
    % check for SPC1 definition
    if nspc > 0
        fprintf(fid, '\n\tNumber of constraint set given: %d.', nspc);
        if PARAM.SPC
            fprintf(fid, '\n\t\tActive constraint set: %d.', PARAM.SPC);
            % check if all nodes given are defined
            spcindex = find( SPC.ID == PARAM.SPC);
            for j=1:length(spcindex)
                [v, i] = intersect(NODE.ID, SPC.Nodes(spcindex(j)).list);
                
                if length(i) ~= length(SPC.Nodes(spcindex(j)).list)
                    
                    error('Unable to determine nodes given in SPC set %d.', SPC.ID(spcindex(j)));
                    
                end
                SPC.Nodes(spcindex(j)).list = i;
                
            end
        end
    else
        fprintf(fid, '\n\tNo constraint set given.');
    end
    %*******************************************************************************
    % set MAT struct
    
    if nmat >0
        
        fprintf(fid, '\n\tSorting Material database...');
        
        [MAT.ID, index] = sort(MAT.ID);
        [labels, i] = unique(MAT.ID);
        
        if (length(labels) ~= nmat)
            
            n = [1 : nmat];
            error('Material entries have duplicated labels: %d', MAT.ID(setdiff(n, i)));
            
        end
        
        MAT.E = MAT.E(index);
        MAT.G = MAT.G(index);
        MAT.nu = MAT.nu(index);
        MAT.ST = MAT.ST(index);
        MAT.SC = MAT.SC(index);
        MAT.SS = MAT.SS(index);
        MAT.Rho = MAT.Rho(index);
        MAT.Rho = PARAM.WTMASS .* MAT.Rho;
        fprintf(fid, 'done.');
        
        index = find(MAT.Rho==0);
        if (~isempty(index))
            fprintf(fid, '\n\t\t### Warning: material %d has null density.', MAT.ID(index));
        end
        
    else
        
        fprintf(fid, '\n\t### Warning: no material card available.');
        
    end
    %*******************************************************************************
    % set PBAR struct
    
    if npbar >0
        fprintf(fid, '\n\tSorting Bar Property database...');
        
        [PBAR.ID, index] = sort(PBAR.ID);
        [labels, i] = unique(PBAR.ID);
        
        if (length(labels) ~= npbar)
            
            n = [1 : npbar];
            error('Bar property entries have duplicated labels: %d', PBAR.ID(setdiff(n, i)));
            
        end
        
        %
        PBAR.Mat = PBAR.Mat(index);
        PBAR.Type = PBAR.Type(index);
        PBAR.SI = PBAR.SI(index);
        
        for n=1:npbar
            
            i = find(MAT.ID == PBAR.Mat(n));
            if isempty(i)
                error('Unable to find Material %d for Bar property %d.', PBAR.Mat(n), PBAR.ID(n));
            else
                PBAR.Mat(n) = i; % substitute MAT ID with MAT index for rapid accessing
            end
            
        end
        
        PBAR.A = PBAR.A(index);
        PBAR.I = PBAR.I(index,:);
        PBAR.J = PBAR.J(index);
        PBAR.RhoNS = PARAM.WTMASS .* PBAR.RhoNS(index);
        PBAR.Kshear = PBAR.Kshear(index,:);
        PBAR.Str_point = PBAR.Str_point(:,:,index);
        PBAR.RhoNSV = zeros(size(PBAR.RhoNS));
        
        if (npbarsm)
            %   assembly missing data for user defined section properties
            %   Fuselage
            PBAR = smonoq1_fuse_stiff(fid, PBAR);    % unframed fuselage from GUESS
            PBAR = smonoq2_fuse_stiff(fid, PBAR);    % framed fuselage from GUESS
            %   lifting surfaces
            PBAR = smonoq_mweb_stiff(fid, PBAR);     % bi-symmetric multi-web wing-box type 1
            %   real wingbox with stringers
            PBAR = smonoqwb1_stiff(fid, PBAR,0);% wing-box+stringer type 1
            PBAR = smonoqwb1_stiff(fid, PBAR,1);% wing-box+stringer type 1
            
        end
        fprintf(fid, '\n\tdone.');
        
    end
    
    % set PBEAM struct
    
    if npbeam >0
        
        fprintf(fid, '\n\tSorting Beam Property database...');
        
        [PBEAM.ID, index] = sort(PBEAM.ID);
        [labels, i] = unique(PBEAM.ID);
        
        if (length(labels) ~= npbeam)
            
            n = [1 : npbeam];
            error('Beam property entries have duplicated labels: %d', PBEAM.ID(setdiff(n, i)));
            
        end
        
        PBEAM.Mat = PBEAM.Mat(index);
        
        for n=1:npbeam
            
            i = find(MAT.ID == PBEAM.Mat(n));
            if isempty(i)
                error('Unable to find Material %d for Beam property %d.', PBEAM.Mat(n), PBEAM.ID(n));
            else
                PBEAM.Mat(n) = i; % substitute MAT ID with MAT index for rapid accessing
            end
            
        end
        
        PBEAM.A = PBEAM.A(index);
        PBEAM.I = PBEAM.I(index);
        PBEAM.J = PBEAM.J(index);
        PBEAM.RhoNS = PBEAM.RhoNS(index);
        PBEAM.X_L = PBEAM.X_L(index);
        % node propoerties
        PBEAM.Kshear = PBEAM.Kshear(index, :);
        PBEAM.NSI = PBEAM.NSI(index,:);
        PBEAM.NSCG = PBEAM.NSCG(index,:);
        PBEAM.NA = PBEAM.NA(index,:);
        % section propoerties
        PBEAM.DA = PBEAM.DA(:, :, index);
        PBEAM.DI = PBEAM.DI(:, :, index);
        PBEAM.DJ =  PBEAM.DJ(:, :, index);
        PBEAM.DRhoNS =  PBEAM.DRhoNS(:, :, index);
        
        fprintf(fid, 'done.');
        
    end
    
    %*******************************************************************************
    % set interpolation database
    
    if nset
        
        index = unique(CAERO.Set.ID);
        
        if length(index) ~= nset
            n = setdiff(CAERO.Set.ID, index);
            error('Duplicated ID for structural SET1/AELIST %d.', n(1));
            
        end
        
    end
    
    for n=1:nset
        
        [data, index_n, index] = intersect(NODE.ID, CAERO.Set.Node(n).data);
        
        ne = length(CAERO.Set.Node(n).data);
        
        if length(index_n) ~= ne
            
            j = setdiff([1:ne], index);
            error('Unable to find node %d for structural set %d.', ...
                CAERO.Set.Node(n).data(j(1)), CAERO.Set.ID(n));
            
        end
        
        CAERO.Set.Node(n).data = index_n;
        
    end
    %*******************************************************************************
    % set AERONODES database
    
    if nrbe0
        
        for n=1:ngrid
            
            NODE.Aero.Coord(n).data = [];
            NODE.Aero.Index(n).data = [];
            
        end
        
        
        for n = 1:nrbe0
            
            i = find(RBE0.Master(n) == NODE.ID);
            if isempty(i)
                
                error('Unable to find master node %d in node database for aero rigid element %d.', ...
                    RBE0.Master(n), RBE0.ID(n));
                
            end
            
            RBE0.Master(n) = i;
            [data, index] = intersect(NODE.ID, RBE0.Node(n).data);
            
            ne = length(RBE0.Node(n).data);
            
            if length(index) ~= ne
                
                j = setdiff([1:ne], index);
                error('Unable to find slave aero node %d for aero rigid set %d.', ...
                    RBE0.Node(n).data(j(1)), RBE0.ID(n));
                
            end
            
            RBE0.Node(n).data = index;
            % add to master node, slave aero points
            coord = NODE.Coord(RBE0.Node(n).data, 1:3) - repmat(NODE.Coord(RBE0.Master(n), 1:3), ne, 1);
            NODE.Aero.Coord(RBE0.Master(n)).data = [NODE.Aero.Coord(RBE0.Master(n)).data, coord'];
            NODE.Aero.Index(RBE0.Master(n)).data = [NODE.Aero.Index(RBE0.Master(n)).data, RBE0.Node(n).data];
            
        end
        
    end
    
    %*******************************************************************************
    % set CBAR struct: This struct is not sorted by ID.
    % It is not important because elements are accessed within a loop
    
    if ncbar >0
        
        fprintf(fid, '\n\tSetting Bar database...');
        
        [labels, i] = unique(BAR.ID);
        
        if (length(labels) ~= ncbar)
            
            n = [1 : ncbar];
            error('Bar entries have duplicated labels: %d', BAR.ID(setdiff(n, i)));
            
        end
        
        BAR.Colloc = zeros(2, 3, ncbar); % collocation point coords
        BAR.R = zeros(3, 3, 5, ncbar);   % rotation material matrix
        BAR.D = zeros(6, 6, 2, ncbar);   % stiffness matrix
        BAR.M = zeros(6, 6, 3, ncbar);   % lumped body mass matrix associated to beams nodes
        % sort all database and use local indeces
        for n=1:ncbar
            
            % substitute NODE ID with NODE index for rapid accessing
            
            i = find(NODE.ID == BAR.Conn(n,1));
            if isempty(i)
                error('Unable to determine node %d position for CBAR %d.',BAR.Conn(n,1), BAR.ID(n));
            end
            
            j = find(NODE.ID == BAR.Conn(n,2));
            if isempty(j)
                error('Unable to determine node %d position for CBAR %d.',BAR.Conn(n,2), BAR.ID(n));
            end
            
            % add the third node
            NODE.ID(ngrid + n) = int32(NODE.ID(ngrid) + n);
            NODE.CS(ngrid + n) = int32(0);
            NODE.CD(ngrid + n) = int32(0);
            index = [i, ngrid + n, j];
            BAR.Conn(n,:) = index;
            NODE.Coord(ngrid + n,:) = mean([NODE.Coord(i,:) ; NODE.Coord(j,:)] ,1);
            % substitute PID ID with PBAR index for rapid accessing
            
            i = find(PBAR.ID == BAR.PID(n));
            
            if length(i) ~= 1
                
                error('Unable to determine property card for CBAR %d.', BAR.ID(n));
                
            end
            
            BAR.PID(n) = i;
            
            % overwrite offset if g0 is integer
            
            if BAR.barg0(n)
                
                i = find(NODE.ID == int32(BAR.Orient(n,1)));
                if isempty(i)
                    
                    error('Unale to find node G0 %d in node database for bar %d.', ...
                        int32(BAR.Orient(n,1)), BAR.ID(n));
                    
                end
                
                BAR.Orient(n,:) = NODE.Coord(i, :) - NODE.Coord(BAR.Conn(n,1), :);
                
            end
            
            % convert offset and orientation in BASIC reference frame
            
            % orient
            
            switch BAR.OffsetT(n)
                
                case {1,3,5,7}
                    
                    if NODE.CD(BAR.Conn(n,1)) ~= 0
                        
                        BAR.Orient(n,:) = (COORD.R(:, :, NODE.CD(BAR.Conn(n,1))) * BAR.Orient(n,:)')';
                        
                    end
            end
            
            % offset
            
            switch BAR.OffsetT(n)
                
                case {1,2} % GGG , BGG
                    
                    % offset1
                    if NODE.CD(BAR.Conn(n,1)) ~= 0
                        
                        BAR.Offset(n,1:3) = (COORD.R(:, :, NODE.CD(BAR.Conn(n,1))) * BAR.Offset(n,1:3)')';
                        
                    end
                    % offset3
                    if NODE.CD(BAR.Conn(n,3)) ~= 0
                        
                        BAR.Offset(n,7:9) = (COORD.R(:, :, NODE.CD(BAR.Conn(n,3))) * BAR.Offset(n,7:9)')';
                        
                    end
                    
                case {3,4} % GGO , BGO
                    
                    % offset1
                    if NODE.CD(BAR.Conn(n,1)) ~= 0
                        
                        BAR.Offset(n,1:3) = (COORD.R(:, :, NODE.CD(BAR.Conn(n,1))) * BAR.Offset(n,1:3)')';
                        
                    end
                    
                    R = get_basic_coord(BAR.Orient(n,:), NODE.Coord(BAR.Conn(n,3),:), NODE.Coord(BAR.Conn(n,1),:));
                    
                    % offset3
                    BAR.Offset(n,7:9) = (R * BAR.Offset(n,7:9)')';
                    
                    
                case {5,6} %GOG , BOG
                    
                    % offset3
                    if NODE.CD(BAR.Conn(n,3)) ~= 0
                        
                        BAR.Offset(n,7:9) = (COORD.R(:, :, NODE.CD(BAR.Conn(n,3))) * BAR.Offset(n,7:9)')';
                        
                    end
                    
                    R = get_basic_coord(BAR.Orient(n,:), NODE.Coord(BAR.Conn(n,3),:), NODE.Coord(BAR.Conn(n,1),:));
                    
                    % offset1
                    BAR.Offset(n,1:3) = (R * BAR.Offset(n,1:3)')';
                    
                    
                case {7,8} %GOO , BOO
                    
                    R = get_basic_coord(BAR.Orient(n,:), NODE.Coord(BAR.Conn(n,3),:), NODE.Coord(BAR.Conn(n,1),:));
                    
                    % offset1
                    BAR.Offset(n,1:3) = (R * BAR.Offset(n,1:3)')';
                    % offset3
                    BAR.Offset(n,7:9) = (R * BAR.Offset(n,7:9)')';
                    
                    
            end %switch
            
            % offset for the midline node
            BAR.Offset(n, 4:6) = get_midline_offset(NODE.Coord(BAR.Conn(n,1),:), ...
                NODE.Coord(BAR.Conn(n,2),:), NODE.Coord(BAR.Conn(n,3),:), BAR.Offset(n,1:3), BAR.Offset(n,7:9));
            
            % store global collocation point coordinates
            BAR.Colloc(:, :, n) = interp_colloc_pos(NODE.Coord(BAR.Conn(n,1),:) + BAR.Offset(n, 1:3), ...
                NODE.Coord(BAR.Conn(n,2),:) + BAR.Offset(n, 4:6), ...
                NODE.Coord(BAR.Conn(n,3),:) + BAR.Offset(n, 7:9));
            
            % build nodes and collocation point reference frames
            
            x1 = NODE.Coord(BAR.Conn(n,3),:) + BAR.Offset(n, 7:9) - (NODE.Coord(BAR.Conn(n,1),:) + BAR.Offset(n, 1:3));
            x2 = BAR.Orient(n,:);
            x3 = cross(x1, x2);
            if norm(x3) == 0
                
                error('Wrong Orientation direction definition for Bar %d. Please check', BAR.ID(n));
                
            end
            x2 = cross(x3, x1);
            
            x1 = x1 ./ norm(x1);
            x2 = x2 ./ norm(x2);
            x3 = x3 ./ norm(x3);
            R= [x1', x2', x3'];
            % when assembled all the reference frame belonging to the BAR are the same
            for i=1:5 % 3 reference for 3 nodes + 2 reference for two collocation points
                
                BAR.R(:,:,i, n) =  R;
                
            end
            
            %WARNING: offset are expressend in the basic coordinates system which
            %         correspond before deformation to each node reference frame
            % get local offset vector
            %BAR.Offset(n, 1:3) = (R' * BAR.Offset(n, 1:3)')';
            %BAR.Offset(n, 4:6) = (R' * BAR.Offset(n, 4:6)')';
            %BAR.Offset(n, 7:9) = (R' * BAR.Offset(n, 7:9)')';
            
            % assembly stiffnes matrix
            %
        end % bar loop
        % set stiffness and mass database
        BAR = set_cbar_database(ncbar, BAR, PBAR, MAT, NODE, COORD);
        %
        if nrbe0
            for n=ngrid+1:(ngrid + ncbar)
                
                NODE.Aero.Coord(n).data = [];
                NODE.Aero.Index(n).data = [];
                
            end
        end
        
        ngrid = ngrid + ncbar; % update node counter
        
        fprintf(fid, 'done.');
        
    end
    %*******************************************************************************
    % CONM struct
    
    CONM = [];
    mtot = ncom1 + ncom2;
    
    if mtot > 0
        
        CONM1.M = PARAM.WTMASS .* CONM1.M;
        CONM2.M = PARAM.WTMASS .* CONM2.M;
        
        CONM.Offset = zeros(mtot, 3);
        
        fprintf(fid, '\n\tSorting Lumped mass database...');
        
        % CONM1 struct
        i = 0;
        for n = 1 : ncom1
            
            i = i + 1;
            % set NODE index for rapid accessing
            j = find(CONM1.Node(i) == NODE.ID);
            
            if isempty(j)
                
                error('Unable to find node %d in Node database for CONM1 %d.', CONM1.Node(i), CONM1.ID(i));
                
            end
            
            CONM.Node(n) = j; % set index for rapid accessing
            
            CONM.M(:,:,n) = zeros(6,6);
            CONM.M(:,:,n) = CONM1.M(:,:,i);
            
            if CONM1.CID(i) > 0
                
                j = find(CONM1.CID(i) == COORD.ID);
                
                if isempty(j)
                    error('Unable to find reference frame %d in COORD database for CONM1 %d.', CONM1.CID(i), CONM1.ID(i));
                end
                
                zcg = CONM.M(5,1,n) / CONM.M(1,1,n);
                ycg = -CONM.M(6,1,n) / CONM.M(1,1,n);
                xcg = CONM.M(6,2,n) / CONM.M(1,1,n);
                v = [xcg; ycg; zcg];
                
                v = COORD.R(:,:,j) * v; % get mass cg in global reference frame
                
                CONM.Offset(n,:) = v'; % store offset
                
                S = zeros(3,3);
                S(2,1) =  CONM.M(1,1,n) * v(3);
                S(3,1) = -CONM.M(1,1,n) * v(2);
                S(1,2) = -S(2,1);
                S(3,2) =  CONM.M(1,1,n) * v(1);
                S(1,3) = -S(3,1);
                S(2,3) = -S(3,2);
                
                CONM.M(4:6, 1:3, n) = S;
                CONM.M(1:3, 4:6, n) = S';
                % mass matrix in basic reference frame in the node
                RJ = COORD.R(:,:,j) * sqrt(CONM1.M(4:6,4:6,i));
                CONM.M(4:6, 4:6, n) = RJ * RJ' - ...
                    CONM1.M(1,1,i) * crossm(CONM.Offset(n,:)) * crossm(CONM.Offset(n,:));
                
            end
            
            if CONM1.CID(n) == 0
                
                zcg = CONM.M(5,1,n) / CONM.M(1,1,n);
                ycg = -CONM.M(6,1,n) / CONM.M(1,1,n);
                xcg = CONM.M(6,2,n) / CONM.M(1,1,n);
                v = [xcg; ycg; zcg];
                
                CONM.Offset(n,:) = v';
                
                S = zeros(3,3);
                S(2,1) =  CONM.M(1,1,n) * v(3);
                S(3,1) = -CONM.M(1,1,n) * v(2);
                S(1,2) = -S(2,1);
                S(3,2) =  CONM.M(1,1,n) * v(1);
                S(1,3) = -S(3,1);
                S(2,3) = -S(3,2);
                
                CONM.M(4:6, 1:3, n) = S;
                CONM.M(1:3, 4:6, n) = S';
                CONM.M(4:6, 4:6, n) = CONM1.M(4:6,4:6,i) - CONM1.M(1,1,i) * crossm(CONM.Offset(n,:)) * ...
                    crossm(CONM.Offset(n,:));
                
            end
            
        end
        
        % CONM2 struct
        i = 0;
        for n = (ncom1+1) : mtot
            
            i = i+1;
            j = find(CONM2.Node(i) == NODE.ID);
            
            if isempty(j)
                
                error('Unable to find node %d in Node database for CONM2 %d.', CONM2.Node(i), CONM2.ID(i));
            end
            
            CONM.Node(n) = j; % set NODE index for rapid accessing
            
            if CONM2.CID(n) == 0
                
                CONM.Offset(n,:) = CONM2.Offset(i, :);
                CONM.M(:,:,n) = zeros(6,6);
                CONM.M(1:3,1:3,n) = CONM2.M(1:3,1:3,i);
                % set static moment
                S = zeros(3,3);
                S(2,1) =  CONM.M(1,1,n) * CONM2.Offset(i, 3);
                S(3,1) = -CONM.M(1,1,n) * CONM2.Offset(i, 2);
                S(1,2) = -S(2,1);
                S(3,2) =  CONM.M(1,1,n) * CONM2.Offset(i, 1);
                S(1,3) = -S(3,1);
                S(2,3) = -S(3,2);
                
                CONM.M(4:6, 1:3, n) = S;
                CONM.M(1:3, 4:6, n) = S';
                % get Inertia in a reference frame with origin in the NODE and parallel
                % to the basic reference frame
                CONM.M(4:6, 4:6, n) = CONM2.M(4:6,4:6,i) - CONM2.M(1,1,i) .* ...
                    (crossm(CONM2.Offset(i, 1:3)) * crossm(CONM2.Offset(i, 1:3)));
                
            end
            
            if CONM2.CID(n) > 0
                
                j = find(CONM2.CID(i) == COORD.ID);
                
                if isempty(j)
                    
                    error('Unable to find reference frame %d in COORD database for CONM2 %d.', ...
                        CONM2.CID(i), CONM2.ID(i));
                    
                end
                % set offset in the global reference frame
                CONM2.Offset(i, 1:3) = (COORD.R(:,:,j) * CONM2.Offset(i, 1:3)')';
                CONM.Offset(n,:) = CONM2.Offset(i, :);
                CONM.M(:,:,n) = zeros(6,6);
                CONM.M(1:3,1:3,n) = CONM2.M(1:3,1:3,i);
                % set static moment
                S = zeros(3,3);
                S(2,1) =  CONM.M(1,1,n) * CONM2.Offset(i, 3);
                S(3,1) = -CONM.M(1,1,n) * CONM2.Offset(i, 2);
                S(1,2) = -S(2,1);
                S(3,2) =  CONM.M(1,1,n) * CONM2.Offset(i, 1);
                S(1,3) = -S(3,1);
                S(2,3) = -S(3,2);
                
                CONM.M(4:6, 1:3, n) = S;
                CONM.M(1:3, 4:6, n) = S';
                % get Inertia in a reference frame with origin in the NODE and parallel to the basic reference frame
                RJ = COORD.R(:,:,j) * sqrt(CONM2.M(4:6,4:6,i));
                CONM.M(4:6, 4:6, n) = RJ * RJ' - ...
                    CONM2.M(1,1,i) .* (crossm(CONM2.Offset(i, 1:3)) * crossm(CONM2.Offset(i, 1:3)));
                
            end
            
            if CONM2.CID(n) < 0
                
                % set offset in the global reference frame as the difference between the CG and the NODE
                CONM2.Offset(i, 1:3) = CONM2.Offset(i, 1:3) - NODE.Coord(CONM.Node(n),1:3);
                CONM.Offset(n,:) = CONM2.Offset(i, 1:3);
                CONM.M(:,:,n) = zeros(6,6);
                CONM.M(1:3,1:3,n) = CONM2.M(1:3,1:3,i);
                % set static moment
                S = zeros(3,3);
                S(2,1) =  CONM.M(1,1,n) * CONM2.Offset(i, 3);
                S(3,1) = -CONM.M(1,1,n) * CONM2.Offset(i, 2);
                S(1,2) = -S(2,1);
                S(3,2) =  CONM.M(1,1,n) * CONM2.Offset(i, 1);
                S(1,3) = -S(3,1);
                S(2,3) = -S(3,2);
                
                CONM.M(4:6, 1:3, n) = S;
                CONM.M(1:3, 4:6, n) = S';
                % get Inertia in a reference frame with origin in the NODE
                % and parallel to the basic reference frame
                CONM.M(4:6, 4:6, n) = CONM2.M(4:6,4:6,i) - CONM2.M(1,1,i) .* ...
                    (crossm(CONM2.Offset(i, 1:3)) * crossm(CONM2.Offset(i, 1:3)));
            end
            % constrain matrix symmetry
            %    CONM.M(:, :, n) = (CONM.M(:, :, n) + CONM.M(:, :, n)') ./ 2.0;
            
        end
        
        fprintf(fid, 'done.');
    end
    %*******************************************************************************
    % MODEL dofs. Discard nodes not having mass, stiffness or RBE2
    fprintf(fid, '\n\tSetting Model dofs...');
    NODE.Index = int32(zeros(1,ngrid));
    index = [];
    for n = 1:ncbar
      index = [index, BAR.Conn(n,:)];
    end
    NODE.Index(index) = int32(1);
    index = [];
    for n = 1:ncbeam
      index = [index, BEAM.Conn(n,:)];
    end
    NODE.Index(index) = int32(1);
    index = [];
    for n = 1:mtot
      index = [index, CONM.Node(n)];
    end
    NODE.Index(index) = int32(1);
    index = [];
    for n=1:ncelas
      index = [index, find(NODE.ID == CELAS.Node(n,1))];
      index = [index, find(NODE.ID == CELAS.Node(n,2))];
    end
    NODE.Index(index) = int32(1);
    index = [];
    for n=1:nrbe2
      index = [index, find(NODE.ID == RBE2.IDM(n))];
      for k=1:length(RBE2.IDS(n).data)
        index = [index, find(NODE.ID == RBE2.IDS(n).data(k))];
      end
    end
    NODE.Index(index) = int32(1);
%
    index = sort(find(NODE.Index));
    i = [1:length(index)];
    %if length(index) ~= ngrid
    %	warning('The number of given nodes is greater than the number of structural nodes.');
    %end
    NODE.Index(index) = int32(i);
    % set reference frame for each node in the basic reference frame
    NODE.R = zeros(3, 3, ngrid);
    for j=1:ngrid
      NODE.R(:,:,j) = eye(3);
    end
   %
   % check master aero nodes
    for n=1:nrbe0
        
        if NODE.Index(RBE0.Master(n)) == 0
            
            error('Wrong master node %d for aero set %d: node is not master.', NODE.ID(RBE0.Master(n)), RBE0.ID(n));
            
        end
        
        index = NODE.Index(RBE0.Node(n).data);
        
        i = find(index);
        if ~isempty(i)
            
            error('Wrong slave node %d for aero set %d.', NODE.ID(RBE0.Node(n).data(i(1))), RBE0.ID(n));
            
        end
        
    end
    
    fprintf(fid, 'done.');
    
    % set GRDPNT
    
    if PARAM.GRDPNT ~= 0
        
        j = find(PARAM.GRDPNT == NODE.ID);
        if isempty(j)
            error('Unable to find reference Grid point %d for the weight generator.', PARAM.GRDPNT);
        else
            PARAM.GRDPNT = int32(j);
        end
    else
        PARAM.GRDPNT = int32(0);
    end

    %*******************************************************************************
    % assembly CONM and element to determine actual CG coords and general Weight and Balance
    
    % set lumped mass CG
    [MASS.CG, MASS.MCG, MASS.MRP] = ...
        wb_set_conm_mass(mtot, NODE.Index, NODE.Coord, NODE.R, PARAM.GRDPNT, CONM);
    % set bar mass CG
    [MASS.CG, MASS.MCG, MASS.MRP] =...
        wb_add_bar_mass(ncbar, NODE.Coord, NODE.R, MASS.CG, PARAM.GRDPNT, MASS.MCG, MASS.MRP, BAR);
    % set beam mass CG
    [MASS.CG, MASS.MCG, MASS.MRP] =...
        wb_add_bar_mass(ncbeam, NODE.Coord, NODE.R, MASS.CG, PARAM.GRDPNT, MASS.MCG, MASS.MRP, BEAM);
    % get principal axes
    [MASS.MCG_pa, MASS.R_pa] = wb_principal_axis(MASS.MCG);
    
    %*******************************************************************************
    % set NODE constrained dof
    NODE.DOF = int32(zeros(ngrid, 6));
    % apply SPC set if required by the user
    if PARAM.SPC
        
        count = 0;
        fprintf(fid, '\n\tSetting Nodal contraints...');
        spcindex = find( SPC.ID == PARAM.SPC);
        
        if isempty(spcindex)
            error('Unable to find SPC set %d as required.', PARAM.SPC);
        else
            
            for i = 1:length(spcindex)
                
                NODE.DOF(SPC.Nodes(i).list, SPC.DOF(i).list) = int32(1);
                
            end
            set = [1:6];
            ndof = 0;
            % count dof
            for n=1:ngrid
                
                if NODE.Index(n) % structural node
                    
                    index = find(NODE.DOF(n, :));
                    index = setdiff(set, index);
                    NODE.DOF(n, :) = int32(zeros(1,6));
                    NODE.DOF(n, index) = int32([ndof+1 : ndof+length(index)]);
                    ndof = ndof + length(index);
                    
                end
            end
            
        end %end check
        
        fprintf(fid,'done.');
    else
        
        set = [1:6];
        ndof = 0;
        % count dof
        for n=1:ngrid
            
            if NODE.Index(n) % structural node
                
                NODE.DOF(n, :) = int32(zeros(1,6));
                NODE.DOF(n, :) = int32([ndof+1 : ndof+6]);
                ndof = ndof + 6;
                
            end
        end
        
    end % end PARAM
    if ~isempty(PARAM.EIG_FILE)
      NODE.Index = [1:ngrid];
      set = [1:6];
      ndof = 0;
      % count dof
      for n=1:ngrid
        NODE.DOF(n, :) = int32(zeros(1,6));
        NODE.DOF(n, :) = int32([ndof+1 : ndof+6]);
        ndof = ndof + 6;
      end
    end
    %*******************************************************************************
    if ncelas>0
        fprintf(fid, '\n\tSorting Celas database...');
        CelasID = sort(CELAS.ID);
        if ~isempty(find(diff(CelasID)==0,1))
            error(fid, '\n\t: duplicated Celas.');
        end
        for i = 1 : ncelas
            if CELAS.Node(i,1) == CELAS.Node(i,2)
                error('\n\tCelas grids must be different %d. ',CELAS.ID(i));
            end
            CelasDOF1 = zeros(1,6);
            CelasDOF2 = zeros(1,6);
            for j= 1: length(CELAS.DOF(i,1).data)
                CelasDOF1(j) = str2double(CELAS.DOF(i,1).data(j));
            end
            for j= 1: length(CELAS.DOF(i,2).data)
                CelasDOF2(j) = str2double(CELAS.DOF(i,2).data(j));
            end
            CELAS.DOF(i,1).data = NODE.DOF(NODE.ID == CELAS.Node(i,1),CelasDOF1(CelasDOF1~=0));
            CELAS.DOF(i,2).data = NODE.DOF(NODE.ID == CELAS.Node(i,2),CelasDOF2(CelasDOF2~=0));
            
        end
        
        fprintf(fid, 'done.');
    end
%   check landing gear nodes
    for i=1:length(PARAM.LANDG)
      j = find(NODE.ID == PARAM.LANDG(i));
      if isempty(j)
        error('Unable to find node %d for LANDG.', PARAM.LANDG(i));
      else
        PARAM.LANDG(i) = j; % use node index for rapid accessing
      end
    end
    % check SUPORT if any
    if (~isempty(PARAM.SUPORT))
        %
        for n=1:size(PARAM.SUPORT,1)
            ne = find(PARAM.SUPORT(n,1) == NODE.ID);
            if isempty(ne)
                error('Unable to find node %d required in SUPORT card.', PARAM.SUPORT(n,1));
            end
            sdof = num2str(PARAM.SUPORT(n,2));
            for j = 1:length(sdof)
                if (NODE.DOF(ne, str2num(sdof(j))) == 0)
                    error('Required SUPORT for constrained degree of freedom.');
                end
            end
        end
        %
    end
    if nrbe2 >0
        [RBE2,NODE,nrbe2,ndof] = checkRBE2(fid,RBE2,NODE,PARAM,BAR,BEAM,CELAS,ngrid,SPC,ndof);
    else
        NODE.DOF2 = NODE.DOF;
    end
    %*******************************************************************************
    % check bar and beam ID and properties
    v = intersect(BAR.ID, BEAM.ID);
    
    if (~isempty(v))
        
        error('Bar and Beam entries have duplicated labels: %d ', v);
        
    end
    
    v = intersect(PBAR.ID, PBEAM.ID);
    
    if (~isempty(v))
        
        error('Bar and Beam entries have duplicated property labels: %d ', v);
        
    end
    %*******************************************************************************
    % check eigenvalues parameters (if any)
    
    if (neig >0)
        fprintf(fid, '\n\tSetting Eigenvalues solver...');
        
        switch(PARAM.MSCALE)
            
            case 'MASS'
                
            case 'MAX'
                
            case 'POINT'
                
                index = find(PARAM.MG == NODE.ID);
                if (isempty(index))
                    error('Unable to find node %d for eigenvalue scaled as given in EIGR card.', PARAM.MG);
                end
                n = NODE.DOF(index, PARAM.MC);
                if (n == 0)
                    error('Node %d required in EIGR card has no %d dof.', PARAM.MG, PARAM.MC);
                end
                PARAM.MG = index;
                
            otherwise
                fprintf(fid, '\n\t\t### Warning: wrong normalization method given. Set to mass unity.');
                PARAM.MSCALE = 'MASS';
        end
        
    end
    %*******************************************************************************
%   check body aero
    if nbaero > 0        
        fprintf(fid, '\n\tSetting Body database...');
        for n=1:nbaero
            if BAERO.CP(n) ~= 0
                j = find(COORD.ID == BAERO.CP(n));
                if isempty(j)
                    error('Unable to find reference frame %d for BAERO entry %d.', BAERO.CP(n), BAERO.ID(n));
                else
                    BAERO.CP(n) = j;
                end
            end
            if BAERO.SET(n)<=0
              fprintf(fid, '\n\t\t### Warning: no interpolation set given for body %d.', BAERO.ID(n));
            else
              if ninterp>0
                j = find(CAERO.Set.ID ==BAERO.SET(n));
                if ~isempty(j)
                  BAERO.SET(n) = j;
                else
                  error(['Unable to find SET ', num2str(BAERO.SET(n)),' for body ', num2str(BAERO.ID(n))]);
                end
              end
            end
        end
        CAERO.body = body_lattice_setup(BAERO, COORD);
%       Add fictious CT CAERO for vertical tailplane
        if (ncaero)
          [CAERO, nadd] = add_vert_patch(fid, CAERO);
          ncaero = ncaero+nadd;
        end
%       Check interfering CAERO
        fprintf(fid,'\n\t\tDetecting body interfering VLM patches...');
        CAERO.body.geo.CAERO_INT = interf_caero(CAERO);
        for n=1:nbaero
            for k=1:length(CAERO.body.geo.CAERO_INT{n})
              fprintf(fid, '\n\t\t\t### Warning: CAERO %d will be considered as interfering patch with body %d.', ...
                           CAERO.ID(CAERO.body.geo.CAERO_INT{n}(k)), CAERO.body.ID(n));
            end
        end
        fprintf(fid,'\n\t\tdone.');
%
        fprintf(fid, '\n\tdone.');
    else
      CAERO.body = BAERO;
    end
%
    % sort CAERO1 struct
    if ncaero > 0
        
        fprintf(fid, '\n\tSetting Aero database...');
        
        CAERO.geo.CG = MASS.CG;
        
        if PARAM.GRDPNT ~= 0
            
            CAERO.geo.ref_point = NODE.Coord(PARAM.GRDPNT, 1:3);
            
        else
            
            CAERO.geo.ref_point = zeros(1,3);
            
        end
        
        for n=1:ncaero
            
            if CAERO.CP(n) ~= 0
                
                j = find(COORD.ID == CAERO.CP(n));
                if isempty(j)
                    error('Unable to find reference frame %d for CAERO entry %d.', CAERO.CP(n), CAERO.ID(n));
                else
                    CAERO.CP(n) = j;
                end
                
                v = COORD.R(:,:,j) * [CAERO.geo.startx(n); CAERO.geo.starty(n); CAERO.geo.startz(n)];
                
                CAERO.geo.startx(n) = v(1) + COORD.Origin(j,1);
                CAERO.geo.starty(n) = v(2) + COORD.Origin(j,2);
                CAERO.geo.startz(n) = v(3) + COORD.Origin(j,3);
                
            end
            
        end
        
        CAERO.geo.nwing = ncaero;
        CAERO.geo.symetric = zeros(ncaero,1)';
        CAERO.geo.flap_vector = zeros(ncaero,max(CAERO.geo.nelem));
        
        fprintf(fid,'done.');
        
    end
    %*******************************************************************************
    % set DAMP
    if (PARAM.SDAMP && ndamp>0)
      index = find(DAMP.ID == PARAM.SDAMP);
      if (isempty(index))
        error(['Unknown TABDMP1 table ID ', num2str(PARAM.SDAMP),' required in SDAMPING']);
      end
      PARAM.SDAMP = index;
    end
    %*******************************************************************************
    % set counters
    fprintf(fid, '\n\tSetting model counters...');
    INFO.ndamp = ndamp;
    INFO.npbarsm = npbarsm;
    INFO.naero = naero;
    INFO.naeros = naeros;
    INFO.neig = neig;
    INFO.ndof = ndof;
    INFO.ndof2 = max(max(NODE.DOF2));
    INFO.ncord = ncord;
    INFO.ngrid = ngrid;
    INFO.nmat = nmat;
    INFO.nbar = ncbar;
    INFO.ninclude = ninclude;
    INFO.npbar = npbar;
    INFO.nbeam = ncbeam;
    INFO.npbeam = npbeam;
    INFO.ncaero = ncaero;
    INFO.nbaero = nbaero;
    INFO.nconm = mtot;
    INFO.nf = nforce;
    INFO.nm = nmom;
    INFO.ndload = ndload;
    INFO.cc_nspc = cc_nspc;
    INFO.cc_trim = cc_trim;
    INFO.nspc = nspc;
    INFO.ninterp = ninterp;
    INFO.nset = nset;
    INFO.nrbe0 = nrbe0;
    INFO.nrbe2 = nrbe2;
    INFO.nflw = nflw;
    INFO.nlink = nlink;
    INFO.nmkaero = nmkaero;
    INFO.amesh_av_vlm = false;
    INFO.amesh_av_dlm = false;
    INFO.ntrim = ntrim;
    INFO.ntrimext = ntrimext;
    INFO.nderext = nderext;
    INFO.ndesvar = ndesvar;
    INFO.ndresp = ndresp;
    INFO.ndlink = ndlink;
    INFO.ndvprel = ndvprel;
    INFO.nstabresp = nstabresp;
    INFO.ntrimresp = ntrimresp;
    INFO.spline_type = 0;
    fprintf(fid, 'done.');
    %*******************************************************************************
    % Set available extra solvers
    PARAM.MSOL = set_available_sol(fid, INFO, PARAM);
    %*******************************************************************************
    if (naeros && naero)
        % check for consistent reference values for correct dynamic/rigid derivatives
        if (BREF_DLM~=BREF_VLM)
            error('Rerefence span for VLM and DLM is different.')
        end
        if (SREF_DLM~=SREF_VLM)
            error('Rerefence surface for VLM and DLM is different.')
        end
        if (CREF_DLM~=CREF_VLM)
            error('Rerefence chord for VLM and DLM is different.')
        end
        if (SIMXY_DLM ~= SIMXY_VLM)
            error('Simmetry for XY plane for VLM and DLM is different.')
        end
        if (SIMXZ_DLM ~= SIMXZ_VLM)
            error('Simmetry for XZ plane for VLM and DLM is different.')
        end
    end
    %
    if (~isempty(CONTROL_NAME))
        if (length(unique(CONTROL_NAME))~=CAERO.geo.nc)
            error('Control surface duplicated name given.');
        end
    end
    %
    SOL_INDEX = find(PARAM.MSOL);
    %
    check_steady = false;
    check_dynamic = false;
    
    for (NSOL=1:length(SOL_INDEX))
        
        switch PARAM.MSOL(SOL_INDEX(NSOL))
            % STEADY AEROELASTIC SOLUTION LINEAR/NONLINEAR
            case {144, 644, 700}
                
                if (~check_steady)
                    
                    if (ncaero==0 && nbaero==0)
                        error('No CAERO1 or CAEROB card given.');
                    end
                    
                    if ~naeros
                        error('No AEROS card given.');
                    end
                    
                    if CAERO.Trim.CID
                        
                        i = find(CAERO.Trim.CID == COORD.ID);
                        if isempty(i)
                            error('Unable to find AEROS reference frame %d.', CAERO.Trim.CID);
                        end
                        
                        CAERO.Trim.CID = i; %substitute coord ID with its index
                        
                    end
                    
                    if cc_trim
                        if ntrim % for steady calculations TRIM card gives at least aerodynamic conditions
                            % TRIM card is used to set all flight mechanics variables and control surface rotations
                            % check for duplicated labels
                            [index, i] = unique(CAERO.Trim.ID);
                            if (length(i) ~= ntrim)
                                n = [1 : ntrim];
                                index = setdiff(n, i);
                                error('Trim set entries have duplicated labels: %d', CAERO.Trim.ID(index(1)));
                            end
                        else
                            error('No TRIM card given.');
                        end
                    else
                        if PARAM.SOL ~= 146
                            error('No TRIM= card given. Unable to select flight condition.');
                        end
                    end
                    
                    CAERO.Trim.Param = TRIM_PARAM;
                    CAERO.Trim.Value = TRIM_VALUE;
                    % overwrite Tornado default values if required in AERO / AEROS cards
                    CAERO.state.SIMXZ = SIMXZ_VLM;
                    CAERO.state.SIMXY = SIMXY_VLM;
                    if CREF_VLM
                        CAERO.ref.C_mgc = CREF_VLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference chord set to unity.');
                        CAERO.ref.C_mgc = 1.0;
                    end

                    if BREF_VLM
                        CAERO.ref.b_ref = BREF_VLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference span set to unity.');
                        CAERO.ref.b_ref = 1.0;
                        BREF_VLM = 1.0;
                    end
                    
                    if SREF_VLM
                        CAERO.ref.S_ref = SREF_VLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference surface set to unity.');
                        CAERO.ref.S_ref = 1.0;
                        SREF_VLM = 1.0;
                    end
                    CAERO.ref.C_mac = 0;
                    %    CAERO.ref.mac_pos = [];
                    if (ncaero)
                      [CAERO.lattice_vlm, CAERO.ref] = vlm_lattice_setup(PARAM.FID, CAERO.geo, ...
                        CAERO.state, CAERO.ref);
                      CAERO.lattice_vlm.Control.Name = CONTROL_NAME;
                      INFO.amesh_av_vlm = true;
                      dummy_lattice = CAERO.lattice_vlm;
                      fprintf(fid, '\n\tVortex Lattice grid created.');
                      % check for COORD for AESURF
                      for n=1:naesurf
                        id = CAERO.AESURF.AERID(n);
                        index = find(CAERO.ID == id);
                        index = find(CAERO.lattice_vlm.Control.Patch == index);
                        if CAERO.AESURF.CID(n)>0
                          c = find(COORD.ID == CAERO.AESURF.CID(n));   
                          CAERO.lattice_vlm.Control.Hinge(index,1,1:3) = COORD.Origin(c,:);
                          P2 = squeeze(CAERO.lattice_vlm.Control.Hinge(index,2,1:3)) + COORD.R(:,2,c);
                          CAERO.lattice_vlm.Control.Hinge(index,2,1:3) = P2;
                        end
                      end
                    end
                    check_steady = true;
                end
                %*******************************************************************************
                % FLUTTER SOLUTION
            case {145, 701}
                
                if (~check_dynamic)
                    
                    if ~ncaero
                        error('No CAERO1 card given. Unable to create DLM mesh.');
                    end
                    
                    if ~naero
                        error('No AERO card given.');
                    end
                    
                    % clean reduced order model data
                    CAERO.state.Mach_qhh = unique(CAERO.state.Mach_qhh);
                    index = find(CAERO.state.Mach_qhh > 0);
                    CAERO.state.Mach_qhh = CAERO.state.Mach_qhh(index);
                    
                    CAERO.state.Kfreq = unique(CAERO.state.Kfreq);
                    index = find(CAERO.state.Kfreq > 0);
                    CAERO.state.Kfreq = CAERO.state.Kfreq(index);
                    if CREF_DLM
                        CAERO.ref.C_mgc = CREF_DLM/2;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference chord for DLM set to unity.');
                        CAERO.ref.C_mgc = 1.0;
                        CREF_DLM = 1.0;
                    end
                    %
                    CAERO.state.SIMXZ = SIMXZ_DLM;
                    CAERO.state.SIMXY = SIMXY_DLM;
                    % set DLM mesh
                    % erase any twist given
                    if  (PARAM.DLM_KMAX)
                      [CAERO.geo, CAERO.Interp] = update_k_mesh(fid, CAERO, CAERO.ref.C_mgc, ...
                                                  PARAM.DLM_KMAX, PARAM.DLM_AR, PARAM.DLM_NP); 
                    end
                    [CAERO.lattice_dlm] = dlm_lattice_setup(PARAM.FID, CAERO.geo, CAERO.state.SIMXZ, CAERO.ref.C_mgc, CAERO.INT);
                    INFO.amesh_av_dlm = true;
                    CAERO.lattice_dlm.Control.Name = CONTROL_NAME;
                    for n=1:naesurf
                      id = CAERO.AESURF.AERID(n);
                      index = find(CAERO.ID == id);
                      index = find(CAERO.lattice_dlm.Control.Patch == index);
                      if CAERO.AESURF.CID(n) >0
                        c = find(COORD.ID == CAERO.AESURF.CID(n));   
                        CAERO.lattice_dlm.Control.Hinge(index,1,1:3) = COORD.Origin(c,:);
                        P2 = squeeze(CAERO.lattice_dlm.Control.Hinge(index,2,1:3)) + COORD.R(:,2,c);
                        CAERO.lattice_dlm.Control.Hinge(index,2,1:3) = P2;
                      end
                    end
                    %
                    % check gust X0 point                    
                    %
                    if (ngust)
                      X0min = min(CAERO.lattice_dlm.COLLOC(:,1).*CAERO.ref.C_mgc);
                      for n=1:ngust 
                        if GUST.X0(n)>X0min
                          error(['Gust input point X0 ',num2str(GUST.X0(n)),' is placed too far downstream. The minimum input point should be less than ',num2str(X0min),'.']);
                        end
                      end
                    end
                    %
                    dummy_lattice = CAERO.lattice_dlm;
                    % overwrite Tornado default values if required in AERO / AEROS cards
                    CAERO.ref.C_mac = CREF_DLM/2.0;
                    
                    if BREF_DLM
                        CAERO.ref.b_ref = BREF_DLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference span for DLM set to unity.');
                        CAERO.ref.b_ref = 1.0;
                        BREF_DLM = 1.0;
                    end
                    
                    if SREF_DLM
                        CAERO.ref.S_ref = SREF_DLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference surface for DLM set to unity.');
                        CAERO.ref.S_ref = 1.0;
                        SREF_DLM = 1.0;
                    end
                    
                    check_dynamic = true;
                end
                
            case {103,105}
                
                if (neig ==0)
                    error('No EIGR card given.');
                end
                
            case {101, 600}
                
                if (PARAM.LOAD == 0)
                    error('No LOAD set provided.');
                end
            case{150}
                
                if (~check_steady)
                    
                    
                    
                    if ~naeros
                        error('No AEROS card given.');
                    end
                    
                    if CAERO.Trim.CID
                        
                        i = find(CAERO.Trim.CID == COORD.ID);
                        if isempty(i)
                            error('Unable to find AEROS reference frame %d.', CAERO.Trim.CID);
                        end
                        
                        CAERO.Trim.CID = i; %substitute coord ID with its index
                        
                    end
                    
                    if cc_trim
                        if ntrim % for steady calculations TRIM card gives at least aerodynamic conditions
                            % TRIM card is used to set all flight mechanics variables and control surface rotations
                            % check for duplicated labels
                            [index, i] = unique(CAERO.Trim.ID);
                            if (length(i) ~= ntrim)
                                n = [1 : ntrim];
                                index = setdiff(n, i);
                                error('Trim set entries have duplicated labels: %d', CAERO.Trim.ID(index(1)));
                            end
                        else
                            error('No TRIM card given.');
                        end
                    else
                        error('No TRIM= card given. Unable to select flight condition.');
                    end
                    
                    CAERO.Trim.Param = TRIM_PARAM;
                    CAERO.Trim.Value = TRIM_VALUE;
                    % overwrite Tornado default values if required in AERO / AEROS cards
                    CAERO.state.SIMXZ = SIMXZ_VLM;
                    CAERO.state.SIMXY = SIMXY_VLM;
                    if CREF_VLM
                        CAERO.ref.C_mgc = CREF_VLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference chord for VLM set to unity.');
                        CAERO.ref.C_mgc = 1.0;
                        CREF_VLM = 1.0;
                    end

                    if BREF_VLM
                        CAERO.ref.b_ref = BREF_VLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference span for VLM set to unity.');
                        CAERO.ref.b_ref = 1.0;
                        BREF_VLM = 1.0;
                    end
                    
                    if SREF_VLM
                        CAERO.ref.S_ref = SREF_VLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference surface for VLM set to unity.');
                        CAERO.ref.S_ref = 1.0;
                        SREF_VLM = 1.0;
                    end
                    CAERO.ref.C_mac = 0;
                    %    CAERO.ref.mac_pos = [];
                    if (ncaero)
                      [CAERO.lattice_vlm, CAERO.ref] = vlm_lattice_setup(PARAM.FID, CAERO.geo, ...
                        CAERO.state, CAERO.ref);
                      CAERO.lattice_vlm.Control.Name = CONTROL_NAME;
                    
                      INFO.amesh_av_vlm = true;
                      dummy_lattice = CAERO.lattice_vlm;
                      fprintf(fid, '\n\tVortex Lattice grid created.');
                    end
                    check_steady = true;
                end
                
                if (~check_dynamic)
                    
                    if ~ncaero
                        error('No CAERO1 card given. Unable to create DLM mesh.');
                    end
                    
                    if ~naero
                        error('No AERO card given.');
                    end
                    
                    % clean reduced order model data
                    CAERO.state.Mach_qhh = unique(CAERO.state.Mach_qhh);
                    index = find(CAERO.state.Mach_qhh > 0);
                    CAERO.state.Mach_qhh = CAERO.state.Mach_qhh(index);
                    
                    CAERO.state.Kfreq = unique(CAERO.state.Kfreq);
                    index = find(CAERO.state.Kfreq > 0);
                    CAERO.state.Kfreq = CAERO.state.Kfreq(index);
                    
                    if CREF_DLM
                        CAERO.ref.C_mgc = CREF_DLM;
                    else
                        CAERO.ref.C_mgc = 1.0;
                        CREF_DLM = 1.0;
                    end
                    %
                    CAERO.state.SIMXZ = SIMXZ_DLM;
                    CAERO.state.SIMXY = SIMXY_DLM;
                    % set DLM mesh
                    % erase any twist given
                    if  (PARAM.DLM_KMAX)
                      [CAERO.geo, CAERO.Interp] = update_k_mesh(fid, CAERO, CREF_DLM/2.0, ...
                                                  PARAM.DLM_KMAX, PARAM.DLM_AR, PARAM.DLM_NP); 
                    end
                    [CAERO.lattice_dlm] = dlm_lattice_setup(PARAM.FID, CAERO.geo, CAERO.state.SIMXZ, CREF_DLM/2.0, CAERO.INT);
                    INFO.amesh_av_dlm = true;
                    CAERO.lattice_dlm.Control.Name = CONTROL_NAME;
                    
                    dummy_lattice = CAERO.lattice_dlm;
                    % overwrite Tornado default values if required in AERO / AEROS cards
                    CAERO.ref.C_mac = CREF_DLM/2.0;
                    
                    if BREF_DLM
                        CAERO.ref.b_ref = BREF_DLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference span for DLM set to unity.');
                        CAERO.ref.b_ref = 1.0;
                        BREF_DLM = 1.0;
                    end
                    
                    if SREF_DLM
                        CAERO.ref.S_ref = SREF_DLM;
                    else
                        fprintf(fid, '\n\t\t### Warning: reference surface for DLM set to unity.');
                        CAERO.ref.S_ref = 1.0;
                        SREF_DLM = 1.0;
                    end
                    
                    check_dynamic = true;
                end
                
            otherwise
                
                error('Unknown solver required (allowed values are 101, 103, 105, 144, 145, 600, 644, 701).');
                
        end
        
    end % solver loop
    
    
    %*******************************************************************************
    % sort interp database
    interp_type = unique(CAERO.Interp.Type);
    INFO.spline_type = interp_type;
    if (length(INFO.spline_type)>1)
      error('SPLINE types cannot be mixed up. Use only SPLINE1, SPLINE2 or SPLINE3 cards.');
    end
    
    % SPLINE2 and SPLINE3 interpolation matrices need a row vector (see
    % assembly_colloc_interp_mat, assembly_node_interp_mat etc...)
    if ~isempty(interp_type) && (interp_type == 2 || interp_type == 3)
        for n = 1:nset
            CAERO.Set.Node(n).data = CAERO.Set.Node(n).data';
        end
    end
    
    % if no aerodynamic mesh is available, skip interpolation cards check
    if (INFO.amesh_av_dlm || INFO.amesh_av_vlm)
        
        if ninterp > 0
            
            fprintf(fid, '\n\tSetting Aero interpolation database...');
            
            [index, i] = unique(CAERO.Interp.ID);
            
            if (length(i) ~= ninterp)
                
                n = [1 : nset];
                index = setdiff(n, i);
                error('Interpolation set entries have duplicated labels: %d', CAERO.Interp.ID(index(1)));
                
            end
            
            if ~nset
                
                error('No structural set given for interface procedure.');
                
            end
            % allocate data
            for n=1:ncaero
                % number of elements
                ne =  dummy_lattice.DOF(n, CAERO.geo.nelem(n), 2) - dummy_lattice.DOF(n, 1, 1) +1;
                CAERO.IS(n).data = zeros(ne, 1);
                
            end
            
            for n=1:ninterp
                
                i = find(CAERO.ID == CAERO.Interp.Patch(n));
                m = find(CAERO.Set.ID == CAERO.Interp.Set(n));
                
                if isempty(m)
                    
                    error('Unable to find structural set %d for the interpolation set %d.',...
                        CAERO.Interp.Set(n), CAERO.Interp.ID(n));
                    
                else
                    
                    CAERO.Interp.Set(n)	= m; % substitute with set index for rapid accessing
                    
                    if isempty(i)
                        
                        error('Unable to find aero patch %d for the interpolation set %d.',...
                            CAERO.Interp.Patch(n), CAERO.Interp.ID(n));
                        
                    else
                        
                        % check panel index
                        
                        if CAERO.Interp.Index(n, 2) > (1 + dummy_lattice.DOF(i, CAERO.geo.nelem(i), 2) - dummy_lattice.DOF(i, 1, 1))
                            
                            error('Panel index in interpolation set %d exceeds maximum panel indexing for patch %d.', ...
                                CAERO.Interp.ID(n), CAERO.ID(i));
                            
                        end
                        
                        CAERO.IS(i).data([CAERO.Interp.Index(n, 1) : CAERO.Interp.Index(n, 2)]) = n; % substitute with interp set index
                        
                    end
                    
                end
                
            end
            
            for n=1:ninterp
                i = find(CAERO.ID == CAERO.Interp.Patch(n));
                CAERO.Interp.Patch(n) = i; % substitute with patch index for rapid accessing
            end
            for n=1:ninterp
              if (isequal(CAERO.Interp.Type(n),1))
                if (CAERO.Interp.Param(n, 1)>0)
                  i1 = find(NODE.ID == CAERO.Interp.Param(n, 1));
                  if (isempty(i1))
                    error(['Node ', num2str(CAERO.Interp.Param(n, 1)),' in SPLINE1 ',num2str(CAERO.Interp.ID(n)),' does not exist.']);
                  end
                  i = find( CAERO.Set.Node(CAERO.Interp.Set(n)).data == i1);
                  if isempty(i)
                    error('Unable to find NODE %d in the interpolation set %d for SPLINE1 %d.',...
                        CAERO.Interp.Param(n, 1), CAERO.Set.ID(CAERO.Interp.Set(n)), CAERO.Interp.ID(n));
                  end
                  CAERO.Interp.Param(n, 1) = i;
                else
                  %fprintf(fid, '\n\t\t### Warning: origin node not defined in SPLINE1 %d.\n\t\t    First node in SET1 %d will be used.', ...
                  %                    CAERO.Interp.ID(n), CAERO.Set.ID(CAERO.Interp.Set(n)));
                  CAERO.Interp.Param(n, 1) = 1;
                end
                if (CAERO.Interp.Param(n, 2)>0)
                  i2 = find(NODE.ID == CAERO.Interp.Param(n, 2));
                  if (isempty(i2))
                    error(['Node ', num2str(CAERO.Interp.Param(n, 2)),' in SPLINE1 ',num2str(CAERO.Interp.ID(n)),' does not exist.']);
                  end
                  i = find( CAERO.Set.Node(CAERO.Interp.Set(n)).data == i2);
                  if isempty(i)
                    error('Unable to find NODE %d in the interpolation set %d for SPLINE1 %d.',...
                        CAERO.Interp.Param(n, 1), CAERO.Set.ID(CAERO.Interp.Set(n)), CAERO.Interp.ID(n));
                  end
                  CAERO.Interp.Param(n, 2) = i;
                else
                  %fprintf(fid, '\n\t\t### Warning: second node for y-axis not defined in SPLINE1 %d.\n\t\t    Last node in SET1 %d will be used.', ...
                  %                    CAERO.Interp.ID(n), CAERO.Set.ID(CAERO.Interp.Set(n)));
                  CAERO.Interp.Param(n, 2) = length(CAERO.Set.Node(CAERO.Interp.Set(n)).data);
                end
                if (CAERO.Interp.Param(n, 2) == CAERO.Interp.Param(n, 1))
                  error(['Node ', num2str(CAERO.Interp.Param(n, 1)),' is used twice to define SPLINE1 ',num2str(CAERO.Interp.ID(n)),' reference frame.']);
                end
              end
            end

            % check if any panel is not interfaced
            
            for n=1:ncaero
                
                ne = length(CAERO.IS(n).data);
                index = find(CAERO.IS(n).data); % get interfaced panels
                index = setdiff([1:ne], index); % get not interfaced panels
                il = length(index);
%                dof = [dummy_lattice.DOF(CAERO.Interp.Patch(n), 1, 1) : dummy_lattice.DOF(CAERO.Interp.Patch(n), CAERO.geo.nelem(n), 2)];
                if il ~= 0
                    fprintf(fid, '\n\t\t### Warning: patch %d has %d unsplined panels.', CAERO.ID(n), il);
                end
                
                
            end
            
            fprintf(fid,'done.');
            
            
        end
        
    end
    
end
%
% DESOPT
%
if ndesvar
  desid = unique(DESOPT.DESVAR.ID);
  if (length(desid) ~= length(DESOPT.DESVAR.ID))
    for i=1:length(DESOPT.DESVAR.ID)
      if find(DESOPT.DESVAR.ID(i)>1)
        error(['Duplicate DESVAR ', DESOPT.DESVAR.ID(i)]);
      end
    end
  end
end
%
if ndvprel
  pindex = find(DESOPT.DVPREL.Type == 1);
  if ~isempty(pindex)
    PID = DESOPT.DVPREL.PID(pindex);
    [v, i] = intersect(PBAR.ID, PID);
    if length(i) ~= length(PID)
        sindex = setdiff([1:length(PID)], i);
        error('Unable to determine DVPREL property %d in set %d.', PID(sindex(1)), DESOPT.DVPREL.ID(pindex(sindex(1))));
    end
    PID = i;
    DESOPT.DVPREL.PID(pindex) = PID;
  end
%
  DESOPT = init_desopt(DESOPT, INFO, BAR, PBAR, MAT, NODE);
% update ndesvar counter due to DLINK
  INFO.ndesvar = length(DESOPT.DESVAR.ID);
%
end
%
%
% TRIMEXT SPLINE
%
for n=1:ntrimext
  index = find(CAERO.Set.ID == CAERO.Trim.Ext.Set(n)); 
  if isempty(index)
    error('SET1 %d required in TRIMEXT ID %d.', CAERO.Trim.Ext.Set(n), CAERO.Trim.Ext.ID(n));
  end
  CAERO.Trim.Ext.Set(n) = index;
  if (CAERO.Trim.Ext.Index(n,1)>0)
    i1 = find(NODE.ID == CAERO.Trim.Ext.Index(n,1));
    if (isempty(i1))
      error(['Node ', num2str(CAERO.Trim.Ext.Index(n,1)),' in TRIMEXT ',num2str(CAERO.Trim.Ext.ID(n)),' does not exist.']);
    end
    i = find( CAERO.Set.Node(CAERO.Trim.Ext.Set(n)).data == i1);
    if isempty(i)
      error(['Unable to find NODE ', num2str(CAERO.Trim.Ext.Index(n,1)),' in the interpolation set ', num2str(CAERO.Set.ID(CAERO.Trim.Ext.Set(n))),'  for TRIMEXT ', num2str(CAERO.Trim.Ext.ID(n))]);
    end
    CAERO.Trim.Ext.Index(n,1) = i;
  else
    CAERO.Trim.Ext.Index(n,1) = 1;
  end
  if (CAERO.Trim.Ext.Index(n,2)>0)
    i2 = find(NODE.ID == CAERO.Trim.Ext.Index(n,2));
    if (isempty(i2))
      error(['Node ', num2str(CAERO.Trim.Ext.Index(n,2)),' in TRIMEXT ',num2str(CAERO.Trim.Ext.ID(n)),' does not exist.']);
    end
    i = find( CAERO.Set.Node(CAERO.Trim.Ext.Set(n)).data == i2);
    if isempty(i)
      error(['Unable to find NODE ', num2str(CAERO.Trim.Ext.Index(n,2)),' in the interpolation set ', num2str(CAERO.Set.ID(CAERO.Trim.Ext.Set(n))),'  for TRIMEXT ', num2str(CAERO.Trim.Ext.ID(n))]);
    end
    CAERO.Trim.Ext.Index(n,2) = i;
  else
    CAERO.Trim.Ext.Index(n,2) = length(CAERO.Set.Node(CAERO.Trim.Ext.Set(n)).data);;
  end
% check AERO ID (>=100)
  [~, ia, ib] = intersect(CAERO.ID,  CAERO.Trim.Ext.CAERO(n).data(CAERO.Trim.Ext.CAERO(n).data>=100));     
  if length(ib)~=length(CAERO.Trim.Ext.CAERO(n).data(CAERO.Trim.Ext.CAERO(n).data>=100))
    error(['TRIMEXT ', num2str(CAERO.Trim.Ext.ID(n)),' required CAERO which does not exist.']);
  end
% check BAERO ID (<100)
  if nbaero
    [~, ia, ib] = intersect(BAERO.ID, CAERO.Trim.Ext.CAERO(n).data(CAERO.Trim.Ext.CAERO(n).data<100));     
    if length(ib)~=length(CAERO.Trim.Ext.CAERO(n).data(CAERO.Trim.Ext.CAERO(n).data<100))
      error(['TRIMEXT ', num2str(CAERO.Trim.Ext.ID(n)),' required BAERO which does not exist.']);
    end
  else
    ia = find(CAERO.Trim.Ext.CAERO(n).data<100);
    if ~isempty(ia)
      error(['TRIMEXT ', num2str(CAERO.Trim.Ext.ID(n)),' required BAERO which does not exist.']);
    end
  end
%
  if CAERO.Trim.Ext.Coord(n)
    index = find(COORD.ID == CAERO.Trim.Ext.Coord(n));
    if ~isempty(index)
      CAERO.Trim.Ext.Coord(n) = index;
     else
      error(['Unable to find COORD ', num2str(CAERO.Trim.Ext.Coord(n)), ' in TRIMEXT card ', num2str(CAERO.Trim.Ext.ID(n)),'.']);
    end
  end
%
end
%
% check responce param
% store master surface labels
%
if INFO.amesh_av_dlm
  dummy_lattice = CAERO.lattice_dlm;
end
if INFO.amesh_av_vlm
  dummy_lattice = CAERO.lattice_vlm;
end
if INFO.amesh_av_vlm || INFO.amesh_av_dlm
  if isempty(CAERO.Trim.Link)
    masterSurf =unique(dummy_lattice.Control.Name);
  else
    masterSurf =[unique(CAERO.Trim.Link.Master), unique(setdiff(dummy_lattice.Control.Name,[CAERO.Trim.Link.Slave, CAERO.Trim.Link.Master]))];
  end
  CAERO.Trim.MasterSurf = masterSurf; 
end
%

if ~isempty(find(PARAM.MSOL==145,1))
    if ~isempty(PARAM.AEROFORCE)
        if ~ischar(PARAM.AEROFORCE)
            idSETf = find(SET.ID == PARAM.AEROFORCE);
            if isempty(idSETf)
                error('Check AEROFORCE card, requires SET that does not exist')
            else
                idAeroPannel = SET.Val(idSETf).data;
                if ~all(ismember(idAeroPannel,1:CAERO.lattice_dlm.np))
                    error('Check AEROFORCE card, forces required on aerodynamic panels that do not exist')
                else
                    PARAM.AEROFORCE = idAeroPannel;
                end
            end
        else
            PARAM.AEROFORCE = 1:CAERO.lattice_dlm.np;
        end
    end
    if ~isempty(PARAM.HINGEFORCE)
        if ~ischar(PARAM.HINGEFORCE)
            idSETf = find(SET.ID == PARAM.HINGEFORCE);
            if isempty(idSETf)
                error('Check HINGEFORCE card, requires SET that does not exist')
            else
                idHinge = SET.Val(idSETf).data;
                if isempty(CAERO.Trim.Link)
                    nslave = 0;
                else
                    nslave = length(unique(CAERO.Trim.Link.Slave));
                end
                if ~all(ismember(idHinge,1:CAERO.geo.nc -nslave ))
                    error('Check HINGEFORCE card, forces required on aerodynamic control that do not exist')
                else
                    PARAM.HINGEFORCE = idHinge;
                end
            end
        else
            PARAM.HINGEFORCE = 1:CAERO.geo.nc -length(unique(CAERO.Trim.Link.Slave));
        end
    end
elseif ~isempty(find(PARAM.MSOL==144,1))
    if ~isempty(PARAM.AEROFORCE)
        if ~ischar(PARAM.AEROFORCE)
            idSETf = find(SET.ID == PARAM.AEROFORCE);
            if isempty(idSETf)
                error('Check AEROFORCE card, requires SET that does not exist')
            else
                idAeroPannel = SET.Val(idSETf).data;
                if ~all(ismember(idAeroPannel,1:size(CAERO.lattice_vlm.N,1)))
                    error('Check AEROFORCE card, forces required on aerodynamic panels that do not exist')
                else
                    PARAM.AEROFORCE = idAeroPannel;
                end
            end
        else
            PARAM.AEROFORCE = 1:1:size(CAERO.lattice_vlm.N,1);
        end
    end
    if ~isempty(PARAM.HINGEFORCE)
        if ~ischar(PARAM.HINGEFORCE)
            idSETf = find(SET.ID == PARAM.HINGEFORCE);
            if isempty(idSETf)
                error('Check HINGEFORCE card, requires SET that does not exist')
            else
                idHinge = SET.Val(idSETf).data;
                if isempty(CAERO.Trim.Link)
                    nslave = 0;
                else
                    nslave = length(unique(CAERO.Trim.Link.Slave));
                end
                if ~all(ismember(idHinge,1:CAERO.geo.nc -nslave ))
                    error('Check HINGEFORCE card, forces required on aerodynamic control that do not exist')
                else
                    PARAM.HINGEFORCE = idHinge;
                end
            end
        else
            PARAM.HINGEFORCE = 1:CAERO.geo.nc -length(unique(CAERO.Trim.Link.Slave));
        end
    end
else
    if ~isempty(PARAM.AEROFORCE)
        fprintf(fid, '\n\n\t\t#Warning: SOL 144/145 cannot be run. Some data are missing.\n\t\tThe required AEROFORCE is set to empty.')
        PARAM.AEROFORCE = [];
    end
    if ~isempty(PARAM.HINGEFORCE)
        fprintf(fid, '\n\n\t\t#Warning: SOL 144/145 cannot be run. Some data are missing.\n\t\tThe required HINGEFORCE is set to empty.')
        PARAM.HINGEFORCE = [];
    end
    
end


if ~isempty(PARAM.DISP)
    if ~ischar(PARAM.DISP)
        idSETf = find(SET.ID == PARAM.DISP);
        if isempty(idSETf)
            error('DISP card requires an undefined SET.')
        else
            idDisp = sort(SET.Val(idSETf).data);
            if ~all(ismember(idDisp,NODE.ID))
                error('Check DISP card, displacements required on grid that do not exist.')
            else
                [dummy,PARAM.DISP] = ismember(idDisp,NODE.ID);
            end
        end
    else
        PARAM.DISP = 1:length(NODE.ID);
    end
end

if ~isempty(PARAM.VELOCITY)
    if ~ischar(PARAM.VELOCITY)
        idSETf = find(SET.ID == PARAM.VELOCITY);
        if isempty(idSETf)
            error('VELOCITY card requires an undefined SET.')
        else
            idVelocity =sort( SET.Val(idSETf).data);
            if ~all(ismember(idVelocity,NODE.ID))
                error('Check VELOCITY card, velocities required on grid that do not exist.')
            else
                [dummy,PARAM.VELOCITY] = ismember(idVelocity,NODE.ID);
            end
        end
    else
        PARAM.VELOCITY = 1:length(NODE.ID);
    end
end

if ~isempty(PARAM.ACCELERATION)
    if ~ischar(PARAM.ACCELERATION)
        idSETf = find(SET.ID == PARAM.ACCELERATION);
        if isempty(idSETf)
            error('ACCELERATION card requires an undefined SET.')
        else
            idAcceleration = sort(SET.Val(idSETf).data);
            if ~all(ismember(idAcceleration,NODE.ID))
                error('Check ACCELERATION card, accelerations required on grid that do not exist.')
            else
                [dummy,PARAM.ACCELERATION] = ismember(idAcceleration,NODE.ID);
            end
        end
    else
        PARAM.ACCELERATION = 1:length(NODE.ID);
    end
end

% internal load on Bar

if ~isempty(PARAM.IFORCE)
    if ~ischar(PARAM.IFORCE)
        idSETf = find(SET.ID == PARAM.IFORCE);
        if isempty(idSETf)
            error('Check IFORCE card: it requires a SET that does not exist')
        else
            idIforce = sort(SET.Val(idSETf).data);
            if ~all(ismember(idIforce, BAR.ID))
                error('Check IFORCE card, internal loads required on bar that do not exist')
            else
                [~, PARAM.IFORCE] = ismember(idIforce, BAR.ID);
                %                PARAM.IFORCE = idIforce;
            end
        end
    else
        PARAM.IFORCE = 1:length(BAR.ID);
    end
end

% internal load on beam

if ~isempty(PARAM.IFORCEBE)
    if ~ischar(PARAM.IFORCEBE)
        idSETf = find(SET.ID == PARAM.IFORCEBE);
        if isempty(idSETf)
            error('Check I card, requires SET that does not exist')
        else
            idIforce = sort(SET.Val(idSETf).data);
            if ~all(ismember(idIforce,BEAM.ID))
                error('Check IFORCEBE card, internal loads required on beam that do not exist')
            else
                [dummy,PARAM.IFORCEBE] = ismember(idIforce,BEAM.ID);
                %                PARAM.IFORCE = idIforce;
            end
        end
    else
        PARAM.IFORCEBE = 1:length(BEAM.ID);
    end
end

% check SURFDEF card (label)

if ~isempty(SURFDEF.ID)
    if ~isempty(find(PARAM.MSOL==144,1))
        if isempty(CAERO.Trim.Link)
            masterSurf =unique(CAERO.lattice_vlm.Control.Name);
        else
            masterSurf =[unique(CAERO.Trim.Link.Master), unique(setdiff(CAERO.lattice_vlm.Control.Name,[CAERO.Trim.Link.Slave, CAERO.Trim.Link.Master]))];
        end
    elseif ~isempty(find(PARAM.MSOL==145,1))
        if isempty(CAERO.Trim.Link)
            masterSurf =unique(CAERO.lattice_dlm.Control.Name);
        else
            masterSurf =[unique(CAERO.Trim.Link.Master), unique(setdiff(CAERO.lattice_dlm.Control.Name,[CAERO.Trim.Link.Slave, CAERO.Trim.Link.Master]))];
        end
    else
        error('SOL 144/145 cannot be run. Some data are missing.\n\t\tThe required SURFDEF input cannot be applied.')
    end
    SURFDEF.LabelID = zeros(1, length(SURFDEF.ID));
    for i=1: length(SURFDEF.ID)
        labelSurf = SURFDEF.Label{i};
        if any(strcmp(labelSurf,masterSurf))
            SURFDEF.LabelID(i) = find(strcmp(labelSurf,masterSurf) == 1);
        else
            error([labelSurf, 'in SURFDEF ', num2str(SURFDEF.ID(i)),'does not exist or is slave control.'])
        end
    end
    SURFDEF = rmfield(SURFDEF,'Label');
end
% Check dynamic loads
if (PARAM.LOAD)
  if ~all(ismember(PARAM.LOAD,DEXTLOAD.ID))
    fprintf('\n\t\t The selected DEXTLOAD input %d does not exist.', ...
    PARAM.LOAD(find(ismember(PARAM.LOAD,DEXTLOAD.ID)==0)));
    error('Error in LOAD= card.')
  else
    for i=1:ndload
      index = find(DEXTLOAD.Node(i) == NODE.ID);
      if (isempty(index))
        error(['Node ID ', num2str(DEXTLOAD.Node(i)),' in DEXTLOAD card ', num2str(DEXTLOAD.ID(i)),'does not exist.']);
      end
      if (NODE.DOF(index, DEXTLOAD.NDOF(i))==0)
        error(['Node ', num2str(DEXTLOAD.Node(i)),' has no DOF ', DEXTLOAD.NDOF(i),'.']);
      end
      DEXTLOAD.Node(i) = index;
    end
    PARAM.LOAD = find(PARAM.LOAD == DEXTLOAD.ID);
  end
end
% Check gust load set
if ~isempty(PARAM.GUST)
    if ~all(ismember(PARAM.GUST,GUST.ID))
      error('Gust load selected does not exist')
    else
      PARAM.GUST = find(PARAM.GUST == GUST.ID);
    end
end
% Check surfDef load set
if ~isempty(PARAM.SURFDEF)
    if ~all(ismember(PARAM.SURFDEF,SURFDEF.ID))
        fprintf('\n\t\t The selected SURFDEF input %d does not exist.', ...
                PARAM.SURFDEF(find(ismember(PARAM.SURFDEF,SURFDEF.ID)==0)));
        error('Error in SURDEF= card.')
    else
        PARAM.SURFDEF =  find(PARAM.SURFDEF == SURFDEF.ID);
    end
end
% Check FMODES 
if (~isempty(PARAM.FMODES) && ~isempty(PARAM.MSELECT))
  if (~isempty(setdiff(PARAM.FMODES, PARAM.MSELECT)))
    error('FMODES are not a subset of MSELECT.')
  end
end
% Check UMODES 
if (~isempty(PARAM.UMODES) && ~isempty(PARAM.MSELECT))
  if (~isempty(setdiff(PARAM.UMODES, PARAM.MSELECT)))
    error('UMODES are not a subset of MSELECT.')
  end
end
%
% DEREXT
%
if PARAM.DER_TYPE == 2 && ntrimext == 0
  error('PARAM DER_TYPE set to 2 but no TRIMEXT card provided.')
end
if nderext
  [labels, i] = unique(CAERO.Trim.DExt.ID);
  % check for duplicated labels
  if (length(labels) ~= nderext)
    n = [1 : nderext];
    dof = CAERO.Trim.DExt.ID(setdiff(n, i));
    for k=1:length(dof)
      fprintf(fid, '\n\t### Warning: duplicated labels for DEXT: %d.', CAERO.Trim.DExt.ID(dof(k)));
    end
    error('DEXT entries have duplicated labels.');
  end
%
  for n=1:ntrimext
    if CAERO.Trim.Ext.DExt(n)
      index = find(CAERO.Trim.DExt.ID == CAERO.Trim.Ext.DExt(n)); 
      if ~isempty(index)
        CAERO.Trim.Ext.DExt(n) = index;
      else
        error(['TRIMEXT card ', num2str(CAERO.Trim.Ext.ID(n)), ' refer to unknown DEXT card ', num2str(CAERO.Trim.Ext.DExt(n)),'.']);
      end
    end
  end
%
%  if PARAM.DER_TYPE == 2
%    id = CAERO.Trim.Ext.DExt(CAERO.Trim.Ext.DExt>0);
%    [labels, i] = unique(id);
%    n = [1 : length(id)];
%    dof = id(setdiff(n, i));
%    for k=1:length(dof)
%      fprintf(fid, '\n\t### Warning: multiple TRIMEXT cards refer to DEREXT %d.', id(dof(k)));
%    end
%    error('TRIMEXT entries refer to the same DEREXT card.');
%  end
%
end
%
fprintf(fid,'\n\ndone.\n');
%
end % end of read file

%*******************************************************************************
% AUXILIARY FUNCTIONs
%*******************************************************************************
% Parse NASTRAN field and return it as a double
function num = num_field_parser(line, index)

FIELD = 8;

if length(line) < FIELD * (index-1)
    
    num = 0;
    
else
    
    minc = min(length(line), index * FIELD);
    field = strtok(line((index-1) * FIELD+1:minc));
    
    if ~isempty(field)
        
        num = str2num(field);
        
    else
        
        num = 0;
        
    end
    
end

end
%*******************************************************************************
function num = num_field_parser_1(line, index)

FIELD = 8;

if length(line) < FIELD * (index-1)
    
    num = 1;
    
else
    
    minc = min(length(line), index * FIELD);
    field = strtok(line((index-1) * FIELD+1:minc));
    
    if ~isempty(field)
        
        num = str2num(field);
        
    else
        
        num = 1;
        
    end
    
end

end
%*******************************************************************************
% Parse NASTRAN field and return it as a string
function str = string_field_parser(line, index)

FIELD = 8;

if length(line) < FIELD * (index-1)
    
    str = [];
    
else
    
    field = line((index-1) * FIELD+1:end);
    
    if ~isempty(field)
        
        if length(field) > FIELD
            str = strtok(char(field(1:FIELD)));
        else
            str = strtok(char(field));
        end
        
    else
        
        str = [];
        
    end
end

end
%*******************************************************************************
% Parse NASTRAN field and return it as a string
function str = include_field_parser(line, index)

FIELD = 8;

if length(line) < FIELD * (index-1)
    
    str = [];
    
else
    
    field = line((index-1) * FIELD+1:end);
    
    if ~isempty(field)
        
        if length(field) > FIELD
            str = char(field(1:FIELD));
        else
            str = char(field);
        end
        
    else
        
        str = [];
        
    end
end

end
%*******************************************************************************
function str = name_field_parser(line, index)

FIELD = 8;

if length(line) < FIELD * (index-1)
    
    str = '';
    
else
    
    field = line((index-1) * FIELD+1:end);
    
    if ~isempty(field)
        
        if length(field) > FIELD
            str = strtok(char(field(1:FIELD)));
        else
            str = strtok(char(field));
        end
        
    else
        
        str = '';
        
    end
end

end
%*******************************************************************************
% Determine BASIC reference frame as reported in NASTRAN guide for BAR elements
function R = get_basic_coord(ORIENT, NODE3, NODE1)

x1 = NODE3 - NODE1;
x2 = ORIENT;
x3 = cross(x1, x2);
if norm(x3) ==0
    
    error('Error in Bar/Beam basic reference frame construction. Points are collinear.');
    
end

x2 = cross(x3, x1);
x1 = x1 ./ norm(x1);
x2 = x2 ./ norm(x2);
x3 = x3 ./ norm(x3);
R = [x1', x2', x3'];

end
%*******************************************************************************
% Determine offset vector for midline node in global reference frame
function offset = get_midline_offset(NODE1, NODE2, NODE3, OFFSET1, OFFSET3)

x1 = NODE1 + OFFSET1;
x3 = NODE3 + OFFSET3;

midnode = mean([x1 ; x3], 1);
% offset2
offset = midnode - NODE2;

end
%*******************************************************************************
% Parse FORCE or MOMENT cards
% This struct is not sorted by ID. It is not important

function UPD_FORCE = get_load_dir(nforce, UFORCE, NODE, COORD)

FORCE = UFORCE;

for n=1:nforce
    
    i = find(NODE.ID == FORCE.Node(n));
    
    if isempty(i)
        error('Unable to find node %d for force %d.', FORCE.Node(n), FORCE.ID(n));
    else
        FORCE.Node(n) = i; % use node index for rapid accessing
    end
    
    switch FORCE.Type(n)
        
        case 0 % FORCE
            % set force in global reference frame
            if FORCE.CID(n) ~=0
                
                i = find(COORD.ID == FORCE.CID(n));
                if isempty(i)
                    
                    error('Unable to find reference frame %d in Coordinate system database.', FORCE.CID(n));
                    
                else
                    FORCE.CID(n) = i; % store reference index for rapid accessing
                    % determine global values
                    FORCE.Orient(n,:) = (COORD.R(:,:,i) * FORCE.Orient(n,:)')';
                end
                
            end
            FORCE.Orient(n,:) = FORCE.Orient(n,:) ./ norm(FORCE.Orient(n,:));
            
        case 1 %FORCE1
            
            % set force in global reference frame
            index = [];
            for j=1:2
                for i = 1:ngrid
                    if NODE.ID(i) == int32(FORCE.Orient(n,j));
                        index(j) = i;
                        break;
                    end
                end
            end
            
            if length(index) ~=2
                
                error('Unable to determine direction for force %d acting on node %d.', FORCE.ID(n), NODE.ID(FORCE.Node(n)));
                
            else
                
                FORCE.Orient(n,:) = NODE.Coord(index(2),:) - NODE.Coord(index(1),:);
                if norm(FORCE.Orient(n,:)) == 0
                    error('Input nodes for force acting on node %d are coincident.', NODE.ID(FORCE.Node(n)));
                end
                FORCE.Orient(n,:) = FORCE.Orient(n,:) ./ norm(FORCE.Orient(n,:));
            end
            
        case 2 %FORCE2
            
            index = [];
            for j=1:3
                for i = 1:ngrid
                    if NODE.ID(i) == int32(FORCE.Orient(n,j));
                        index(j) = i;
                        break;
                    end
                end
            end
            
            if length(index) ~=3
                
                error('Unable to determine direction for force %d acting on node %d.', FORCE.ID(n), NODE.ID(FORCE.Node(n)));
                
            else
                
                for i = 1:ngrid
                    if NODE.ID(i) == FORCE.CID(n);
                        index(4) = i;
                        break;
                    end
                end
                
                if length(index) ~=4
                    
                    error('Unable to determine direction for force %d acting on node %d.',FORCE.ID(n), NODE.ID(FORCE.Node(n)));
                    
                else
                    x1 = NODE.Coord(index(2),:) - NODE.Coord(index(1),:);
                    x2 = NODE.Coord(index(4),:) - NODE.Coord(index(3),:);
                    
                    FORCE.Orient(n,:) = cross(x1, x2);
                    if norm(FORCE.Orient(n,:)) == 0
                        error('Input nodes for force %d acting on node %d are collinear.', FORCE.ID(n), NODE.ID(FORCE.Node(n)));
                    end
                    FORCE.Orient(n,:) = FORCE.Orient(n,:) ./ norm(FORCE.Orient(n,:));
                end
            end
    end % end switch
    
end % end loop

UPD_FORCE = FORCE;

end
%*******************************************************************************
function data = interp_tapered_beam_data(ns, input, x)

data = input;

if ns >2 % if midside info are available
    
    index = [1:ns];
    i = find(input(1, :) == 0);
    if (~isempty(i)) && (norm(input(i)) > 0)
        
        j = setdiff(index, i);
        
        data(1, i) = interp1(x(j), input(1,j), x(i));
    end
    
end

end
%*******************************************************************************
function UPD_FORCE = get_flw_load_dir(nforce, UFORCE, NODE, COORD)

FORCE = UFORCE;

for n=1:nforce
    
    i = find(NODE.ID == FORCE.Node(n));
    
    if isempty(i)
        error('Unable to find node %d for follower force %d.', FORCE.Node(n), FORCE.ID(n));
    else
        FORCE.Node(n) = i; % use node index for rapid accessing
    end
    
    if FORCE.CID(n) ~=0
        
        i = find(COORD.ID == FORCE.CID(n));
        if isempty(i)
            
            error('Unable to find reference frame %d in Coordinate system database for follower force %d.', FORCE.CID(n), FORCE.ID(n));
            
        else
            
            FORCE.CID(n) = i; % store reference index for rapid accessing
            % determine global values == node reference frame
            FORCE.Orient(n,:) = (COORD.R(:,:,i) * FORCE.Orient(n,:)')';
            FORCE.Offset(n,:) = (COORD.R(:,:,i) * FORCE.Offset(n,:)')';
            
        end
        
    end
end

UPD_FORCE = FORCE;

end
%*******************************************************************************
function SOL = set_available_sol(fid, INFO, PARAM)

nsol = 10;
SOL = zeros(nsol, 1);

fprintf(fid, '\n\tSetting available solvers...');
% cards required for static analysis 101
lins_solver = [INFO.ndof INFO.cc_nspc PARAM.LOAD];
index = find(lins_solver==0);
if (isempty(index))
    SOL(1) = 101;
end
% cards required for vibration modes solver 103
eig_solver = [INFO.ndof INFO.neig];
index = find(eig_solver==0);
if (isempty(index))
    SOL(2) = 103;
end

% cards required for buckling solver 105
buck_solver = [INFO.ndof INFO.neig INFO.cc_nspc PARAM.LOAD];
index = find(eig_solver==0);
if (isempty(index))
    SOL(3) = 105;
end

% cards required for linear aeroelastic solver 144
lin_aerosolver = [INFO.ndof INFO.ncaero INFO.ninterp INFO.naeros INFO.ntrim INFO.cc_trim];
index = find(lin_aerosolver==0);
if (isempty(index))
    SOL(4) = 144;
end

% cards required for flutter solver 145
if isempty(PARAM.EIG_FILE)
    fl_solver = [INFO.ndof INFO.ncaero INFO.ninterp INFO.nmkaero INFO.neig INFO.naero];
else
    fl_solver = [INFO.ncaero INFO.ninterp INFO.nmkaero INFO.naero];
end

index = find(fl_solver==0);
if (isempty(index))
    SOL(5) = 145;
end

if SOL(4) == 144 && SOL(5) == 145 && PARAM.SOL == 150
    SOL(10) = 150;
end

% cards required for flutter solver 600
nlins_solver = [INFO.ndof INFO.cc_nspc (INFO.nf+INFO.nm+INFO.nflw) (INFO.nbar+INFO.nbeam)];
index = find(nlins_solver==0);
if (isempty(index))
    SOL(6) = 600;
end
% cards required for nonlinear aeroelastic solver 644
nlin_aerosolver = [INFO.ndof INFO.ncaero INFO.ninterp INFO.naeros INFO.ntrim INFO.cc_nspc INFO.cc_trim];
index = find(nlin_aerosolver==0);
if (isempty(index))
    SOL(7) = 644;
end
% cards required for rigid aerodynamic solution using VLM 700
rlin_aerosolver = [max(INFO.ncaero,INFO.nbaero) INFO.naeros INFO.ntrim INFO.cc_trim];
index = find(rlin_aerosolver==0);
if (isempty(index))
    SOL(8) = 700;
end
% cards required for rigid aerodynamic solution using DLM 701
dynlin_aerosolver = [INFO.ncaero INFO.naero INFO.nmkaero];
index = find(dynlin_aerosolver==0);
if (isempty(index))
    SOL(9) = 701;
end

fprintf(fid,'done.');

end

function [O, R] = cord2r_data(COORD2, n)

  O = COORD2.Nodes(1,1:3,n);
  x3 = COORD2.Nodes(2,1:3,n) - COORD2.Nodes(1,1:3,n);
  x1 = COORD2.Nodes(3,1:3,n) - COORD2.Nodes(1,1:3,n);
  x2 = cross(x3, x1);
  if det([x1; x2; x3]) == 0
      error('COORD2 %d definition has collinear points.', COORD2.ID(n));
  end
  x3 = x3 ./ norm(x3);
  x2 = x2 ./ norm(x2);
  x1 = cross(x2, x3);
  x1 = x1 ./ norm(x1);
  R = [x1', x2', x3'];

end