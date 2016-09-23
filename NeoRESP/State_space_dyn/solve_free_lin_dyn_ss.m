%
%***********************************************************************************************************************
%  FFAST Project
%
%  NeoSYM
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Lorenzo Travaglini   <>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by FFAST partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Luca Cavagna
%
%***********************************************************************************************************************
%
% Solve dynamic response for free aircraft in time domain
%
% Aerodynamic model is represented as state space system 
%
% Additional inputs: dT, Tmax
% dT: time step [s]
% Tmax: time period length [s]
% Example: solve_free_lin_dyn_ss('Tmax',5,'dT',0.01,'Ham','filename1.mat','Had','filename2.mat','Hag',filename3.mat');
%
% if Tmax is given, no estimate for damping will be carried out to determine the max response time decay.
% dT in this case will be derived from vacuum eigensolution.
%
% if Tmax is not given, the solver will estimate the damping for rigid and elastic dynamics.
% A new flutter solution will be carried out when PARAM RHO_REF is not equal to RHO_VG given in MKAERO1 card. 
% If PARAM MACH is not within the values in the aero database defined by MKAERO1, the nearest Mach number 
% available in the database will be used (zero order interpolation)
%
% If RHO_REF is equal to RHO_VG, the solver will directly use the flutter outputs of the preprocessor 'init_dyn_solver'.
% if dT is not given, the value will be derived from the aeroelastic eigenmodes.
%
% In any case, if dT is given, the user defined value will be used.
%
% At the first run no Ham, Had, Hag parameter is required. The solver will export according to the model, aerodynamic transfer
% matrices for motion, controls and gust.
%
% The user hence can use mygui or the script aero_ss to create 3 different state space models.
% Successively the solver will be called again, providing the output files from the fitting as inputs:


%-------------------------------------------------------------------------------------------------------------------------
% SUPORT and PARAM MACH/RHOREF/VREF are mandatory.
% MSELECT must include rigid modes from 1->6 to have aero coefficients.
% UMODES and FMODES sets are not mandatory.
% UMODES should at least include the rigid modes of interest, i.e. 3-5 for symmetric flight, 2-4-6 for asymmetric flight.
% FMODES should not include rigid body modes. Their damping will be carried out through a dedicated process to 
% estimate phugoid, short period, roll, duth roll and spiral modes.
% The values of phugoid are rather inaccurate due to the inviscid aero model.
%
%
function [ss_model, Ys, Ts, Xs, U] = solve_free_lin_dyn(varargin)

global dyn_model
global fl_model
%
ss_model = []; Ys = [];  Ts = []; Xs = []; U = [];
%
beam_model = dyn_model.beam;
%
NPFREQ = 4;
T1XR = 20;
T1XE = 20;
%
dtpos = find('.' == beam_model.Param.FILE);
headname = beam_model.Param.FILE(1:dtpos(end)-1);
%
fid = beam_model.Param.FID;
%
if (isempty(beam_model.Param.GUST) && isempty(beam_model.Param.SURFDEF) && isequal(beam_model.Param.LOAD, 0))
  fprintf(fid,'\n No input force defined. Solution ended.\n');
  return;
end
%
%
if beam_model.Param.SOL == 146 
% Initialize time values
  Tmax      = 0;
  Ttot      = 0;
  TimeDelay = 0;
  Dt        = 0;
  HAM_FILE = [];
  HAD_FILE = [];
  HAG_FILE = [];
  CHECK_MODEL = 0;
  input_delay = [];
  delay_f = [];
  delay_vg = [];
  delay_delta = [];
%
  VREF = beam_model.Param.VREF;
  RHOREF = beam_model.Param.RHOREF;
  RHO_VG = beam_model.Param.RHO_VG;
  MREF = beam_model.Param.MACH;
  qinfty = 0.5*RHOREF*VREF^2;
  fprintf(fid,'\n - Reference flight speed:     %g m/s.',   VREF); 
  fprintf(fid,'\n - Reference dynamic pressure: %g Pa.',    qinfty); 
  fprintf(fid,'\n - Reference density:          %g Kg/m3.', RHOREF); 
  fprintf(fid,'\n - Reference Mach number:      %g.',       MREF); 
  MINDEX = find(MREF == dyn_model.dlm.aero.M);
  if (isempty(MINDEX))
    fprintf(fid,'\n ### Warning: The required Mach %g is not within the aerodynamic dabatase.', MREF); 
    mdiff = dyn_model.dlm.aero.M-MREF; mdiff = sqrt(mdiff.*mdiff); [dummy, MINDEX] = min(mdiff); 
    fprintf(fid,'\n              All aero data will be extrapolated to the closest value available of %g.', ...
            dyn_model.dlm.aero.M(MINDEX)); 
  end
%
  nk = length(dyn_model.dlm.aero.k);
  cref = dyn_model.dlm.aero.cref;
  bref = beam_model.Aero.ref.b_ref;
  sref = beam_model.Aero.ref.S_ref;
  SDAMP = beam_model.Param.SDAMP;
%
  fprintf(fid,'\n - Reference chord:    %g m.', cref); 
  fprintf(fid,'\n - Reference span:     %g m.', bref); 
  fprintf(fid,'\n - Reference surface:  %g m2.', sref); 
  if (SDAMP)
    DAMP = beam_model.Damp;
    fprintf(fid,'\n - Structural damping required: '); 
    if (beam_model.Param.KDAMP == 1)
      fprintf(fid,'viscous type.'); 
    else
      error('Histeretic damping not compatible with time domain analysis.'); 
    end
  else
    fprintf(fid,'\n - No structural damping required.'); 
  end
% dofs
  ndof   = beam_model.Info.ndof;
  ndof2  = beam_model.Info.ndof2;
% Aero mesh coordinates and reference point
  np = beam_model.Aero.lattice_dlm.np;
  midPoint = beam_model.Aero.lattice_dlm.COLLOC(1:np,:)*cref;
%
  X0min = min(midPoint(:,1));
  Xsup = beam_model.WB.CG;
%-------------------------------------------------------------------------------
% MODAL BASE
%           
  NMODES = size(beam_model.Struct.Mmm,1);
% Modes used in eig and flutter
  if (isempty(beam_model.Param.MSELECT))
    mbase = beam_model.Struct.ID;
  else
    mbase = beam_model.Param.MSELECT;
  end
% Modes used in dynamic solution
% UMODESIND index from UMODES TO MBASE to reduce matrices
  if (isempty(beam_model.Param.UMODES))
    UMODESIND = [1:NMODES]; 
  else
    if(~all(ismember(beam_model.Param.UMODES, mbase)))
      index = find(ismember(beam_model.Param.UMODES, mbase)==0);
      fprintf(fid,'\n UMODE %d not present in current modal base.',...
              beam_model.Param.UMODES(index)); 
    end
    [dummy, dummy, UMODESIND] = intersect(beam_model.Param.UMODES, mbase);
  end
  UMODES = mbase(UMODESIND);
  nUMODES = length(UMODES);
%
  [UMODESE, index] = setdiff(UMODES, [1:6]);
% Find rigid modes involved            
  UMODESR = setdiff(UMODES, UMODESE);
  nr = length(UMODESR); % number of rigid modes in UMODES
  ne = length(UMODESE); % number of elastic modes in UMODES
% Find mode index in UMODES array
  [dummy, dummy, UMODESRLOC] = intersect(UMODESR, UMODES);
  [dummy, dummy, UMODESELOC] = intersect(UMODESE, UMODES);
% Find mode index in MBASE array
  UMODESRGLOB =  UMODESIND(1:nr);
  UMODESEGLOB =  UMODESIND(nr+1:end);
%
  side_mode   = find(UMODES == 2);
  plunge_mode = find(UMODES == 3);
  roll_mode   = find(UMODES == 4);
  pitch_mode  = find(UMODES == 5);
%
%  dyn_model.dlm.data.Qhh(:,UMODESRGLOB(side_mode),1,MINDEX) = 0;
%  dyn_model.dlm.data.Qhh(:,UMODESRGLOB(plunge_mode),1,MINDEX) = 0;
%  dyn_model.dlm.data.Qhh(:,UMODESRGLOB(roll_mode),1,MINDEX) = 0;
%
%-------------------------------------------------------------------------------
% update matrices to account for damping
  Kmm = beam_model.Struct.Kmm;
  Bmm = zeros(NMODES); % viscous damp
  Gmm = zeros(NMODES); % complex stiff
  if (SDAMP)
    if (beam_model.Param.KDAMP==1)
      Bmm = modal_damp(DAMP.g{SDAMP}, DAMP.Freq{SDAMP}, DAMP.Type(SDAMP), beam_model.Param.KDAMP, ...
                          beam_model.Struct.Mmm, beam_model.Struct.Omega./(2*pi));
    end
  end
%-------------------------------------------------------------------------------
%
% check load SET, if symmetric or antisymmetric or neither
%
  symLoad  = 1;
  NsymLoad = 1;
  nLOAD    = 0;
  nSURF    = 0;
  nGUST    = 0;
% nodal inputs
  if  ~isequal(beam_model.Param.LOAD, 0)
    nLOAD = length(beam_model.Param.LOAD);
    Tmax = max(beam_model.Dextload.X0(beam_model.Param.LOAD) + beam_model.Dextload.Tmax(beam_model.Param.LOAD));
    fextdof = beam_model.Dextload.NDOF(beam_model.Param.LOAD);
    symLoad   = all(fextdof == 3 | fextdof == 5);
    NsymLoad  = all(fextdof == 2 | fextdof == 4 | fextdof == 6);
    delay_f = [delay_f, beam_model.Dextload.X0(beam_model.Param.LOAD)];
  end
% controls
  if  ~isempty(beam_model.Param.SURFDEF)
    surfLoad = zeros(length(beam_model.Param.SURFDEF),1);
    nSURF = length(beam_model.Param.SURFDEF);
    Tmax = max([Tmax, max(beam_model.Surfdef.X0(beam_model.Param.SURFDEF) + beam_model.Surfdef.Tmax(beam_model.Param.SURFDEF))]);
    for i = 1: nSURF % choose if longitudinal or latero directional control
      [dummy, surfLoad(i)] = max(abs(dyn_model.dlm.data.Qhd(1:6,beam_model.Param.SURFDEF(i))));
    end
    symLoad  = all(surfLoad == 3 | surfLoad == 5);
    NsymLoad = all(surfLoad == 2 | surfLoad == 4 | surfLoad == 6);
    delay_delta = [delay_delta, beam_model.Surfdef.X0(beam_model.Param.SURFDEF)];
  end
% gusts
  if ~isempty(beam_model.Param.GUST)
    nGUST = length(beam_model.Param.GUST);
    gpos = find(X0min - beam_model.Gust.X0(beam_model.Param.GUST) <0);
    if ~isempty(gpos)
      fprintf(fid,'### Warning: gust entry point for GUST %d is placed downstream the first panel. The entry point is set to frame origin.', ...
               beam_model.Gust.ID(beam_model.Param.GUST(gpos)));
      beam_model.Gust.X0(beam_model.Param.GUST(gpos)) = 0;
    end
    Tmax = max([Tmax, max(abs((X0min - beam_model.Gust.X0(beam_model.Param.GUST))./VREF) + beam_model.Gust.Tmax(beam_model.Param.GUST))]);
    symLoad = all([symLoad , all(beam_model.Gust.DIR(beam_model.Param.GUST) == 3)]);
    NsymLoad = all([NsymLoad, all(beam_model.Gust.DIR(beam_model.Param.GUST) == 2)]);
    delay_vg = [delay_vg, (X0min - beam_model.Gust.X0(beam_model.Param.GUST))./VREF];
  end
%-------------------------------------------------------------------------------
  defom = find(UMODESE>6); ndefom = length(UMODESE(defom));
  TimeDefo = zeros(ndefom,1);
  DFREQ = zeros(ndefom,1);
  if (isempty(beam_model.Param.FMODES))
    flwbase = mbase;
  else
    flwbase = beam_model.Param.FMODES;
  end
%
% Check user defined inputs
  if nargin > 0
	  PARAM = varargin;
    for n=1:2:length(PARAM);
      param_set = 0;
      if (strcmp(PARAM(n), 'Tmax')) 
        Ttot = PARAM{n+1};
        fprintf(fid,'\n - User defined simulation period Tmax: %g s.', Ttot);
        param_set = 1;
      end
      if (strcmp(PARAM(n), 'dT')) 
        Dt = PARAM{n+1};
        fprintf(fid,'\n - User defined time step dt: %g s.', Dt);
        param_set = 1;
      end
      if (strcmp(PARAM(n), 'Ham')) 
        HAM_FILE = PARAM{n+1};
        fprintf(fid,'\n - Filename with aerodynamic fitting of Ham: %s.', HAM_FILE);
        if ~exist(HAM_FILE,'file')
          error(['Unable to find file ',char(PARAM(n+1)),'.']);
        end
        param_set = 1;
      end
      if (strcmp(PARAM(n), 'Had')) 
        HAD_FILE = PARAM{n+1};
        fprintf(fid,'\n - Filename with aerodynamic fitting of Had: %s.', HAD_FILE);
        if ~exist(HAD_FILE,'file')
          error(['Unable to find file ',char(PARAM(n+1)),'.']);
        end
        param_set = 1;
      end
      if (strcmp(PARAM(n), 'Hag')) 
        HAG_FILE = PARAM{n+1};
        fprintf(fid,'\n - Filename with aerodynamic fitting of Hag: %s.', HAG_FILE);
        if ~exist(HAG_FILE,'file')
          error(['Unable to find file ',char(PARAM(n+1)),'.']);
        end
        param_set = 1;
      end
      if (strcmp(PARAM(n), 'check')) 
        CHECK_MODEL = PARAM{n+1};
        fprintf(fid,'\n - Model check enabled.');
        param_set = 1;
      end
      if ~param_set
        error(['Unknown input parameter: ', char(PARAM(n)),'.']);
      end
    end
  end
%
if isempty(HAM_FILE)
%  fprintf(fid,'\n - No aerodynamic model for structural motion Ham provided. Aerodynamic data will be exported only.');
% avoid simulation time and time step calculation
  Ttot = 1; 
  dT = 1;
else
  if nLOAD
    if isempty(HAD_FILE)
      error('No aerodynamic model for controls Had provided.');
    end
  end  
  if nGUST
    if isempty(HAG_FILE)
      error('No aerodynamic model for gust Hag provided.');
    end
  end  
end

%
% DAMPING ESTIMATE for TMAX
  if (Ttot==0)
%
%   DAMPING AVAILABLE
%
    if (RHOREF == RHO_VG)
%
%     RIGID MODES
%
      sigma_ph = interp1(dyn_model.flu.Res_rigid.Velocity, dyn_model.flu.Res_rigid.data(MINDEX).RealE{1}, VREF, 'linear', 'extrap');   
      sigma_sp = interp1(dyn_model.flu.Res_rigid.Velocity, dyn_model.flu.Res_rigid.data(MINDEX).RealE{2}, VREF, 'linear', 'extrap');     
      %
      sigma_dr = interp1(dyn_model.flu.Res_rigid.Velocity, dyn_model.flu.Res_rigid.data(MINDEX).RealE{5}, VREF, 'linear', 'extrap');       
      %
      fprintf(fid,'\n - RIGID MODES: time to 1/%g amplitude:', T1XR); 
      symTime = log(1/T1XR) / sigma_sp;
      fprintf(fid,'\n\t - Short period:     %g s.', symTime); 
      NsymTime = log(1/T1XR) / sigma_dr;
      fprintf(fid,'\n\t - Dutch roll:       %g s.', NsymTime); 
%
%     ELASTIC MODES
%
      fprintf(fid,'\n - ELASTIC MODES: time to 1/%g amplitude:', T1XE); 
      for i = 1:ndefom
        ind = find(UMODESE(defom(i)) == flwbase);
        if (isempty(ind))
          error(['Mode ',num2str(UMODESE(defom(i))),' not defined in FMODES set.']);
        end
        [Vel, indVel] = unique(dyn_model.flu.Res.data(MINDEX).Velocity{ind});
        TimeDefo(i) = log(1/T1XE)/(spline(Vel,dyn_model.flu.Res.data(MINDEX).RealE{ind}(indVel),VREF));
        DFREQ(i) = (spline(Vel,dyn_model.flu.Res.data(MINDEX).Freq{ind}(indVel),VREF)); 
        % Print summary to help in choosing modal base
        fprintf(fid,'\n\t - Mode %d: %g s, %g Hz.  ', UMODESE(defom(i)), TimeDefo(i), DFREQ(i)); 
      end
    else
%
%   NEW DAMPING CALCULATION
%
%
%     RIGID MODES
%
      counter = 99;
      RigRes = []; RigRes.data = [];
      RStab_Der = [];
      RStab_Der = get_dynder(1, dyn_model.dlm.aero.k, cref, cref*2, bref, sref, dyn_model.dlm.data.Qhh(:,:,:,MINDEX), 0, []);
      [dummy, RigRes] = rig_modes(fid, -2, RHOREF, VREF, MREF, cref, bref, sref,...
                       beam_model.WB.MCG(1,1), diag(beam_model.WB.MCG(4:6,4:6)), ...
                       RStab_Der, 1, RigRes);
      sigma_ph = RigRes.data.RealE{1}(1);   
      sigma_sp = RigRes.data.RealE{2}(1);    
      %
      sigma_dr = RigRes.data.RealE{5}(1);      
      %
      fprintf(fid,'\n - RIGID MODES: time to 1/%g amplitude:', T1XR); 
      symTime = log(1/T1XR) / sigma_sp;
      fprintf(fid,'\n\t - Short period:     %g s.', symTime); 
      NsymTime = log(1/T1XR) / sigma_dr;
      fprintf(fid,'\n\t - Dutch roll:       %g s.', NsymTime); 
      fprintf(fid,'\n - RIGID MODES: frequency at VREF %g m/s:', VREF); 
      fprintf(fid,'\n\t - Short period:     %g Hz.', RigRes.data.Freq{2}(1)); 
      fprintf(fid,'\n\t - Dutch roll:       %g Hz.', RigRes.data.Freq{5}(1)); 
%
%     ELASTIC MODES
%
      fl_model.Res = [];
      fl_model.Res.data = [];
      fl_model.Res.Env = [];
      vmax =  beam_model.Param.VMAX;
      vstep = vmax/beam_model.Param.NVSTEP;
      % run flutter
      Hamf = zeros(NMODES,NMODES,nk*2);
      Hamf(:,:,1:2:nk*2) = dyn_model.dlm.data.Qhh(:,:,1:nk,MINDEX);
      [VF, HF] = run_flutter(fid, beam_model.Struct.Mmm, Gmm*beam_model.Struct.Kmm, beam_model.Struct.Kmm, beam_model.Struct.Omega./(2*pi), ...
      beam_model.Struct.ID, Hamf, dyn_model.dlm.aero.k, MREF, cref, mbase, UMODESE(defom), RHOREF, [vstep, vmax], beam_model.Param.AUTOPLOT,...
                             counter, 1);
      fprintf(fid,'\n - ELASTIC MODES: time to 1/%g amplitude:', T1XE); 
      for i = 1:ndefom
        [Vel, indVel] = unique(fl_model.Res.data.Velocity{i});
        TimeDefo(i) = log(1/T1XE)/(spline(Vel, fl_model.Res.data.RealE{i}(indVel),VREF));
        DFREQ(i) = (spline(Vel, fl_model.Res.data.Freq{i}(indVel),VREF));
        % Print summary to help in choosing modal base
        fprintf(fid,'\n\t - Mode %d: %g s, ', UMODESE(defom(i)), TimeDefo(i)); 
      end
      fprintf(fid,'\n - ELASTIC MODES: frequency at VREF %g m/s:', VREF); 
      for i = 1:ndefom
        fprintf(fid,'\n\t - Mode %d: %g Hz, ', UMODESE(defom(i)), DFREQ(i)); 
      end
    end
    if (Dt == 0)
      Dt = 1/(max(DFREQ) * NPFREQ);
    end
    T_defo = max(TimeDefo);                
    fprintf(fid,'\n - Elastic modes period TE set to %g s.', T_defo); 
    if symLoad
      fprintf(fid,'\n - Rigid modes period TR set to %g s.', symTime); 
      TimeDelay = max([symTime;TimeDefo]);
    elseif NsymLoad
      fprintf(fid,'\n - Rigid modes period TR set to %g s.', NsymTime); 
      TimeDelay = max([NsymTime;TimeDefo]);
    else
      TimeDelay = max([symTime; NsymTime; TimeDefo]);
    end
    fprintf(fid,'\n - Time step dt: %g s.', Dt);
    fprintf(fid,'\n - Input max period TI: %g s.', Tmax);
    Ttot = Tmax+TimeDelay;
    fprintf(fid,'\n - Simulation period Tmax = TI + max(TR,TE): %g s.', Ttot);
  end
  % compute dt starting from model's frequency
  if (Dt==0)
    Dt = 2*pi / max(beam_model.Struct.Omega(UMODESIND)) / NPFREQ;
    fprintf(fid,'\n - Time step dt: %g s.', Dt);
  end
%-------------------------------------------------------------------------------
%
  T = (0 : Dt : Ttot)';
  nt = length(T);
  fprintf(fid,'\n - Time sample points: %d.', nt);
%
% Check aero model
%
  FMAX     = 1/Dt;
  KMAX     = 2*pi*FMAX*dyn_model.dlm.aero.cref/VREF;
  KMIN_DLM = min((2*pi/beam_model.Param.DLM_NP)./beam_model.Aero.lattice_dlm.dx);
  fprintf(fid,'\n - Max frequency FMAX     : %g Hz.', FMAX);
  fprintf(fid,'\n - Max red. frequency KMAX: %g.', KMAX);
  if (KMAX > KMIN_DLM)
    fprintf(fid,'\n   ### Warning: aerodynamic mesh too coarse to sample KMAX with %d points.', beam_model.Param.DLM_NP);
    fprintf(fid,'\n                Aerodynamic maximum red. frequency: %g. ', KMIN_DLM);
  end
%-------------------------------------------------------------------------------
%
% Assembly state space model
%
%
% scale RIGID modes 
% Mass matrix is made equal to unity to avoid bad scaling in aero terms
% 
PHI = eye(nUMODES);
PHIR = diag(diag(1./sqrt(beam_model.Struct.Mmm(1:6,1:6))));
PHI = blkdiag(PHIR(UMODESRGLOB,UMODESRGLOB), eye(ne));
%
Ham_e = [];
Had = []; Ham_de = [];
Hag = []; Ham_ge = []; Ham_gh = [];
%
Ham = dyn_model.dlm.data.Qhh(UMODESIND,UMODESIND,:,MINDEX);
%
% Aero global forces to Ham
[Cy, Cz, Cl, Cm, Cn] = rigid_aero_force(dyn_model.dlm.data.Cp(:,UMODESIND,:,MINDEX), ...
                                                  dyn_model, cref, bref, sref, Xsup);
Ham_e = [Cy(:,:,1:2:2*nk); Cz(:,:,1:2:2*nk); Cl(:,:,1:2:2*nk); Cm(:,:,1:2:2*nk); Cn(:,:,1:2:2*nk)];
%
if  ~isempty(beam_model.Param.SURFDEF)
  Qdh = dyn_model.dlm.data.Qdh(beam_model.Param.HINGEFORCE, UMODESIND, :, MINDEX);
%  Qdh(:,side_mode,1) = 1i*imag(Qdh(:,side_mode,1));  Qdh(:,plunge_mode,1) = 1i*imag(Qdh(:,plunge_mode,1)); Qdh(:,roll_mode,1) = 1i*imag(Qdh(:,roll_mode,1));
  Had = dyn_model.dlm.data.Qhd(UMODESIND,beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF),:, MINDEX);
%  scale Had
  for i=1:nk
    Had(:,:,i) = PHI' * Had(:,:,i);
  end
  [Cy, Cz, Cl, Cm, Cn] = rigid_aero_force(dyn_model.dlm.data.Cp(:,NMODES+beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF),:,MINDEX),...
                                       dyn_model, cref, bref, sref, Xsup);
  Ham_de = [Cy(:,:,1:2:2*nk); Cz(:,:,1:2:2*nk); Cl(:,:,1:2:2*nk); Cm(:,:,1:2:2*nk); Cn(:,:,1:2:2*nk)];
% Aero hinge moments to Ham and Had
  if ~isempty(beam_model.Param.HINGEFORCE)
    Ham_e = [Ham_e; Qdh];
    Ham_de = [Ham_de; dyn_model.dlm.data.Qdd(beam_model.Param.HINGEFORCE,beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF), :, MINDEX);];
  end
%
end
%
if ~isempty(beam_model.Param.GUST)
  Hag = zeros(nUMODES, nGUST, nk);
  dyn_model.Res.Gust_profile = zeros(nGUST, nt); 
  for i = 1:nGUST
% assembly gust delay
%    Qhg = exp(-( midPoint(:,1) - beam_model.Gust.X0(beam_model.Param.GUST(i)) )*(1i*dyn_model.dlm.aero.k/cref) )...
%      .* repmat(dyn_model.gust.dwnwash(:,beam_model.Param.GUST(i)),1,nk); 
    Qhg = exp(-( midPoint(:,1) - X0min) *(1i*dyn_model.dlm.aero.k/cref) )...
            .* repmat(dyn_model.gust.dwnwash(:,beam_model.Param.GUST(i)),1,nk); 

%    assembly k column
%    scale Hag
    Hag_data = [];
    for j=1:nk
      Hag(:,i,j) = PHI' * dyn_model.gust.Qhg(UMODESIND,:,j,MINDEX) * Qhg(:,j);
      Hag_data(:,1,j) = dyn_model.gust.Cp(:,:,j,MINDEX) * Qhg(:,j);
    end
    [Cy, Cz, Cl, Cm, Cn] = rigid_aero_force(Hag_data, dyn_model, cref, bref, sref, Xsup);
    Ham_ge = [Ham_ge, Cy(:,:,1:2:2*nk); Cz(:,:,1:2:2*nk); Cl(:,:,1:2:2*nk); Cm(:,:,1:2:2*nk); Cn(:,:,1:2:2*nk)];
%
    if nSURF
      if ~isempty(beam_model.Param.HINGEFORCE)
        Hag_data = [];
        Hag_data(:,1,j) = dyn_model.gust.Qdg(beam_model.Param.HINGEFORCE,:,j,MINDEX) * Qhg(:,j);
      end
      Ham_gh = [Ham_gh, Hag_data];
    end
  end
  Ham_ge = [Ham_ge; Ham_gh];
end
% scale Ham
for i=1:nk
  Ham(:,:,i)   = PHI' * Ham(:,:,i)   * PHI;
  Ham_e(:,:,i) =        Ham_e(:,:,i) * PHI;
end
% force zero terms
%Ham(:,side_mode,1) = 0;   Ham(:,plunge_mode,1) = 0;   Ham(:,roll_mode,1) = 0;
%Ham_e(:,side_mode,1) = 0; Ham_e(:,plunge_mode,1) = 0; Ham_e(:,roll_mode,1) = 0;
%Ham(:,side_mode,1) = 1i*imag(Ham(:,side_mode,1));  Ham(:,plunge_mode,1) = 1i*imag(Ham(:,plunge_mode,1)); Ham(:,roll_mode,1) = 1i*imag(Ham(:,roll_mode,1));
%Ham_e(:,side_mode,1) = 1i*imag(Ham_e(:,side_mode,1)); Ham_e(:,plunge_mode,1) = 1i*imag(Ham_e(:,plunge_mode,1)); Ham_e(:,roll_mode,1) = 1i*imag(Ham_e(:,roll_mode,1));
%
%---------------------------------------------------------------------------------------------------
% 
% AERO DATA
% 
if isempty(HAM_FILE)
%
  k = dyn_model.dlm.aero.k;
  Ha = [Ham; Ham_e];
%
  filename = [headname,'_Ham_M_',num2str(MREF),'.mat'];
  save(filename,'k','Ha');
  fprintf(fid, '\n\n - Aerodynamic matrix Ham exported to %s file for fitting.', filename);
  fprintf(fid,'\n   Rows: %d.', nUMODES);
  fprintf(fid,'\n   Columns: %d.', nUMODES);
  fprintf(fid,'\n   Extra outputs: %d.', size(Ham_e,1));
%
  if nSURF
    Ha = [Had; Ham_de];
    filename = [headname,'_Had_M_',num2str(MREF),'.mat'];
    save(filename,'k','Ha');
    fprintf(fid, '\n\n - Aerodynamic matrix Had exported to %s file for fitting.', filename);
    fprintf(fid,'\n   Rows: %d.', nUMODES);
    fprintf(fid,'\n   Columns: %d.', nSURF);
    fprintf(fid,'\n   Extra outputs: %d.', size(Ham_de,1));
  end
%
  if nGUST
    Ha = [Hag; Ham_ge];
    filename = [headname,'_Hag_M_',num2str(MREF),'.mat'];
    save(filename,'k','Ha');
    fprintf(fid, '\n\n - Aerodynamic matrix Hag exported to %s file for fitting.', filename);
    fprintf(fid,'\n   Rows: %d.', nUMODES);
    fprintf(fid,'\n   Columns: %d.', nGUST);
    fprintf(fid,'\n   Extra outputs: %d.', size(Ham_ge,1));
  end
%
  fprintf(fid, '\n - Use the panel GUI to fit aerodynamic data separately. Call ''mygui'' or use the function ''aero_ss''.');
  fprintf(fid, '\n - When fitting is concluded, run the solver with ''Ham'', ''Had'', ''Hag'' parameters followed by each output file created.\n');
%
  return
else
% number of states
  nXa   = 0;
  nXa_d = 0;
  nXa_g = 0;
%
  fprintf(fid,'\n - Number of structural states: %d.', 2*nUMODES);
  [Aa,B0,B1,B2,Ca,E0,E1,E2,Ca_e,E0_e,E1_e,E2_e,nXa,solution] = load_fitting(HAM_FILE, cref, VREF, nUMODES);
  fprintf(fid,'\n - Number of aerodynamic states for Ham: %d.', nXa);
%
  Aid    = solution.inoutresid.AA;
  Bid{1} = solution.inoutresid.BB{1};
  Bid{2} = solution.inoutresid.BB{2};
  Bid{3} = solution.inoutresid.BB{3};
  Cid    = solution.inoutresid.CC;
  Eid{1} = solution.inoutresid.DD{1};
  Eid{2} = solution.inoutresid.DD{2};
  Eid{3} = solution.inoutresid.DD{3};
%
  E0s = Eid{1} - Cid*(Aid\Bid{1});
%  PHI'*real(dyn_model.dlm.data.Qhh(UMODESIND,UMODESIND,1,MINDEX))*PHI
  E1s = Eid{2} - Cid*(Aid\Bid{2}) + Cid*(Aid\(Aid\Bid{1}));
%  PHI'*imag(dyn_model.dlm.data.Qhh(UMODESIND,UMODESIND,1,MINDEX))./dyn_model.dlm.aero.k(1)*PHI
  E2s = 2*(Eid{3} - Cid*(Aid\(Aid\(Aid\(Bid{1})))) - Cid * (Aid\Bid{3}) + Cid * (Aid\(Aid\(Bid{2}))));
%  -2*PHI'*( real(dyn_model.dlm.data.Qhh(UMODESIND,UMODESIND,2,MINDEX)) - real(dyn_model.dlm.data.Qhh(UMODESIND,UMODESIND,1,MINDEX)))*PHI ./(dyn_model.dlm.aero.k(2)-dyn_model.dlm.aero.k(1))^2
%
% Check quasi-steady derivatives
  fprintf(fid,'\n - Quasi-steady derivatives:');
  fprintf(fid,'\n                    SS model       Matrix    Freq. model');
  RStab_Der = get_dynder(1, dyn_model.dlm.aero.k, cref, cref*2, bref, sref, dyn_model.dlm.data.Qhh(:,:,:,MINDEX), 0, []);
  c2ref = 2*cref;
  if ~isempty(pitch_mode)
    clad = 0;
    if ~isempty(plunge_mode)
      r = plunge_mode; c = pitch_mode;
      cla = E0s(r,c)/PHI(r,r)/PHI(c,c)/sref;
      fprintf(fid,'\n   Cz/alpha:        %-15gE0        %-15g', cla, RStab_Der.Alpha.dcl_dalpha(1));
      cla = -(E1s(r,r))*cref/PHI(r,r)^2/sref;
      fprintf(fid,'\n   Cz/alpha:        %-15gE1        %-15g', cla, RStab_Der.Alpha.dcl_dalpha(1));
      clad = -E2s(r,r)*(cref/VREF)/PHI(r,r)^2/sref;
      fprintf(fid,'\n   Cz/alphad:       %-15gE2        %-15g', clad, RStab_Der.Alpha.dcl_dalpha_dot(1));
      czq = E1s(r,c)/PHI(r,r)/PHI(c,c)/sref - clad;
      fprintf(fid,'\n   Cz/q:            %-15gE1,E2     %-15g', czq, RStab_Der.Q_rate.dcl_dQ(1));
    end
    r = pitch_mode; c = plunge_mode; 
    cma = E0s(r,r)/PHI(r,r)^2/sref/c2ref;
    fprintf(fid,'\n\n   Cm/alpha:        %-15gE0        %-15g', cma, RStab_Der.Alpha.dcmm_dalpha(1));
    cma = -(E1s(r,c)*cref)/PHI(r,r)/PHI(c,c)/sref/c2ref;
    fprintf(fid,'\n   Cm/alpha:        %-15gE1        %-15g', cma, RStab_Der.Alpha.dcmm_dalpha(1));
%
    cmad = -E2s(r,c)*(cref/VREF)/PHI(c,c)/PHI(r,r)/sref/c2ref;
    fprintf(fid,'\n   Cm/alphad:       %-15gE2        %-15g', cmad, RStab_Der.Alpha.dcmm_dalpha_dot(1));
    cmq = E1s(r,r)/PHI(r,r)^2/sref/c2ref - cmad;
    fprintf(fid,'\n   Cm/q:            %-15gE1,E2     %-15g', cmq, RStab_Der.Q_rate.dcmm_dQ(1));
%
    fprintf(fid,'\n');
  end
%
  Ca_d = [];
  Ca_g = [];
%
  if nSURF
    if isempty(HAD_FILE)
      error('No file for Had fitting given. Use ''Had'' input paramter.');
    else
      [Aa_d,B0_d,B1_d,B2_d,Ca_d,E0_d,E1_d,E2_d,Ca_de,E0_de,E1_de,E2_de,nXa_d] = load_fitting(HAD_FILE, cref, VREF, nUMODES);
      fprintf(fid,'\n - Number of aerodynamic states for Had: %d.', nXa_d);
    end
  end
  if nGUST
    if isempty(HAG_FILE)
      error('No file for Hag fitting given. Use ''Hag'' input paramter.');
    else
      [Aa_g,B0_g,B1_g,B2_g,Ca_g,E0_g,E1_g,E2_g,Ca_ge,E0_ge,E1_ge,E2_ge,nXa_g] = load_fitting(HAG_FILE, cref, VREF, nUMODES);
      fprintf(fid,'\n - Number of aerodynamic states for Hag: %d.', nXa_g);
    end
  end
end
%---------------------------------------------------------------------------------------------------
%
% INPUT DELAY: delta, delta-dot, delta-ddot, vg, vg-dot, vg-ddot, nodal forces
%
input_delay = zeros(nSURF*3+nGUST*3+nLOAD,1);
for i=1:nSURF
  for j=1:3
    input_delay((j-1)*(nSURF)+i) = delay_delta(i);
  end
end 
for i=1:nGUST
  for j=1:3
    input_delay((j-1)*(nGUST)+i+3*nSURF) = delay_vg(i);
  end
end 
for i=1:nLOAD
  input_delay(nSURF*3 + nGUST*3 + i) = delay_f(i);
end 
%---------------------------------------------------------------------------------------------------
%
% structural matrices
% scale modes
MM = PHI' * beam_model.Struct.Mmm(UMODESIND,UMODESIND) * PHI;
KK = PHI' * beam_model.Struct.Kmm(UMODESIND,UMODESIND) * PHI;
CC = PHI' * Bmm(UMODESIND,UMODESIND) * PHI;
% aeroelastic matrices
Mbar = MM - qinfty*E2;
Cbar = CC - qinfty*E1;
Kbar = KK - qinfty*E0;
%
nXatot = nXa + nXa_d + nXa_g;
I = eye(nUMODES);
O = zeros(nUMODES);
Ia = eye(nXa);
Oa = zeros(nUMODES,nXatot);
Oadg = zeros(nXa,nXa_d+nXa_g);
%
% ss system in descriptor form
%
%----------------------------------------------------------------------------------
Eae = [ I,                  O,             Oa; 
        O,               Mbar,             Oa; 
        zeros(nXa,nUMODES), -B2,          Ia, zeros(nXa,nXa_d+nXa_g)];
%
if nSURF 
  Eae = [Eae; zeros(nXa_d,nUMODES*2+nXa), eye(nXa_d)];
end
%
if nGUST 
  Eae = [Eae; zeros(nXa_g,nUMODES*2+nXa+nXa_d), eye(nXa_g)];
end
%----------------------------------------------------------------------------------
Aae = [ O,              I,              Oa; 
       -Kbar,           -Cbar,          qinfty*Ca, qinfty*Ca_d, qinfty*Ca_g; 
       B0,              B1,             Aa, zeros(nXa,nXa_d+nXa_g)];
%
if nSURF 
  Aae = [Aae; zeros(nXa_d,nUMODES*2+nXa), Aa_d];
end
%
if nGUST 
  Aae = [Aae; zeros(nXa_g,nUMODES*2+nXa+nXa_d), Aa_g];
end
% structural roots
%eigval  = eig(MM\KK,'nobalance')
% aeroelastic roots
%nm = size(Eae,1)-nXa_g;
%[V,eigval]  = eig(Eae(1:nm,1:nm)\Aae(1:nm,1:nm),'nobalance');
%diag(eigval)
%V
%return
%
Bae = [];
if nSURF
  Oi = zeros(nUMODES,nSURF);
  Oia = zeros(nXa,nSURF);
  Bae_d = [Oi,          Oi,          Oi;
           qinfty*E0_d, qinfty*E1_d, qinfty*E2_d;
           Oia,         Oia,         Oia;
           B0_d,        B1_d,        B2_d];
  Bae_g = [];
  if nGUST
    Oid = zeros(nXa_g,nSURF);
    Bae_d = [Bae_d; Oid, Oid, Oid]; 
    Oi = zeros(nUMODES,nGUST);
    Oia = zeros(nXa,nGUST);
    Oid = zeros(nXa_d,nGUST);
    Bae_g = [Oi,          Oi,          Oi;
             qinfty*E0_g, qinfty*E1_g, qinfty*E2_g;
             Oia,         Oia,         Oia;
             Oid,         Oid,         Oid;
             B0_g,        B1_g,        B2_g];
  end
  Bae = [Bae_d, Bae_g];
else
  if nGUST
    Oi = zeros(nUMODES,nGUST);
    Oia = zeros(nXa,nGUST);
    Bae =   [Oi,          Oi,          Oi;
             qinfty*E0_g, qinfty*E1_g, qinfty*E2_g;
             Oia,         Oia,         Oia;
             B0_g,        B1_g,        B2_g];
  end
end
%
if  ~isequal(beam_model.Param.LOAD, 0)
  [Baeh, f_label] = set_modal_extload(beam_model.Struct.NDispl(:,:,UMODESIND), ...
                    beam_model.Node.ID, beam_model.Param.LOAD, beam_model.Dextload);
  Baeh2 = [zeros(nUMODES,nLOAD); PHI'*Baeh; zeros(nXatot,nLOAD)];
  Bae = [Bae, Baeh2]; 
end
%
% add gravitational terms
%
%if ~isempty(pitch_mode)
%  x_index = find(beam_model.Node.DOF2(:,1));
%  grav_load = zeros(ndof2,1);
%  grav_load(beam_model.Node.DOF2(x_index,1),1) = beam_model.Param.G;
%  Aae(nUMODES+1:2*nUMODES, 1:nUMODES) = Aae(nUMODES+1:2*nUMODES, 1:nUMODES) + ...
%      PHI' * beam_model.Struct.V(:, UMODESIND)' * beam_model.Struct.M * grav_load  * PHI(pitch_mode,:);  
%end
%
if ~isempty(roll_mode)
  y_index = find(beam_model.Node.DOF2(:,2));
  grav_load = zeros(ndof2,1);
  grav_load(beam_model.Node.DOF2(y_index,1),1) = -beam_model.Param.G;
  Aae(nUMODES+1:2*nUMODES, 1:nUMODES) = Aae(nUMODES+1:2*nUMODES, 1:nUMODES) + ...
      PHI' * beam_model.Struct.V(:, UMODESIND)' * beam_model.Struct.M * grav_load * PHI(roll_mode,:);  
end
%
% Export state, q, qdot, qddot
%
% divide equations to easy accelerations recovery
%
Aae = Eae\Aae;
Bae = Eae\Bae;
%
Cae = zeros(3*nUMODES, 2*nUMODES+nXatot);
Cae = [blkdiag(PHI,PHI), zeros(2*nUMODES,nXatot); PHI*Aae(nUMODES+1:2*nUMODES,:)];
Dae = [zeros(2*nUMODES,nSURF*3+nGUST*3+nLOAD); PHI*Bae(nUMODES+1:2*nUMODES,:)];
%
%---------------------------------------------------------------------------
% create labels
%
state_name = {};
input_name = {};
output_name = {};
input_group = [];
output_group = [];
%
for i=1:nUMODES
  state_name{i} = ['q', num2str(beam_model.Param.UMODES(i))];
  state_name{i+nUMODES} = ['q', num2str(beam_model.Param.UMODES(i)),'-dot'];
  output_name{i} = state_name{i};
  output_name{i+nUMODES} = state_name{i+nUMODES};
  output_name{i+2*nUMODES} = ['q', num2str(beam_model.Param.UMODES(i)),'-ddot'];
end
output_group.q     = [1:nUMODES];
output_group.qdot  = [nUMODES+1:2*nUMODES];
output_group.qddot = [2*nUMODES+1:3*nUMODES];
% offset for extra aero outputs
output_offset = 3*nUMODES;
%-------------------------------------------------------------------------------
% DISPLACEMENTS
if ~isempty(beam_model.Param.DISP)
  ndisp = size(dyn_model.Out.DISP, 1);
  output_group.node_disp = [output_offset+1:output_offset + ndisp];
  Cae = [Cae; dyn_model.Out.DISP(:,UMODESIND) * PHI, zeros(ndisp,nXatot+nUMODES)];
  Dae = [Dae; zeros(ndisp,nSURF*3+nGUST*3+nLOAD)];
  for j=1:6
    for i=1:length(beam_model.Param.DISP)
      output_offset = output_offset+1;
      output_name{output_offset} = ['DISP-', num2str(beam_model.Node.ID(beam_model.Param.DISP(i))),'-',num2str(j)];
    end
  end
end
%-------------------------------------------------------------------------------
% VELOCITY
if ~isempty(beam_model.Param.VELOCITY)
  nvel = size(dyn_model.Out.VELOCITY, 1);
  output_group.node_vel = [output_offset+1:output_offset + nvel];
  Cae = [Cae; zeros(nvel,nUMODES), dyn_model.Out.VELOCITY(:,UMODESIND) * PHI, zeros(nvel,nXatot)];
  Dae = [Dae; zeros(nvel,nSURF*3+nGUST*3+nLOAD)];
  for j=1:6
    for i=1:length(beam_model.Param.VELOCITY)
      output_offset = output_offset+1;
      output_name{output_offset} = ['VEL-', num2str(beam_model.Node.ID(beam_model.Param.VELOCITY(i))),'-',num2str(j)];
    end
  end
end
%-------------------------------------------------------------------------------
% ACCELERATION
if ~isempty(beam_model.Param.ACCELERATION)
  nacc = size(dyn_model.Out.ACCELERATION, 1);
  output_group.node_acc = [output_offset+1:output_offset + nacc];
  Cae = [Cae; dyn_model.Out.ACCELERATION(:,UMODESIND) * PHI * Aae(nUMODES+1:2*nUMODES,:)];
  Dae = [Dae; dyn_model.Out.ACCELERATION(:,UMODESIND) * PHI * Bae(nUMODES+1:2*nUMODES,:)];
  for j=1:6
    for i=1:length(beam_model.Param.ACCELERATION)
      output_offset = output_offset+1;
      output_name{output_offset} = ['ACC-', num2str(beam_model.Node.ID(beam_model.Param.ACCELERATION(i))),'-',num2str(j)];
    end
  end
end
%-------------------------------------------------------------------------------
% BAR INTERNAL LOAD
if ~isempty(beam_model.Param.IFORCE)
  nstress = size(dyn_model.Out.IFORCE, 1);
  output_group.bar_forces = [output_offset+1:output_offset + nstress];
  Cae = [Cae; dyn_model.Out.IFORCE(:,UMODES) * PHI, zeros(nstress,nXatot+nUMODES)];
  Dae = [Dae; zeros(nstress,nSURF*3+nGUST*3+nLOAD)];
  for i=1:length(beam_model.Param.IFORCE)
    for j=1:6
      output_offset = output_offset+1;
      output_name{output_offset} = ['FORCE-BAR-', num2str(beam_model.Node.ID(beam_model.Param.IFORCE(i))),'-',num2str(j),'-C1'];
    end
    for j=1:6
      output_offset = output_offset+1;
      output_name{output_offset} = ['FORCE-BAR-', num2str(beam_model.Node.ID(beam_model.Param.IFORCE(i))),'-',num2str(j),'-C2'];
    end
  end
end
%-------------------------------------------------------------------------------
% BEAM INTERNAL LOAD
if ~isempty(beam_model.Param.IFORCEBE)
  nstress = size(dyn_model.Out.IFORCEBE, 1);
  output_group.beam_forces = [output_offset+1:output_offset + nstress];
  Cae = [Cae; dyn_model.Out.IFORCEBE(:,UMODES) * PHI, zeros(nstress,nXatot+nUMODES)];
  Dae = [Dae; zeros(nstress,nSURF*3+nGUST*3+nLOAD)];
  for i=1:length(beam_model.Param.IFORCEBE)
    for j=1:6
      output_offset = output_offset+1;
      output_name{output_offset} = ['FORCE-BEAM-', num2str(beam_model.Node.ID(beam_model.Param.IFORCEBE(i))),'-',num2str(j),'-C1'];
    end
    for j=1:6
      output_offset = output_offset+1;
      output_name{output_offset} = ['FORCE-BEAM-', num2str(beam_model.Node.ID(beam_model.Param.IFORCEBE(i))),'-',num2str(j),'-C2'];
    end
  end
end
%
%-------------------------------------------------------------------------------
% AERO FORCES/COEFFICIENTS
%
invPHI = inv(PHI');
aero_coeff = [qinfty*E0, qinfty*E1, qinfty*Ca, zeros(nUMODES, nXa_d+nXa_g)] + qinfty * E2 * Aae(nUMODES+1:2*nUMODES,:);
Cae = [Cae; invPHI * aero_coeff];
Dae = [Dae; qinfty * invPHI * E2 * Bae(nUMODES+1:2*nUMODES,:)];
%
output_group.aero_force_mode = [output_offset+1:output_offset + nUMODES];
for j=1:nUMODES
  output_offset = output_offset+1;
  output_name{output_offset} = ['Qload_mode-', num2str(UMODES(j))];
end
%
if nSURF
  aero_coeff = [zeros(nUMODES, 2*nUMODES + nXa), qinfty*Ca_d, zeros(nUMODES, nXa_g)];
  Cae = [Cae; invPHI * aero_coeff];
  Dae = [Dae; qinfty*invPHI*E0_d, qinfty*invPHI*E1_d, qinfty*invPHI*E2_d, zeros(nUMODES,3*nGUST)];
%
  output_group.aero_force_surf = [output_offset+1:output_offset + nUMODES];
  for j=1:nUMODES
    output_offset = output_offset+1;
    output_name{output_offset} = ['Qload_surf-', num2str(UMODES(j))];
  end
end
%
if nGUST
  aero_coeff = [zeros(nUMODES, 2*nUMODES + nXa + nXa_d), qinfty*Ca_g];
  Cae = [Cae; invPHI*aero_coeff];
  Dae = [Dae; zeros(nUMODES,3*nSURF), qinfty*invPHI*E0_g, qinfty*invPHI*E1_g, qinfty*invPHI*E2_g];
%
  output_group.aero_force_gust = [output_offset+1:output_offset + nUMODES];
  for j=1:nUMODES
    output_offset = output_offset+1;
    output_name{output_offset} = ['Qload_gust-', num2str(UMODES(j))];
  end
end
%
if nLOAD
  aero_coeff = [zeros(nUMODES, 2*nUMODES + nXa + nXa_d), qinfty*Ca_g];
  Cae = [Cae; zeros(nUMODES,2*nUMODES + nXa + nXa_d + nXa_g)];
  Dae = [Dae; zeros(nUMODES,3*nSURF+3*nGUST), Dae(nUMODES+1:2*nUMODES,3*nSURF+3*nGUST+1:3*nSURF+3*nGUST+nLOAD)];
%
  output_group.aero_force_ext = [output_offset+1:output_offset + nUMODES];
  for j=1:nUMODES
    output_offset = output_offset+1;
    output_name{output_offset} = ['Qload_ext-', num2str(UMODES(j))];
  end
end
%
coeff_name = {'Cy', 'Cz', 'Cl', 'Cm', 'Cn'};
if ~isempty(Ca_e)
  %
  % add Cy, Cz, Cl, Cm, Cn due to motion  
  %
  E0_em = E0_e(1:5,:); 
  E1_em = E1_e(1:5,:); 
  E2_em = E2_e(1:5,:); 
  Ca_em = Ca_e(1:5,:);
  aero_coeff = [E0_em, E1_em, Ca_em, zeros(5, nXa_d+nXa_g)] + E2_em * Aae(nUMODES+1:2*nUMODES,:);
  Cae = [Cae; aero_coeff];
  Dae = [Dae; E2_em * Bae(nUMODES+1:2*nUMODES,:)];
  %
  output_group.aero_coeff_mode = [output_offset+1:output_offset + 5];
  for j=1:5
    output_offset = output_offset+1;
    output_name{output_offset} = [coeff_name{j},'_mode'];
  end
  %
  % add hinge moments due to motion
  %
  if nSURF
    if ~isempty(beam_model.Param.HINGEFORCE)
      nhinge = length(beam_model.Param.HINGEFORCE);
      E0_eh = qinfty * E0_e(5 + [1:nhinge],:); 
      E1_eh = qinfty * E1_e(5 + [1:nhinge],:); 
      E2_eh = qinfty * E2_e(5 + [1:nhinge],:); 
      Ca_eh = Ca_e(5 + [1:nhinge],:);
      aero_coeff = [E0_eh, E1_eh, Ca_eh, zeros(5, nXa_d+nXa_g)] + E2_eh * Aae(nUMODES+1:2*nUMODES,:);
      Cae = [Cae; -aero_coeff];
      Dae = [Dae; -E2_eh * Bae(nUMODES+1:2*nUMODES,:)];
      %
      output_group.hinge_moment_mode = [output_offset+1:output_offset + nhinge];
      for j=1:nhinge
        output_offset = output_offset+1;
        output_name{output_offset} = ['HF_mode-', dyn_model.beam.Aero.Trim.MasterSurf{beam_model.Param.HINGEFORCE(j)}];
      end
    end
  end
%
end
%
if nSURF
  if ~isempty(Ca_de)
    %
    % add Cy, Cz, Cl, Cm, Cn due to controls  
    %
    E0_em = E0_de(1:5,:); 
    E1_em = E1_de(1:5,:); 
    E2_em = E2_de(1:5,:); 
    Ca_em = Ca_de(1:5,:);
    aero_coeff = [zeros(5, 2*nUMODES + nXa), Ca_em, zeros(5, nXa_g)];
    Cae = [Cae; aero_coeff];
    Dae = [Dae; E0_em, E1_em, E2_em, zeros(nGUST*3 + nLOAD)];
    %
    output_group.aero_coeff_surf = [output_offset+1:output_offset + 5];
    for j=1:5
      output_offset = output_offset+1;
      output_name{output_offset} = [coeff_name{j},'_surf'];
    end
    %
    % add hinge moments due to controls  
    %
    if ~isempty(beam_model.Param.HINGEFORCE)
      nhinge = length(beam_model.Param.HINGEFORCE);
      E0_eh = qinfty * E0_de(5 + [1:nhinge],:); 
      E1_eh = qinfty * E1_de(5 + [1:nhinge],:); 
      E2_eh = qinfty * E2_de(5 + [1:nhinge],:); 
      Ca_eh = Ca_de(5 + [1:nhinge],:);
      aero_coeff = [zeros(5, 2*nUMODES + nXa), Ca_eh, zeros(5, nXa_g)];
      Cae = [Cae; -aero_coeff];
      Dae = [Dae; -E0_eh, -E1_eh, -E2_eh, zeros(nhinge, nGUST*3 + nLOAD)];
      %
      output_group.hinge_moment_surf = [output_offset+1:output_offset + nhinge];
      for j=1:nhinge
        output_offset = output_offset+1;
        output_name{output_offset} = ['HF_surf-',  dyn_model.beam.Aero.Trim.MasterSurf{beam_model.Param.HINGEFORCE(j)}];
      end
    end
  end
end
%
if nGUST
  if ~isempty(Ca_ge)
    %
    % add Cy, Cz, Cl, Cm, Cn due to gusts  
    %
    E0_em = E0_ge(1:5,:); 
    E1_em = E1_ge(1:5,:); 
    E2_em = E2_ge(1:5,:); 
    Ca_em = Ca_ge(1:5,:);
    aero_coeff = [zeros(5, 2*nUMODES + nXa + nXa_d), Ca_em];
    Cae = [Cae; aero_coeff];
    Dae = [Dae;  zeros(5, nSURF*3), E0_em, E1_em, E2_em, zeros(5, nLOAD)];
    %
    output_group.aero_coeff_gust = [output_offset+1:output_offset + 5];
    for j=1:5
      output_offset = output_offset+1;
      output_name{output_offset} = [coeff_name{j},'_gust'];
    end
    %
    % add hinge moments due to gusts  
    %
    if nSURF
      if ~isempty(beam_model.Param.HINGEFORCE)
        nhinge = length(beam_model.Param.HINGEFORCE);
        E0_eh = qinfty * E0_de(5 + [1:nhinge],:); 
        E1_eh = qinfty * E1_de(5 + [1:nhinge],:); 
        E2_eh = qinfty * E2_de(5 + [1:nhinge],:); 
        Ca_eh = qinfty * Ca_de(5 + [1:nhinge],:);
        aero_coeff = [zeros(5, 2*nUMODES + nXa + nXa_d), Ca_em];
        Cae = [Cae; -aero_coeff];
        Dae = [Dae;  zeros(nhinge, nSURF*3), -E0_eh, -E1_eh, -E2_eh, zeros(nhinge, nLOAD)];
        %
        output_group.hinge_moment_gust = [output_offset+1:output_offset + nhinge];
        for j=1:nhinge
          output_offset = output_offset+1;
          output_name{output_offset} = ['HF_gust-', num2str(beam_model.Param.HINGEFORCE(j))];
        end
      end
    end
  end
%
end
%-------------------------------------------------------------------------------
for i=1:nXa
  state_name{2*nUMODES + i} = ['xa_m', num2str(i)];
end
for i=1:nXa_d
  state_name{2*nUMODES + i + nXa} = ['xa_d', num2str(i)];
end
for i=1:nXa_g
  state_name{2*nUMODES + i + nXa_d + nXa} = ['xa_g', num2str(i)];
end
%
if ~isempty(beam_model.Param.SURFDEF)
  for i = 1: nSURF
    input_name{i} = cell2mat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))));
    input_name{i+nSURF} = cell2mat(strcat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))),'-dot'));
    input_name{i+2*nSURF} = cell2mat(strcat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))),'-ddot'));
  end
%
  input_group.delta     = [1:nSURF];
  input_group.deltadot  = [nSURF+1:2*nSURF];
  input_group.deltaddot = [2*nSURF+1:3*nSURF];
%
end
if ~isempty(beam_model.Param.GUST)
  for i = 1:nGUST
    input_name{i+3*nSURF} = ['alphag',num2str(i)];
    input_name{i+3*nSURF+nGUST} = ['alphag',num2str(i),'-dot'];
    input_name{i+3*nSURF+2*nGUST} = ['alphag',num2str(i),'-ddot'];
  end
%
  input_group.gust_alpha     = [3*nSURF+1:3*nSURF+nGUST];
  input_group.gust_alphadot  = [3*nSURF+nGUST+1:3*nSURF+2*nGUST];
  input_group.gust_alphaddot = [3*nSURF+2*nGUST+1:3*nSURF+3*nGUST];
%
end
%
if  ~isequal(beam_model.Param.LOAD, 0)
  nip = size(Baeh,2);
  for i=1:nip
    input_name{i+3*nSURF+3*nGUST} = f_label{i};
  end
%
  input_group.ext_force  = [3*nSURF+3*nGUST+1:3*nSURF+3*nGUST+nip];
%
end
% assembly state model for structure only
ss_str_model = ss(blkdiag(eye(nUMODES), MM)\[zeros(nUMODES), eye(nUMODES); -KK, zeros(nUMODES,nUMODES)], zeros(2*nUMODES,1), eye(2*nUMODES), []);
%
ss_aero_model = ss(Aa, B0, Ca, E0);
%
ss_model_hh = ss(Aae(1:2*nUMODES+nXa,1:2*nUMODES+nXa), [], eye(2*nUMODES+nXa), []);
%
ss_model = ss(Aae, Bae, Cae, Dae, 'StateName', state_name, 'OutputName', output_name, 'OutputGroup', output_group, ...
              'InputName', input_name, 'InputGroup', input_group,'InputDelay', input_delay);
%
fprintf(fid,'\n\n - Structural model roots:\n');
damp(ss_str_model);
fprintf(fid,'\n\n - Aerodynamic model roots:\n');
damp(ss_aero_model);
fprintf(fid,'\n - Aeroelastic model roots:\n');
damp(ss_model_hh);
fprintf(fid,'\n - Aeroelastic model roots + control/gust:\n');
damp(ss_model);
fprintf(fid,'\n');
if CHECK_MODEL
%  [Wn,Z] = damp(ss_model);
%  Wn.*cref/(2*VREF)
%  pzmap(ss_str_model , ss_model_hh, ss_model)
%  grid on;
%  legend('Structural model','Aeroelastic model', 'Aeroelastic model + control/gust');
  X0 = zeros(2*nUMODES + nXa + nXa_d + nXa_g,1);
  X0(nUMODES + CHECK_MODEL,1) = 1.0e-3;
  X0(nUMODES + 1 : 2*nUMODES,1) = PHI\X0(nUMODES + 1 : 2*nUMODES,1);
  [Ys,Ts,Xs] = initial(ss_model, X0, T);
  return
end
%
U = zeros(3*nSURF+3*nGUST+nLOAD,nt);
% control input
if  ~isempty(beam_model.Param.SURFDEF)
  dyn_model.Res.Control_profile = zeros(nSURF, nt); 
  for i = 1: nSURF
    ind = find(T<= beam_model.Surfdef.Tmax(beam_model.Param.SURFDEF(i)) );
    nind = length(ind);
    delay_pos = find(T<delay_delta(i));
    ndelay = 0;
    if ~isempty(delay_pos)
      ndelay = delay_pos(end);
    end
    input_value = zeros(nt,3);
    inde = ind(end);
%   deflection
    fun = Scalar2VectorialFun(beam_model.Surfdef.fun{beam_model.Param.SURFDEF(i)});
    scale_f = beam_model.Surfdef.Amp(beam_model.Param.SURFDEF(i))*pi/180;
    input_value(1:inde,1) = symbolic_function(fun, T(1:inde)).*scale_f;
    input_value(1:inde,2) = symbolic_derivative1(fun, T(1:inde)).*scale_f;
    input_value(1:inde,3) = symbolic_derivative2(fun, T(1:inde)).*scale_f;
    for j=1:3
      U(i+(j-1)*nSURF,:) = input_value(:,j)';
    end
    dyn_model.Res.Control_profile(i,:) = [zeros(ndelay,1); input_value(1:inde,1).*(180/pi); zeros(nt-nind-ndelay,1)]';
  end
end
% Gust
if ~isempty(beam_model.Param.GUST)
  dyn_model.Res.Gust_profile = zeros(nGUST, nt); 
  for i = 1:nGUST
    ind = find(T<= beam_model.Gust.Tmax(beam_model.Param.GUST(i)) );
    nind = length(ind);
    delay_pos = find(T<delay_vg(i));
    ndelay = 0;
    if ~isempty(delay_pos)
      ndelay = delay_pos(end);
    end
    input_value = zeros(nt,3);
    inde = ind(end);
%   vg
    fun = Scalar2VectorialFun(beam_model.Gust.fun{beam_model.Param.GUST(i)});
    scale_f = beam_model.Gust.Amp(beam_model.Param.GUST(i))/VREF;
    input_value(1:inde,1) = symbolic_function(fun,T(1:inde)).*scale_f;
%   vg dot
    input_value(1:inde,2) = symbolic_derivative1(fun,T(1:inde)).*scale_f;
%   vg ddot      
    input_value(1:inde,3) = symbolic_derivative2(fun,T(1:inde)).*scale_f;
%
    for j=1:3
      U(i+nSURF*3+(j-1)*nGUST,:) = input_value(:,j);
    end
    dyn_model.Res.Gust_profile(i,:) = [zeros(ndelay,1); input_value(1:inde,1)*VREF; zeros(nt-nind-ndelay,1)]';
  end
end
if  ~isequal(beam_model.Param.LOAD, 0)
  dyn_model.Res.Extload_profile = zeros(nLOAD, nt); 
  for i = 1: nLOAD
    ind = find(T<= beam_model.Dextload.Tmax(beam_model.Param.LOAD(i)));
    nind = length(ind);
    delay_pos = find(T<delay_f(i));
    ndelay = 0;
    if ~isempty(delay_pos)
      ndelay = delay_pos(end);
    end
    input_value = zeros(nt,1);
    inde = ind(end);
%   force
    fun = Scalar2VectorialFun(beam_model.Dextload.fun{beam_model.Param.LOAD(i)});
%
    U(i+(nSURF+nGUST)*3,1:inde) = symbolic_function(fun, T(1:inde)).*beam_model.Dextload.Amp(beam_model.Param.LOAD(i));
    dyn_model.Res.Extload_profile(i,:) = [zeros(ndelay,1); input_value(1:inde,1); zeros(nt-nind-ndelay,1)]';
  end
end
%
fprintf(fid,'\n');
[Ys,Ts,Xs] = lsim(ss_model, U', T);
%            
else
  fprintf(fid,'\n SOL 146 must be specified in input file.\n');
end

end
%-------------------------------------------------------------------------------
% Determine generalized forces for external input
function [Q, label] = set_modal_extload(NDispl, NID, PLOAD, LOAD)
  nm = size(NDispl,3);
  nLOAD = length(PLOAD);
  Q = zeros(nm,nLOAD);
  label = {};
  for n=1:nLOAD
    ind = PLOAD(n);
    node = LOAD.Node(ind); % node index
    DOF = LOAD.NDOF(ind);  % node DOF 1->6
%   gen forces
    Q(:,n) = squeeze(NDispl(node, DOF, :));
    label{n} = ['Input node ',num2str(NID(node))];
  end
end
%-------------------------------------------------------------------------------
% evaluate function derivative
function value = symbolic_function(fun,time)
  syms t;
  fun = eval(fun);
  value = subs(fun,'t',time);
end
function value = symbolic_derivative1(fun,time)
  syms t;
  fun = eval(fun);
  dfun = diff(fun,'t');
  value = subs(dfun,'t',time);
end
%
function value = symbolic_derivative2(fun,time)
  syms t;
  fun = eval(fun);
  dfun = diff(fun,'t');
  dfun2 = diff(dfun,'t');
  value = subs(dfun2,'t',time);
end
%
function [Aa,B0,B1,B2,Ca_m,E0_m,E1_m,E2_m,Ca_e,E0_e,E1_e,E2_e,nXa,solution] = load_fitting(FILE, cref, VREF, nmodes)
  data = load(FILE);
  solution = data.solution;
  Ca_e = []; E0_e = []; E1_e = []; E2_e = [];
  Aid    = solution.inoutresid.AA;
  Bid{1} = solution.inoutresid.BB{1};
  Bid{2} = solution.inoutresid.BB{2};
  Bid{3} = solution.inoutresid.BB{3};
  Cid    = solution.inoutresid.CC;
  Eid{1} = solution.inoutresid.DD{1};
  Eid{2} = solution.inoutresid.DD{2};
  Eid{3} = solution.inoutresid.DD{3};
  nXa = size(Aid,1);
  E0 = Eid{1};
  E1 = Eid{2}*(cref/VREF)^+1;
  E2 = Eid{3}*(cref/VREF)^+2;
  B0 = Bid{1}.*(cref/VREF)^-1;
  B1 = Bid{2}.*(cref/VREF)^0;
  B2 = Bid{3}.*(cref/VREF)^+1;
  Aa = Aid.*(cref/VREF)^-1;
  Ca = Cid;
%
  E0_m = E0(1:nmodes,:);
  E1_m = E1(1:nmodes,:);
  E2_m = E2(1:nmodes,:);
  E0_e = E0(nmodes+1:end,:);
  E1_e = E1(nmodes+1:end,:);
  E2_e = E2(nmodes+1:end,:);
%
  Ca_m = Ca(1:nmodes,:);
  Ca_e = Ca(nmodes+1:end,:);
%
end