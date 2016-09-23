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
% Solve dynamic response for free aircraft
%
% Additional inputs: dT, Tmax
% dT: time step [s]
% Tmax: time period length [s]
% Example: solve_free_lin_dyn('Tmax',5,'dT',0.01);
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
function [Ys,Ts,Xs,U]=solve_free_lin_dyn_ss_fm(varargin)

global dyn_model
global fl_model
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
  HAMR_FILE = [];
  HAME_FILE = [];
  DRAG_FILE = [];
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
%
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
  Xsup = beam_model.Node.Coord(beam_model.Param.SUPORT(1) == beam_model.Node.ID,:);
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
% check if x mode used
  XMODE = ~isempty(find(UMODES == 1));
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
      if (strcmp(PARAM(n), 'HamR')) 
        HAMR_FILE = PARAM{n+1};
        fprintf(fid,'\n - Filename with aerodynamic fitting of rigid dynamics + inputs: %s.', HAMR_FILE);
        if ~exist(HAMR_FILE,'file')
          error(['Unable to find file ',char(PARAM(n+1)),'.']);
        end
        param_set = 1;
      end
      if (strcmp(PARAM(n), 'HamE')) 
        HAME_FILE = PARAM{n+1};
        fprintf(fid,'\n - Filename with aerodynamic fitting of deformable dynamics: %s.', HAME_FILE);
        if ~exist(HAME_FILE,'file')
          error(['Unable to find file ',char(PARAM(n+1)),'.']);
        end
        param_set = 1;
      end
      if (strcmp(PARAM(n), 'drag_data')) 
        DRAG_FILE = PARAM{n+1};
        fprintf(fid,'\n - Filename with aerodynamic drag data: %s.', DRAG_FILE);
        if ~exist(DRAG_FILE,'file')
          error(['Unable to find file ',char(PARAM(n+1)),'.']);
        end
        param_set = 1;
      end
      if ~param_set
        error(['Unknown input parameter: ', char(PARAM(n)),'.']);
      end
    end
  end
%
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
% TRANSFORMATION. rotate x,z axes
%
PHIR = eye(6);
PHIR(1,1) = -1; %-x
PHIR(3,3) = -1; %-z 
PHIR(4,4) = -1; %-phi
PHIR(6,6) = -1; %-psi
%
PHIRT = eye(6);
PHIRT(1:3,4:6) = crossm(beam_model.WB.CG - Xsup);
%
ni = nSURF + nGUST;
if isempty(HAMR_FILE)
%
  rmodes = find(mbase <= 6);
  emodes = find(mbase > 6);
  %
  if (length(rmodes)<6)
    error('Rigid modes 1->6 must be all declared in MSELECT set.');
  end
  %
  PHIE = eye(length(emodes));
  PHI = blkdiag(PHIR, PHIE);
  PHICG = blkdiag(PHIRT, PHIE);
  %
  Had = [];
  Hag = [];
  % use ALL body modes for conversion
  HamR = dyn_model.dlm.data.Qhh(:,rmodes,:,MINDEX);
  for i=1:nk
  % transform Ham to body axes
    HamR(:,:,i) = beam_model.Struct.Mmm\(PHI' * PHICG' * HamR(:,:,i) * PHIRT * PHIR);
%    HamR(:,:,i) = PHI' * HamR(:,:,i) * PHIR;
  end
  % transform variables to: u, alpha, beta, p, q, r
  [HamRsa, k2] = iner2sa_neoresp(HamR, dyn_model.dlm.aero.k, cref, 4);
  %
  PHIR = PHIR(UMODESRLOC,UMODESRLOC);
  PHIRT = PHIRT(UMODESRLOC,UMODESRLOC);
%
  PHIE = eye(ne);
  PHI = blkdiag(PHIR, PHIE);
  PHICG = blkdiag(PHIRT, PHIE);
%
  HamE = [];
  if ne
    HamE = dyn_model.dlm.data.Qhh(UMODESIND,UMODESEGLOB,:,MINDEX);
    for i=1:nk
      HamE(:,:,i) = beam_model.Struct.Mmm(UMODESIND,UMODESIND)\(PHI' * PHICG' * HamE(:,:,i));
%      HamE(:,:,i) = PHI' * HamE(:,:,i);
    end
  end
  %
  if  ~isempty(beam_model.Param.SURFDEF)
    Had = dyn_model.dlm.data.Qhd(UMODESIND,beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF),:, MINDEX);
    for i=1:nk
      Had(:,:,i) = beam_model.Struct.Mmm(UMODESIND,UMODESIND)\ (PHI' * PHICG' * Had(:,:,i));
%      Had(:,:,i) = PHI' * Had(:,:,i);
    end
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
      for j=1:nk
        Hag(:,i,j) = beam_model.Struct.Mmm(UMODESIND,UMODESIND)\(PHI' * PHICG' * dyn_model.gust.Qhg(UMODESIND,:,j,MINDEX) * Qhg(:,j));
%        Hag(:,i,j) = PHI'*dyn_model.gust.Qhg(UMODESIND,:,j,MINDEX) * Qhg(:,j);
      end
    end
  end
%
% AERO DATA
%
  minus2 = setdiff([1:nk+1],2);
  k = k2(minus2);
% avoid export zero terms for fitting
  if XMODE
    Ha = HamRsa(UMODESIND(2:end),UMODESRGLOB(2:end),minus2);
  else
    Ha = HamRsa(UMODESIND,UMODESRGLOB,minus2);
  end
%
  filename = [headname,'_Ham_M_',num2str(MREF),'_R.mat'];
% append input columns
  if nSURF
    if XMODE
      Ha = [Ha, Had(2:end,:,:)];
    else
      Ha = [Ha, Had];
    end
  end
  if nGUST
    if XMODE
      Ha = [Ha, Hag(2:end,:,:)];
    else
      Ha = [Ha, Hag];
    end
  end
  save(filename,'k','Ha');
  fprintf(fid, '\n\n - Aerodynamic matrix exported to %s file for first order fitting.', filename);
  fprintf(fid,'\n   Rows: %d.', nUMODES);
  fprintf(fid,'\n   Columns: %d.', length(UMODESRGLOB));
  fprintf(fid,'\n   Columns appended for inputs: %d.', ni);
%
  k = dyn_model.dlm.aero.k;
%
  if ne
    Ha = HamE;
    filename = [headname,'_Ham_M_',num2str(MREF),'_E.mat'];
    save(filename,'k','Ha');
    fprintf(fid, '\n\n - Aerodynamic matrix exported to %s file for second order fitting.', filename);
    fprintf(fid,'\n   Rows: %d.', nUMODES);
    fprintf(fid,'\n   Columns: %d.', length(UMODESEGLOB));
  end
%
  fprintf(fid, '\n - Transform aero matrix to stability axes frame.');
  fprintf(fid, '\n - Use the panel GUI to fit aerodynamic data.');
  fprintf(fid, '\n - When fitting is concluded, run the solver with ''aero_file'' parameter followed by the output file created.\n');
%
%  mygui;
%[Aid, Bid, Cid, Eid] = aero_ss(k, Ha);
  return
else
%
  if (ne && isempty(HAME_FILE))
    error('No aerodynamic fitting file for elastic modes given. Use input parameter ''HamE'' or change UMODES parameter.')
  end
% number of states
  nXaR   = 0;
% initialize to null values elastic dofs
  nXaE   = 0;
  AaE=[]; B0E=[]; B1E=[]; B2E=[]; CaE=[]; E0E=[]; E1E=[]; E2E=[];
%
  fprintf(fid,'\n - Number of rigid states: %d.', 2*nr);
  [AaR,B0R,B1R,~,CaR,E0R,E1R,~,nXaR] = load_fitting(HAMR_FILE, cref, VREF);
  fprintf(fid,'\n - Number of aerodynamic states: %d.', nXaR);
  fprintf(fid,'\n - Number of inputs: %d.', ni);
%
% append zero terms for xmode
%
  if XMODE
%   null forces
    E0R = blkdiag(0, E0R);
    E1R = blkdiag(0, E1R);
    CaR = [zeros(1,nXaR); CaR];
%   null aero inputs
    B0R = [zeros(nXaR,1), B0R];
    B1R = [zeros(nXaR,1), B1R];
  end
%
  CaR = beam_model.Struct.Mmm(UMODESIND,UMODESIND) * CaR;
  E0R = beam_model.Struct.Mmm(UMODESIND,UMODESIND) * E0R;
  E1R = beam_model.Struct.Mmm(UMODESIND,UMODESIND) * E1R;
%
  if XMODE
    CaR(1,:) = 0.0;
    E0R(1,:) = 0.0; E0R(:,1) = 0.0;
    E1R(1,:) = 0.0; E1R(:,1) = 0.0;
%   add drag contribution to E0R matrix
    CL0 = beam_model.WB.MCG(1,1) * beam_model.Param.G / qinfty / sref
    [CD0, CDA, CDu, CLu, Cmu] = load_drag_file(DRAG_FILE);
%
    fprintf(fid,'\n - Data used for x-mode:');
    fprintf(fid,'\n\t CL0: %f.', CL0);
    fprintf(fid,'\n\t CD0: %f.', CD0);
    fprintf(fid,'\n\t CDA: %f.', CDA);
    fprintf(fid,'\n\t CDu: %f.', CDu);
    fprintf(fid,'\n\t CLu: %f.', CLu);
    fprintf(fid,'\n\t Cmu: %f.', Cmu);
%
    E0R(1,1) =  (-2*CD0 - CDu) * sref;
    plunge_index = find(UMODES ==3);  
    pitch_index = find(UMODES ==5);  
%
    fprintf(fid,'\n - Rigid aero derivatives:');
    CLA = -E0R(plunge_index, plunge_index)/sref
    CMA = E0R(pitch_index, plunge_index)/sref/(cref*2)
%
    CLQstar = -E0R(plunge_index, pitch_index)/sref/(cref)
    CMQstar =  E0R(pitch_index, pitch_index)/sref/(cref*2)/cref

%
    if ~isempty(plunge_index)
      E0R(1,plunge_index) = CL0 - CDA;
      E0R(plunge_index,1) = (-2*CL0 +CLu)*sref;
      E0R(plunge_index,plunge_index) = E0R(plunge_index,plunge_index) -CD0*sref;  
    end
    if ~isempty(pitch_index)
        E0R(pitch_index,1) = Cmu*sref*(cref*2);  
    end
  end
%
E0R
  if ne
    fprintf(fid,'\n - Number of elastic states: %d.', 2*ne);
    [AaE,B0E,B1E,B2E,CaE,E0E,E1E,E2E,nXaE] = load_fitting(HAME_FILE, cref, VREF);
    fprintf(fid,'\n - Number of aerodynamic states: %d.', nXaE);
    CaE = beam_model.Struct.Mmm(UMODESIND,UMODESIND) * CaE;
    E0E = beam_model.Struct.Mmm(UMODESIND,UMODESIND) * E0E;
    E1E = beam_model.Struct.Mmm(UMODESIND,UMODESIND) * E1E;
  end
%
end
%
% scale rigid states to have u,v,w,p,q,r multiplying aero matrices
PHIV = (1/VREF).*eye(nr);
%
% update mass matrix to account for new reference frame (all rigid modes)
MASS = PHIR' * PHIRT' * beam_model.Struct.Mmm(1:6,1:6) * PHIRT * PHIR;

MASS = PHIR' * beam_model.WB.MCG * PHIR;

MASS(1:6,1:6)

S = MASS(4:6,1:3)
MdOMEGA1 = MASS(1,1).*crossm([VREF 0 0]);
MdOMEGA2 = crossm(S * [VREF 0 0]') - crossm([VREF 0 0]) * S';
MG1 = [0, -beam_model.Param.G, 0; 
       beam_model.Param.G, 0 0; 
       0 0 0];
%
MG2 = S * MG1;
MG1 = MASS(1,1) * MG1;
%
% find transl modes
[~, itu, jt] = intersect(UMODES, [1 2 3]);
% find rot modes
[~, iru, jr] = intersect(UMODES, [4 5 6]);
MdOMEGA1 = MdOMEGA1(jt, jr);
MdOMEGA2 = MdOMEGA2(jr, jr);
MG1 = MG1(jt, jr);
MG2 = MG2(jr, jr);
% update mass matrix to account for new reference frame (UMODES only)
PHIR = PHIR(UMODESRLOC,UMODESRLOC);
PHIRT = PHIRT(UMODESRLOC,UMODESRLOC);
PHIE = eye(ne);
PHI = blkdiag(PHIR, PHIE);
PHICG = blkdiag(PHIRT, PHIE);
MRR = MASS(UMODESRGLOB,UMODESRGLOB)
MASS = PHI' * PHICG' * beam_model.Struct.Mmm(UMODESIND,UMODESIND) * PHICG * PHI;


MEE = [];
% index for rigid modes
ir = [1:nr];
MbarR = MRR - qinfty*E1R(ir,ir)*PHIV;
%
MbarE = []; CbarE = []; KbarE = []; 
%
if ne
  % index for elastic modes
  ie = [nr+1:nr+ne];
  MEE = MASS(UMODESELOC,UMODESELOC);
  MbarE = MEE - qinfty*E2E(ie,:);
  CbarE = Bmm(UMODESEGLOB,UMODESEGLOB)  - qinfty*E1E(ie,:);
  KbarE = beam_model.Struct.Kmm(UMODESEGLOB,UMODESEGLOB) - qinfty*E0E(ie,:);
end
if ne
%       xdot              vdot                       qdot              qddot                xaR             xaE    
  Eae = [ eye(nr),         zeros(nr,nr+2*ne+nXaR+nXaE);                                                                           % xdot
          zeros(nr),       MbarR,                     zeros(nr,ne),     -qinfty*E2E(ir,:), zeros(nr, nXaR+nXaE);              % vdot
          zeros(ne,2*nr),                             eye(ne),          zeros(ne,ne+nXaR+nXaE);                                %qdot
          zeros(ne,nr)     -qinfty*E1R(ie,ir)*PHIV,  zeros(ne,ne),     MbarE,              zeros(ne, nXaR+nXaE);               % qddot
          zeros(nXaR,nr),  -B1R(:,ir)*PHIV,          zeros(nXaR,2*ne),                     eye(nXaR),      zeros(nXaR,nXaE); % XaRdot
          zeros(nXaE,2*nr+ne),                                          -B2E,               zeros(nXaE,nXaR), eye(nXaE)];       % XaEdot
  %       xdot              vdot                       qdot              qddot              xaR                 xaE    
  Aae = [ zeros(nr),    eye(nr),                   zeros(nr, 2*ne+ nXaR+nXaE);                                      % x
          zeros(nr),     qinfty*E0R(ir,ir)*PHIV,   qinfty*E0E(ir,:), qinfty*E1E(ir,:),    CaR(ir,:),          CaE(ir,:);     % v
          zeros(ne,2*nr+ne),                                              eye(ne),         zeros(ne, nXaR+nXaE);     %q
          zeros(ne,nr),     qinfty*E0R(ie,ir)*PHIV,  -KbarE,            -CbarE,            CaR(ie,:),          CaE(ie,:);      % qdot
          zeros(nXaR,nr),   B0R(:,ir)*PHIV,           zeros(nXaR,2*ne),                   AaR,                 zeros(nXaR,nXaE);    % XaR
          zeros(nXaE,2*nr),                           B0E,               B1E,              zeros(nXaE,nXaR),    AaE];    % XaE
else
%       xdot              vdot                       qdot              qddot            
  Eae = [ eye(nr),         zeros(nr,nr+nXaR);                                           
          zeros(nr),       MbarR,                     zeros(nr, nXaR);              
          zeros(nXaR,nr),  -B1R(:,ir)*PHIV,                               eye(nXaR)]; 
%         x               v                        XaR         
  Aae = [ zeros(nr),      eye(nr),                zeros(nr, nXaR);                                      
          zeros(nr),      qinfty*E0R(ir,ir)*PHIV, CaR(ir,:);
          zeros(nXaR,nr), B0R(:,ir)*PHIV,         AaR];    
end
%-------------------------------------------------------------------------------
% overwrite first rows and get displacements in inertial frame
% XI_dot = R0 * (VB - VB0x theta)
R0 = -eye(3); R0(2,2) = 1;
XI2 = -R0 * crossm([VREF, 0 0]);
Aae(itu, iru)    = XI2(jt,jr); % -RO * VB0x theta
Aae(itu, nr+itu) = R0(jt,jt); % RO * VB
%-------------------------------------------------------------------------------
% add weight contribution to transl DOFs due to rotation
Aae(nr + itu, iru) = Aae(nr + itu, iru) + MG1;
% add weight contribution to rotat DOFs due to rotation when ref point is not CG
%Aae(nr + iru, iru) = Aae(nr + iru, iru) + MG2;
% add body terms due to omega
Aae(nr + itu, nr + iru) = Aae(nr + itu, nr + iru) + MdOMEGA1;
% add body terms when ref point is not CG
%Aae(nr + iru, nr + iru) = Aae(nr + iru, nr + iru) + MdOMEGA2;
%---------------------------------------------------------------------------------------------------
%
% INPUT DELAY: delta, delta-dot, vg, vg-dot, nodal forces
%
ninp = nSURF*2+nGUST*2+nLOAD;
input_delay = zeros(ninp,1);
for i=1:nSURF
  for j=1:2
    input_delay((j-1)*(nSURF)+i) = delay_delta(i);
  end
end 
for i=1:nGUST
  for j=1:2
    input_delay((j-1)*(nGUST)+i+2*nSURF) = delay_vg(i);
  end
end 
for i=1:nLOAD
  input_delay(nSURF*2 + nGUST*2 + i) = delay_f(i);
end 
%---------------------------------------------------------------------------------------------------
Bae = []; BaeS = []; BaeG = []; BaeL = [];
if nSURF
  if ne
    BaeS = [ zeros(nr,nSURF*2);
        qinfty*E0R(ir,nr+1:nr+nSURF), qinfty*E1R(ir,nr+1:nr+nSURF);
        zeros(ne,nSURF*2);
       qinfty*E0R(ie,nr+1:nr+nSURF), qinfty*E1R(ie,nr+1:nr+nSURF);
       B0R(:,nr+1:nr+nSURF),     B1R(:,nr+1:nr+nSURF);
       zeros(nXaE, nSURF*2)];
  else
    BaeS = [ zeros(nr,nSURF*2);
        qinfty*E0R(ir,nr+1:nr+nSURF), qinfty*E1R(ir,nr+1:nr+nSURF);
       B0R(:,nr+1:nr+nSURF),     B1R(:,nr+1:nr+nSURF)];
  end
end
%
if nGUST
  if ne
    BaeG = [ zeros(nr,nGUST*2);
        qinfty*E0R(ir,nr+1+nSURF:nr+nSURF+nGUST), qinfty*E1R(ir,nr+1+nSURF:nr+nSURF+nGUST);
        zeros(ne,nGUST*2);
       qinfty*E0R(ie,nr+1+nSURF:nr+nSURF+nGUST), qinfty*E1R(ie,nr+1+nSURF:nr+nSURF+nGUST);
       B0R(:,nr+1+nSURF:nr+nSURF+nGUST), B1R(:,nr+1+nSURF:nr+nSURF+nGUST);
       zeros(nXaE, nGUST*2)];
  else
    BaeG = [ zeros(nr,nGUST*2);
             qinfty*E0R(ir,nr+1+nSURF:nr+nSURF+nGUST), qinfty*E1R(ir,nr+1+nSURF:nr+nSURF+nGUST);
             B0R(:,nr+1+nSURF:nr+nSURF+nGUST), B1R(:,nr+1+nSURF:nr+nSURF+nGUST)];
  end
end
if  ~isequal(beam_model.Param.LOAD, 0)
  [Baeh, f_label] = set_modal_extload(beam_model.Struct.NDispl(:,:,UMODESIND), ...
                    beam_model.Node.ID, beam_model.Param.LOAD, beam_model.Dextload);
  Baeh = PHI' * PHICG' * Baeh;
  if ne
    BaeL = [zeros(nr,nLOAD); Baeh(UMODESRLOC,:); zeros(ne,nLOAD); Baeh(UMODESELOC,:); zeros(nXaR+nXaE,nLOAD)];
  else
    BaeL = [zeros(nr,nLOAD); Baeh(UMODESRLOC,:); zeros(nXaR,nLOAD)];
  end
end
%
Bae = [BaeS, BaeG, BaeL]; 
%
% Export state, q, qdot, qddot
%
Cae = zeros(3*nUMODES, 2*nUMODES+nXaR+nXaE);
Cae = [eye(2*nUMODES), zeros(2*nUMODES,+nXaR+nXaE);];% Aae(nUMODES+1:2*nUMODES,:)\Eae(nUMODES+1:2*nUMODES,:)];
Dae = [zeros(2*nUMODES,ninp)];% Bae(nUMODES+1:2*nUMODES,:)\Eae(nUMODES+1:2*nUMODES,:)];
%
%---------------------------------------------------------------------------
% create labels
%
state_name = {};
input_name = {};
output_name = {};
for i=1:nUMODES
  state_name{i} = ['q', num2str(beam_model.Param.UMODES(i))];
  state_name{i+nUMODES} = ['q', num2str(beam_model.Param.UMODES(i)),'-dot'];
  output_name{i} = state_name{i};
  output_name{i+nUMODES} = state_name{i+nUMODES};
%  output_name{i+2*nUMODES} = ['q', num2str(beam_model.Param.UMODES(i)),'-ddot'];
end
%
for i=1:nXaR
  state_name{2*nUMODES + i} = ['xaR', num2str(i)];
end
for i=1:nXaE
  state_name{2*nUMODES + i + nXaR} = ['xaE', num2str(i)];
end
%
if ~isempty(beam_model.Param.SURFDEF)
  for i = 1: nSURF
%    input_name{i} = cell2mat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))));
%    input_name{i+nSURF} = cell2mat(strcat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))),'-dot'));
input_name{i} = 'control1';
input_name{i+nSURF} = 'control1v';

  end
end
if ~isempty(beam_model.Param.GUST)
  for i = 1:nGUST
    input_name{i+2*nSURF} = ['alphag',num2str(i)];
    input_name{i+2*nSURF+nGUST} = ['alphag',num2str(i),'-dot'];
  end
end
%
if  ~isequal(beam_model.Param.LOAD, 0)
  nip = size(Baeh,2);
  for i=1:nip
    input_name{i+2*nSURF+2*nGUST} = f_label{i};
  end
end
%
ss_model = dss(Aae, Bae, Cae, Dae, Eae, 'StateName', state_name, 'OutputName', output_name, 'InputName', input_name, 'InputDelay', input_delay);


%ss_model = ss(Eae\Aae, [], [], []);
%
U = zeros(2*nSURF+2*nGUST+nLOAD,nt);
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
    for j=1:2
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
    for j=1:2
      U(i+nSURF*2+(j-1)*nGUST,:) = input_value(:,j);
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
    U(i+(nSURF+nGUST)*2,1:inde) = symbolic_function(fun, T(1:inde)).*beam_model.Dextload.Amp(beam_model.Param.LOAD(i));
    dyn_model.Res.Extload_profile(i,:) = [zeros(ndelay,1); input_value(1:inde,1); zeros(nt-nind-ndelay,1)]';
  end
end
%
X0 = zeros(2*nUMODES+nXaR,1);
%X0(3) = 0.1;
X0(5) = VREF/57.3;

%[Ys,Ts,Xs] = initial(ss_model, X0, T);
disp('ciaop')
%eigval  = eig(Eae\Aae,'nobalance')

[Ys,Ts,Xs] = lsim(ss_model, U', T);
%save('pippo.mat')
Eae
Aae
return




% OUTPUT
% Force terms
% nodal force
  LoadE = zeros(nLOAD, nt); 
% gust + controls
  Qload  = zeros(nUMODES, nGUST*np + nSURF, nk); % nmodes x nforce
  Load   = zeros(nGUST*np + nSURF, nt);          % nforce x nt 
  if (MODACC ==0)
    QloadR = zeros(ndof,nGUST*np+nSURF,nk);
  end
% MODAL CP
  if ~isempty(beam_model.Param.AEROFORCE)
    CPmode = dyn_model.dlm.data.Cp(beam_model.Param.AEROFORCE, UMODESIND, :, MINDEX);
  end
% Aero forces
  [Cy_mode, Cz_mode, Cl_mode, Cm_mode, Cn_mode] = rigid_aero_force(dyn_model.dlm.data.Cp(:,UMODESIND,:,MINDEX), ...
                                                    dyn_model, cref, bref, sref, Xsup);
% Hinge moments
  if ~isempty(beam_model.Param.HINGEFORCE)
    HFmode = dyn_model.dlm.data.Qdh(beam_model.Param.HINGEFORCE, UMODESIND, :, MINDEX);
  end
%-------------------------------------------------------------------------------
% FORCING TERMS
%
% nodal force
  if  ~isequal(beam_model.Param.LOAD, 0)
    dyn_model.Res.Extload_profile = zeros(nLOAD, nt); 
    for i = 1: nLOAD
      ftime = zeros(nt,1);
      DELAY = abs(beam_model.Dextload.X0(beam_model.Param.LOAD(i)));
      ind = find(T >= DELAY & T<= DELAY + beam_model.Dextload.Tmax(beam_model.Param.LOAD(i)));
      t = T(ind) - DELAY;
      fun = Scalar2VectorialFun(beam_model.Dextload.fun{beam_model.Param.LOAD(i)});
      ftime(ind) = eval(fun) * beam_model.Dextload.Amp(beam_model.Param.LOAD(i));
      dyn_model.Res.Extload_profile(i,:) = ftime;
      ftime = [ftime(1:nt-1);-ftime];
      fome = fft(ftime);
      LoadE(i,:) = fome(1:nt);
    end
  end
% control input
  if  ~isempty(beam_model.Param.SURFDEF)
    dyn_model.Res.Control_profile = zeros(nSURF, nt); 
    for i = 1: nSURF
      ftime = zeros(nt,1);
      DELAY = abs(beam_model.Surfdef.X0(beam_model.Param.SURFDEF(i)));
      ind = find(T >= DELAY & T<= DELAY + beam_model.Surfdef.Tmax(beam_model.Param.SURFDEF(i)));
      t = T(ind) - DELAY;
      fun = Scalar2VectorialFun(beam_model.Surfdef.fun{beam_model.Param.SURFDEF(i)});
      ftime(ind) = eval(fun) * beam_model.Surfdef.Amp(beam_model.Param.SURFDEF(i));
      dyn_model.Res.Control_profile(i,:) = ftime;
%     transfrom to radians
      ftime = ftime * pi/180;
      ftime = [ftime(1:nt-1);-ftime];
      fome = fft(ftime);
      Load(i,:) = fome(1:nt);
      Qload(:,i,:) = dyn_model.dlm.data.Qhd(UMODESIND,beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i)),:, MINDEX);
      if (MODACC==0)
        QloadR(:,i,:) = dyn_model.dlm.data.Qnd(:,beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i)), :, MINDEX);
      end
    end
    if ~isempty(beam_model.Param.AEROFORCE)
      CPsurf = dyn_model.dlm.data.Cp(beam_model.Param.AEROFORCE, NMODES + beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i)), :, MINDEX);
      CP2surf = zeros(length(beam_model.Param.AEROFORCE), nSURF, nk*2);
      CP2surf(:,:,1:2:nk*2) = CPsurf;
      [dummy, dummy, dummy, CPsurf] = spline_coefficients_vec(dyn_model.dlm.aero.k, CP2surf);
    end
    if ~isempty(beam_model.Param.HINGEFORCE)
      HFsurf = dyn_model.dlm.data.Qdd(beam_model.Param.HINGEFORCE,beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF), :, MINDEX);
      HF2surf = zeros(length(beam_model.Param.HINGEFORCE),nSURF,nk*2);
      HF2surf(:,:,1:2:nk*2) = HFsurf;
      [dummy, dummy, dummy, HFsurf] = spline_coefficients_vec(dyn_model.dlm.aero.k, HF2surf);
    end
%   Aero forces
    [Cy_surf, Cz_surf, Cl_surf, Cm_surf, Cn_surf] = ...
                      rigid_aero_force(dyn_model.dlm.data.Cp(:,NMODES+beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF),:,MINDEX),...
                                       dyn_model, cref, bref, sref, Xsup);
  end
% Gust
  if ~isempty(beam_model.Param.GUST)
    dyn_model.Res.Gust_profile = zeros(nGUST, nt); 
    for i = 1:nGUST
      ftime = zeros(nt,1);
      ind = find(T>=0 & T<= beam_model.Gust.Tmax(beam_model.Param.GUST(i)) );
      t = T(ind);
      fun = Scalar2VectorialFun(beam_model.Gust.fun{beam_model.Param.GUST(i)});
      ftime(ind) = eval(fun)*beam_model.Gust.Amp(beam_model.Param.GUST(i));
      dyn_model.Res.Gust_profile(i,:) = ftime;
      ftime = [ftime(1:nt-1);-ftime];
      fome = fft(ftime);
      Load(nSURF+1+(i-1)*np : nSURF + i*np,:) = ...
        exp(-( midPoint(:,1) -beam_model.Gust.X0(beam_model.Param.GUST(i)) )*(1i*k/cref) )...
        .* (dyn_model.gust.dwnwash(:,beam_model.Param.GUST(i)) *  conj(fome(1:nt)'))./VREF ; 
      Qload(:,nSURF+1+(i-1)*np : nSURF + i*np,:) = dyn_model.gust.Qhg(UMODESIND,:,:,MINDEX);
      if (MODACC==0)
        QloadR(:, nSURF+1+(i-1)*np : nSURF + i*np,:) = dyn_model.gust.Qng(:,:,:,MINDEX);
      end
    end
    if ~isempty(beam_model.Param.AEROFORCE)
      CPgust = dyn_model.gust.Cp(beam_model.Param.AEROFORCE,:,:,MINDEX);
      CP2gust = zeros(length(beam_model.Param.AEROFORCE),np,nk*2);
      CP2gust(:,:,1:2:nk*2) = CPgust;
      [dummy, dummy, dummy, CPgust] = spline_coefficients_vec(dyn_model.dlm.aero.k, CP2gust);
    end
    if ~isempty(beam_model.Param.HINGEFORCE)
      HFgust = dyn_model.gust.Qdg(beam_model.Param.HINGEFORCE,:,:,MINDEX);
      HF2gust = zeros(length(beam_model.Param.HINGEFORCE),np,nk*2);
      HF2gust(:,:,1:2:nk*2) = HFgust;
      [dummy, dummy, dummy, HFgust] = spline_coefficients_vec(dyn_model.dlm.aero.k, HF2gust);
    end
%   Aero forces
    [Cy_gust, Cz_gust, Cl_gust, Cm_gust, Cn_gust] = rigid_aero_force(dyn_model.gust.Cp(:,:,:,MINDEX), dyn_model, cref, bref, sref, Xsup);
  end
%
% Assembly aerodynamic matrices
% HAM: modal aero 
  Ham = zeros(nUMODES,nUMODES,nk*2);
  Ham(:,:,1:2:nk*2) = dyn_model.dlm.data.Qhh(UMODESIND,UMODESIND,:,MINDEX);
  [dummy, dummy, dummy, Ham] = spline_coefficients_vec(dyn_model.dlm.aero.k, Ham);
% HAL: modal aero for inputs
  Hal = zeros(nUMODES,nSURF+nGUST*np,nk*2);
  Hal(:,:,1:2:nk*2) = Qload;
  [dummy, dummy, dummy, Hal] = spline_coefficients_vec(dyn_model.dlm.aero.k, Hal);
  if (MODACC == 0)
%   HDM: nodal aero
    Hdm = zeros(ndof,nUMODES,nk*2);
    Hdm(:,:,1:2:nk*2) = dyn_model.dlm.data.Qnh(:,UMODESIND,:,MINDEX);
    [dummy, dummy, dummy, Hdm] = spline_coefficients_vec(dyn_model.dlm.aero.k, Hdm);
%   HDM: nodal aero for inputs
    Hdl = zeros(ndof,nSURF+nGUST*np,nk*2);
    Hdl(:,:,1:2:nk*2) = QloadR;
    [dummy, dummy, dummy, Hdl] = spline_coefficients_vec(dyn_model.dlm.aero.k, Hdl);
  end
%
  if ~isempty(beam_model.Param.AEROFORCE)
    CP2mode = zeros(length(beam_model.Param.AEROFORCE),nUMODES,nk*2);
    CP2mode(:,:,1:2:nk*2) = CPmode;
    [dummy, dummy, dummy, CPmode] = spline_coefficients_vec(dyn_model.dlm.aero.k, CP2mode);
  end
%
  if ~isempty(beam_model.Param.HINGEFORCE)
    HF2mode = zeros(length(beam_model.Param.HINGEFORCE),nUMODES,nk*2);
    HF2mode(:,:,1:2:nk*2) = HFmode;
    [dummy, dummy, dummy, HFmode] = spline_coefficients_vec(dyn_model.dlm.aero.k, HF2mode);
  end
%-------------------------------------------------------------------------------
% Frequency response  
%
  Q          = zeros(nUMODES, nt);
  Qdot       = zeros(nUMODES, nt);
  Qddot      = zeros(nUMODES, nt);
  L_tot_Mode = zeros(nUMODES, nt);
  L_tot_Extf  = zeros(nUMODES, nt);
  L_tot       = zeros(nUMODES, nt);
%
  Cyome_mode = zeros(1,nt);
  Czome_mode = zeros(1,nt);
  Clome_mode = zeros(1,nt);
  Cmome_mode = zeros(1,nt);
  Cnome_mode = zeros(1,nt);
%
  Omega(1) = 0.1;
  for i = 2 : nt
    MASS = zeros(nUMODES,nUMODES);
    MASS = [1i*Omega(i).*beam_model.Struct.Mmm(UMODESRGLOB,UMODESRGLOB), zeros(nr, ne); zeros(ne,nr),  ...
            -Omega(i)^2*beam_model.Struct.Mmm(UMODESEGLOB,UMODESEGLOB)];
%   add aero mass terms
    HAM = aero_interp_vec(Ham, 2, k(i), dyn_model.dlm.aero.k);
    HAM(:,UMODESRLOC) = imag(HAM(:,UMODESRLOC))/Omega(i) - 1i * real(HAM(:,UMODESRLOC)) / Omega(i);
%   force terms
    if (nSURF || nGUST)
      L_tot_Mode(:,i) = (qinfty*aero_interp_vec(Hal, 2, k(i), dyn_model.dlm.aero.k )*Load(:,i));
      L_tot(:,i) = L_tot_Mode(:,i);
    end
%   add external contribution
    if (nLOAD)
      L_tot_Extf(:,i) = set_modal_extload(beam_model.Struct.NDispl(:,:,UMODESIND), beam_model.Param.LOAD, beam_model.Dextload, LoadE(:,i));
      L_tot(:,i) = L_tot(:,i) + L_tot_Extf(:,i);
    end
%   system dynamics
    Aow = MASS + Bmm(UMODESIND,UMODESIND)* 1i * Omega(i) + Kmm(UMODESIND,UMODESIND) - qinfty * HAM;
%   response
    Q(:,i) = Aow \ L_tot(:,i);
%   recover modal velocities
    Qdot(UMODESELOC,i)  = (1i*Omega(i)) .* Q(UMODESELOC,i);
    Qdot(UMODESRLOC,i)  = Q(UMODESRLOC,i);
%   determine rigid mode amplitudes
    Q(UMODESRLOC,i)     = Qdot(UMODESRLOC,i)/ (1i*Omega(i));
%   recover modal accelerations
    Qddot(:,i) = (1i*Omega(i)) .* Qdot(:,i);
%   Rigid aero coefficients
    Cyome_mode(:,i) = aero_interp_vec(Cy_mode, 2, k(i), dyn_model.dlm.aero.k)*Q(:,i);
    Czome_mode(:,i) = aero_interp_vec(Cz_mode, 2, k(i), dyn_model.dlm.aero.k)*Q(:,i);
    Clome_mode(:,i) = aero_interp_vec(Cl_mode, 2, k(i), dyn_model.dlm.aero.k)*Q(:,i);
    Cmome_mode(:,i) = aero_interp_vec(Cm_mode, 2, k(i), dyn_model.dlm.aero.k)*Q(:,i);
    Cnome_mode(:,i) = aero_interp_vec(Cn_mode, 2, k(i), dyn_model.dlm.aero.k)*Q(:,i);
  end
% Recover modal dynamics            
  L_tot_Mode = [L_tot_Mode, conj(L_tot_Mode(:,end:-1:2))];
  L_tot_Extf = [L_tot_Extf, conj(L_tot_Extf(:,end:-1:2))];
  Q     = [Q,    conj(Q(:,end:-1:2))];
  Qdot  = [Qdot, conj(Qdot(:,end:-1:2))];
  Qddot = [Qddot,conj(Qddot(:,end:-1:2))];
% Aero coeff
  Cyome_mode = [Cyome_mode, conj(Cyome_mode(:,end:-1:2))];
  Czome_mode = [Czome_mode, conj(Czome_mode(:,end:-1:2))];
  Clome_mode = [Clome_mode, conj(Clome_mode(:,end:-1:2))];
  Cmome_mode = [Cmome_mode, conj(Cmome_mode(:,end:-1:2))];
  Cnome_mode = [Cnome_mode, conj(Cnome_mode(:,end:-1:2))];
%
  ltot_mode = zeros(NMODES,nt*2-1);
  ltot_extf = zeros(NMODES,nt*2-1);
  q         = zeros(NMODES,nt*2-1);
  qdot      = zeros(NMODES,nt*2-1);
  qddot     = zeros(NMODES,nt*2-1);
%
  for i = 1 : nUMODES
    ltot_mode(UMODESIND(i),:) = ifft(L_tot_Mode(i,:));
    ltot_extf(UMODESIND(i),:) = ifft(L_tot_Extf(i,:));
    q(UMODESIND(i),:)         = ifft(Q(i,:));
    qdot(UMODESIND(i),:)      = ifft(Qdot(i,:));
    qddot(UMODESIND(i),:)     = ifft(Qddot(i,:));
  end
% save dynamics
  dyn_model.Res.Time = T';
  dyn_model.Res.Qload = ltot_mode;
  dyn_model.Res.Qextf = ltot_extf;
% clean rigid modes
  q(UMODESRGLOB,:) = bsxfun(@minus,q(UMODESRGLOB,:),q(UMODESRGLOB,end));
  dyn_model.Res.Q  = q(:,1:nt);
  dyn_model.Res.Qd = qdot(:,1:nt);
  dyn_model.Res.Qddot = qddot(:,1:nt);
  % save rigid aero response
  dyn_model.Res.Cy_mode = ifft(Cyome_mode); dyn_model.Res.Cy_mode = dyn_model.Res.Cy_mode(1:nt);
  dyn_model.Res.Cz_mode = ifft(Czome_mode); dyn_model.Res.Cz_mode = dyn_model.Res.Cz_mode(1:nt);
  dyn_model.Res.Cl_mode = ifft(Clome_mode); dyn_model.Res.Cl_mode = dyn_model.Res.Cl_mode(1:nt);
  dyn_model.Res.Cm_mode = ifft(Cmome_mode); dyn_model.Res.Cm_mode = dyn_model.Res.Cm_mode(1:nt);
  dyn_model.Res.Cn_mode = ifft(Cnome_mode); dyn_model.Res.Cn_mode = dyn_model.Res.Cn_mode(1:nt);
%
%-------------------------------------------------------------------------------
%
% STRUCTURAL OUTPUTS
%
%-------------------------------------------------------------------------------
% DISPLACEMENT
  if ~isempty(beam_model.Param.DISP)
    dyn_model.Res.DISP = reshape(dyn_model.Out.DISP*dyn_model.Res.Q,...
                         length(beam_model.Param.DISP),6,nt);
  end
%-------------------------------------------------------------------------------
% VELOCITY
  if ~isempty(beam_model.Param.VELOCITY)
    dyn_model.Res.VELOCITY = reshape(dyn_model.Out.VELOCITY*dyn_model.Res.Qd,...
                             length(beam_model.Param.VELOCITY),6,nt);
  end
%-------------------------------------------------------------------------------
% ACCELERATION
  if ~isempty(beam_model.Param.ACCELERATION)
    dyn_model.Res.ACCELERATION = reshape(dyn_model.Out.ACCELERATION*dyn_model.Res.Qddot,...
                                 length(beam_model.Param.ACCELERATION),6,nt);
  end
%-------------------------------------------------------------------------------
% BAR INTERNAL LOAD
  if ~isempty(beam_model.Param.IFORCE)
    dyn_model.Res.IFORCE = reshape((dyn_model.Out.IFORCE*dyn_model.Res.Q),...
                           6,2,length(beam_model.Param.IFORCE),nt);
  end
%-------------------------------------------------------------------------------
% BEAM INTERNAL LOAD
  if ~isempty(beam_model.Param.IFORCEBE)
    dyn_model.Res.IFORCEBE = reshape((dyn_model.Out.IFORCEBE*dyn_model.Res.Q),...
                             6,2,length(beam_model.Param.IFORCEBE),nt);
  end
%-------------------------------------------------------------------------------
%
% AERO OUTPUTS
%
%-------------------------------------------------------------------------------
% CP response
  if ~isempty(beam_model.Param.AEROFORCE)
%   controls
    if  ~isempty(beam_model.Param.SURFDEF)
      CPome_surf = zeros(length(beam_model.Param.AEROFORCE),nt);
      for i = 2 : nt
        CPome_surf(:,i) = aero_interp_vec(CPsurf, 2, k(i), dyn_model.dlm.aero.k) * Load(1:nSURF,i);
      end
      dyn_model.Res.CP_surf = zeros(length(beam_model.Param.AEROFORCE),nt*2-1);
      CPome_surf = [CPome_surf,conj(CPome_surf(:,end:-1:2))];
      for i = 1 : length(beam_model.Param.AEROFORCE)
        dyn_model.Res.CP_surf(i,:) = ifft(CPome_surf(i,:));
      end
      dyn_model.Res.CP_surf = dyn_model.Res.CP_surf(:,1:nt);
    end
%   gusts
    if ~isempty(beam_model.Param.GUST)
      CPome_gust = zeros(length(beam_model.Param.AEROFORCE),nt);
      for i = 2 : nt
        CPome_gust(:,i) = sum(aero_interp_vec(CPgust, 2, k(i), dyn_model.dlm.aero.k)*...
                              reshape(Load(nSURF+1:nSURF+nGUST*np,i),np,nGUST),2);
      end
      CPome_gust = [CPome_gust,conj(CPome_gust(:,end:-1:2))];
      dyn_model.Res.CP_gust = zeros(length(beam_model.Param.AEROFORCE),nt*2-1);
      for i = 1 : length(beam_model.Param.AEROFORCE)
        dyn_model.Res.CP_gust(i,:) = ifft(CPome_gust(i,:));
      end
      dyn_model.Res.CP_gust = dyn_model.Res.CP_gust(:,1:nt);
    end
%   modal
    CPome_mode = zeros(length(beam_model.Param.AEROFORCE),nt);
    for i = 2 : nt
      CPome_mode(:,i) = aero_interp_vec(CPmode, 2, k(i), dyn_model.dlm.aero.k) * Q(:,i);
    end
    CPome_mode = [CPome_mode,conj(CPome_mode(:,end:-1:2))];
    dyn_model.Res.CP_mode = zeros(length(beam_model.Param.AEROFORCE),nt*2-1);
    for i = 1 : length(beam_model.Param.AEROFORCE)
      dyn_model.Res.CP_mode(i,:) = ifft(CPome_mode(i,:));
    end
    dyn_model.Res.CP_mode = dyn_model.Res.CP_mode(:,1:nt);
  end
%-------------------------------------------------------------------------------
% HF responce
  if ~isempty(beam_model.Param.HINGEFORCE)
%   control contribution
    if  ~isempty(beam_model.Param.SURFDEF)
      HFome_surf = zeros(length(beam_model.Param.HINGEFORCE),nt);
      for i = 2 : nt
        HFome_surf(:,i) = -qinfty*aero_interp_vec(HFsurf, 2, k(i), dyn_model.dlm.aero.k) * Load(1:nSURF,i);
      end
      HFome_surf = [HFome_surf,conj(HFome_surf(:,end:-1:2))];
      dyn_model.Res.HF_surf = zeros(length(beam_model.Param.HINGEFORCE),nt*2-1);
      for i = 1 : length(beam_model.Param.HINGEFORCE)
        dyn_model.Res.HF_surf(i,:) = ifft(HFome_surf(i,:));
      end
      dyn_model.Res.HF_surf = dyn_model.Res.HF_surf(:,1:nt);
    end
%   gust contribution
    if ~isempty(beam_model.Param.GUST)
      HFome_gust = zeros(length(beam_model.Param.HINGEFORCE),nt);
      for i = 2 : nt
        HFome_gust(:,i) = sum(-qinfty*aero_interp_vec(HFgust, 2, k(i), dyn_model.dlm.aero.k)* ...
                               reshape(Load(nSURF+1:nSURF+nGUST*np,i),np,nGUST),2);
      end
      HFome_gust = [HFome_gust,conj(HFome_gust(:,end:-1:2))];
      dyn_model.Res.HF_gust = zeros(length(beam_model.Param.HINGEFORCE),nt*2-1);
      for i = 1 : length(beam_model.Param.HINGEFORCE)
        dyn_model.Res.HF_gust(i,:) = ifft(HFome_gust(i,:));
      end
      dyn_model.Res.HF_gust = dyn_model.Res.HF_gust(:,1:nt);
    end
%   modal contribution
    HFome_mode = zeros(length(beam_model.Param.HINGEFORCE),nt);
    for i = 2 : nt
      HFome_mode(:,i) = -qinfty*aero_interp_vec(HFmode, 2, k(i), dyn_model.dlm.aero.k)*Q(:,i);
    end
    HFome_mode = [HFome_mode,conj(HFome_mode(:,end:-1:2))];
    dyn_model.Res.HF_mode = zeros(length(beam_model.Param.HINGEFORCE),nt*2-1);
    for i = 1 : length(beam_model.Param.HINGEFORCE)
      dyn_model.Res.HF_mode(i,:) = ifft(HFome_mode(i,:));
    end
      dyn_model.Res.HF_mode = dyn_model.Res.HF_mode(:,1:nt);
  end
%-------------------------------------------------------------------------------
% Controls
  if  ~isempty(beam_model.Param.SURFDEF)
    Cyome_surf = zeros(1,nt);
    Czome_surf = zeros(1,nt);
    Cmome_surf = zeros(1,nt);
    Clome_surf = zeros(1,nt);
    Cnome_surf = zeros(1,nt);
    for i = 2 : nt
      Cyome_surf(:,i) = sum(aero_interp_vec(Cy_surf, 2, k(i), dyn_model.dlm.aero.k) * Load(1:nSURF,i));
      Czome_surf(:,i) = sum(aero_interp_vec(Cz_surf, 2, k(i), dyn_model.dlm.aero.k) * Load(1:nSURF,i));
      Clome_surf(:,i) = sum(aero_interp_vec(Cl_surf, 2, k(i), dyn_model.dlm.aero.k) * Load(1:nSURF,i));
      Cmome_surf(:,i) = sum(aero_interp_vec(Cm_surf, 2, k(i), dyn_model.dlm.aero.k) * Load(1:nSURF,i));
      Cnome_surf(:,i) = sum(aero_interp_vec(Cn_surf, 2, k(i), dyn_model.dlm.aero.k) * Load(1:nSURF,i));
    end
    Cyome_surf = [Cyome_surf,conj(Cyome_surf(:,end:-1:2))];
    Czome_surf = [Czome_surf,conj(Czome_surf(:,end:-1:2))];
    Clome_surf = [Clome_surf,conj(Clome_surf(:,end:-1:2))];
    Cmome_surf = [Cmome_surf,conj(Cmome_surf(:,end:-1:2))];
    Cnome_surf = [Cnome_surf,conj(Cnome_surf(:,end:-1:2))];
    dyn_model.Res.Cy_surf = ifft(Cyome_surf); dyn_model.Res.Cy_surf = dyn_model.Res.Cy_surf(:,1:nt);
    dyn_model.Res.Cz_surf = ifft(Czome_surf); dyn_model.Res.Cz_surf = dyn_model.Res.Cz_surf(:,1:nt);
    dyn_model.Res.Cl_surf = ifft(Clome_surf); dyn_model.Res.Cl_surf = dyn_model.Res.Cl_surf(:,1:nt);
    dyn_model.Res.Cm_surf = ifft(Cmome_surf); dyn_model.Res.Cm_surf = dyn_model.Res.Cm_surf(:,1:nt);
    dyn_model.Res.Cn_surf = ifft(Cnome_surf); dyn_model.Res.Cn_surf = dyn_model.Res.Cn_surf(:,1:nt);
  end
%-------------------------------------------------------------------------------
% GUST
  if  ~isempty(beam_model.Param.GUST)
    Cyome_gust = zeros(1,nt);
    Czome_gust = zeros(1,nt);
    Cmome_gust = zeros(1,nt);
    Clome_gust = zeros(1,nt);
    Cnome_gust = zeros(1,nt);
    for i = 2 : nt
      Cyome_gust(:,i) = sum(aero_interp_vec(Cy_gust, 2, k(i), dyn_model.dlm.aero.k)*reshape(Load(nSURF+1:nSURF+nGUST*np,i), np, nGUST));
      Czome_gust(:,i) = sum(aero_interp_vec(Cz_gust, 2, k(i), dyn_model.dlm.aero.k)*reshape(Load(nSURF+1:nSURF+nGUST*np,i), np, nGUST));
      Clome_gust(:,i) = sum(aero_interp_vec(Cl_gust, 2, k(i), dyn_model.dlm.aero.k)*reshape(Load(nSURF+1:nSURF+nGUST*np,i), np, nGUST));
      Cmome_gust(:,i) = sum(aero_interp_vec(Cm_gust, 2, k(i), dyn_model.dlm.aero.k)*reshape(Load(nSURF+1:nSURF+nGUST*np,i), np, nGUST));
      Cnome_gust(:,i) = sum(aero_interp_vec(Cn_gust, 2, k(i), dyn_model.dlm.aero.k)*reshape(Load(nSURF+1:nSURF+nGUST*np,i), np, nGUST));
    end
    Cyome_gust = [Cyome_gust,conj(Cyome_gust(:,end:-1:2))];
    Czome_gust = [Czome_gust,conj(Czome_gust(:,end:-1:2))];
    Clome_gust = [Clome_gust,conj(Clome_gust(:,end:-1:2))];
    Cmome_gust = [Cmome_gust,conj(Cmome_gust(:,end:-1:2))];
    Cnome_gust = [Cnome_gust,conj(Cnome_gust(:,end:-1:2))];
    dyn_model.Res.Cy_gust = ifft(Cyome_gust); dyn_model.Res.Cy_gust =dyn_model.Res.Cy_gust(:,1:nt);
    dyn_model.Res.Cz_gust = ifft(Czome_gust); dyn_model.Res.Cz_gust =dyn_model.Res.Cz_gust(:,1:nt);
    dyn_model.Res.Cl_gust = ifft(Clome_gust); dyn_model.Res.Cl_gust =dyn_model.Res.Cl_gust(:,1:nt);
    dyn_model.Res.Cm_gust = ifft(Cmome_gust); dyn_model.Res.Cm_gust =dyn_model.Res.Cm_gust(:,1:nt);
    dyn_model.Res.Cn_gust = ifft(Cnome_gust); dyn_model.Res.Cn_gust =dyn_model.Res.Cn_gust(:,1:nt);
  end
  fprintf(fid,'\n');
%-------------------------------------------------------------------------------
% ACCELERATION MODES
%
  if (MODACC == 0)
    if (~isempty(beam_model.Param.IFORCE) || ~isempty(beam_model.Param.IFORCEBE))
%
      NODEST = beam_model.Node;
      if ~isempty(beam_model.RBE2.ID)
        NODEST.DOF = NODEST.DOF2;
      end
      M = beam_model.Struct.M; 
      K = beam_model.Struct.K;
      Mu = M * beam_model.Struct.V;
%     Apply SUPORT
      [D, Kll, Klr, Krr, Krl, rdof, ldof, KEPS] = get_suport_shapes(K, NODEST, beam_model.Param.SUPORT, beam_model.Param.EPS);
      invKll = inv(Kll);
%
      U = zeros(ndof2, nt);
%
      Fexta = zeros(ndof, 1);
      Fextl = zeros(ndof, 1);
%
      for i = 2 : nt
%
        Fext  = zeros(ndof, 1);
%       External inputs along FE mesh (gust + controls) : HAG * LOAD
        if (nSURF || nGUST)
          Fexta  =  qinfty * aero_interp_vec(Hdl,2, k(i), dyn_model.dlm.aero.k) * Load(:,i);
          Fext = Fexta;
        end
%       add external contribution
        if (nLOAD)
          Fextl = set_nodal_extload(beam_model.Node.DOF, beam_model.Param.LOAD, beam_model.Dextload, LoadE(:,i));
          Fext = Fext + Fextl;
        end
%       Aero forces along FE mesh : HAM * U * q
        Faeroel = -qinfty * aero_interp_vec(Hdm,2, k(i), dyn_model.dlm.aero.k)* Q(:,i);
%       reduce RBE2 dofs from ndof to ndfo2
        if ~isempty(beam_model.RBE2.ID)
          Faeroel = RBE2Assembly2(beam_model.RBE2, Faeroel);
          Fext    = RBE2Assembly2(beam_model.RBE2, Fext);
        end
%       Forces due to qddot   
        Fqddot = -Omega(i)^2 * Mu(:,UMODESIND) *  Q(:,i);
%       rhs
        Ftot = Fext - Fqddot - Faeroel;
%       solve system
        U(ldof,i) = invKll * Ftot(ldof);
%
      end
%
      U = [U, conj(U(:,end:-1:2))];
      u = zeros(ndof2, nt*2-1);
      SOL = zeros(ndof, nt);
%     transform ldof nodal displacements
      for i = 1 : length(ldof)
        u(ldof(i),:) = ifft(U(ldof(i),:));
      end
%     Get complete dof displacements field
      if ~isempty(beam_model.RBE2.ID)
        for i=1:nt
          SOL(:,i) = RBE2disp(beam_model.RBE2, u(:,i), ndof);
        end  
      else
        SOL = u;
      end
%     Assembly nodal displacements
      ngrid = beam_model.Info.ngrid;
      nbar =  beam_model.Info.nbar;
      nbeam =  beam_model.Info.nbeam;
  	  NDispl = zeros(ngrid, 6, nt);
%     store nodal displacement
      for n = 1:ngrid 
	      dof = beam_model.Node.DOF(n, 1:6);
	      index = find(dof);
	      if ~isempty(index)
          for i=1:nt
		        NDispl(n, index, i) = SOL(dof(index), i);
          end
	      end
      end
%
%      assembly BAR contributions directly in the undeformed position
      if ~isempty(beam_model.Param.IFORCE)
        CForces = zeros(2, 6, nbar); CStrains = zeros(2, 6, nbar); CSM = [];
        nb = length(beam_model.Param.IFORCE);
        dyn_model.Res.MODACC.IFORCE = zeros(6,2,nb);
        for i=1:nt
          [CForces, CStrains, CStresses, CSM] = ...
          get_bar_force_strain(beam_model.Info.nbar, beam_model.Bar, beam_model.PBar, beam_model.Mat, beam_model.Node, ...
          NDispl(:,:,i), beam_model.Param.FUSE_DP);
%         export only required bar
          for k=1:nb
            dyn_model.Res.MODACC.IFORCE(:,:,k,i) = CForces(:,:,beam_model.Param.IFORCE(k))';
          end
        end
      end
%     assembly BEAM contributions directly in the undeformed position
      if ~isempty(beam_model.Param.IFORCEBE)
        CForces = zeros(2, 6, nbeam);  CStrains = zeros(2, 6, nbeam); CSM = [];
        nb = length(beam_model.Param.IFORCEBE);
        dyn_model.Res.MODACC.IFORCEBE = zeros(6,2,nb,nt);
        for i=1:nt
          [CForces, CStrains, CStresses, CSM] = ...
          get_bar_force_strain(beam_model.Info.nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Mat, beam_model.Node, ...
          NDispl(:,:,i), beam_model.Param.FUSE_DP);
          for k=1:nb
            dyn_model.Res.MODACC.IFORCEBE(:,:,k,i) = CForces(:,:,beam_model.Param.IFORCEBE(k))';
          end
        end
      end
%     save displacements
      if ~isempty(beam_model.Param.DISP)
        dyn_model.Res.MODACC.DISP = NDispl(beam_model.Param.DISP,:,:);
      else
        dyn_model.Res.MODACC.DISP = NDispl;
      end
    else
      fprintf(fid, '\n\t### Warning: acceleration modes required. No Bar/beam elements given in IFORCE/IFORCEBE set.');
      fprintf(fid, '\n\t    Process skipped.');
    end
  end % MODE ACC
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
function [Aa,B0,B1,B2,Ca,E0,E1,E2,nXa] = load_fitting(FILE, cref, VREF)
      data = load(FILE);
      solution = data.solution;
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
end
%
function [CD0, CDA, CDu, CLu, Cmu] = load_drag_file(FILE)
fid = 1;
CD0 = 0; 
%CL0 = -1; 
CDA = 0; 
CDu = 0; 
CLu = 0;
Cmu = 0; 
%
if ~exist(FILE, 'file')
  error(['Unable to find ', FILE, ' file.']);
end
a = importdata(FILE);
nd = size(a.data,1);
%
for i=1:nd
  switch(char(a.textdata(i,1)))
    case {'CD0', 'cd0', 'Cd0'}
      CD0 = a.data(i);
%    case {'CL0', 'cl0', 'Cl0'}
%     CL0 = a.data(i);
    case {'CDA', 'cda', 'CdA'}
      CDA = a.data(i);
      if (CDA<0)
        fprintf(fid, '   #Warning: negative value for CDA provided. Absolute value used.');
        CDA = -CDA;
      end        
    case {'Cmu', 'cmu'}
      Cmu = a.data(i);
    case {'CDu', 'cdu'}
      CDu = a.data(i);
      if (CDu<0)
        fprintf(fid, '   #Warning: negative value for CDu provided. Absolute value used.');
        CDu = -CDu;
      end        
    case {'CLu', 'clu'}
      CLu = a.data(i);
      if (CLu<0)
        fprintf(fid, '   #Warning: negative value for CLu provided. Absolute value used.');
        CLu = -CLu;
      end        
  end
end

end
