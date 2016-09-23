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
function Y=solve_free_lin_dyn(varargin)

global dyn_model
global fl_model
%
beam_model = dyn_model.beam;
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
  AEROFILE = [];
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
%  MODACC = beam_model.Param.MODACC;
  SDAMP = beam_model.Param.SDAMP;
%
  fprintf(fid,'\n - Reference chord:    %g m.', cref); 
  fprintf(fid,'\n - Reference span:     %g m.', bref); 
  fprintf(fid,'\n - Reference surface:  %g m2.', sref); 
%  if (MODACC == 0)
%    fprintf(fid,'\n - Acceleration modes required.'); 
%  end
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
      if (strcmp(PARAM(n), 'aero_file')) 
        AEROFILE = PARAM{n+1};
        fprintf(fid,'\n - Filename with aerodynamic fitting: %s.', AEROFILE);
        if ~exist(AEROFILE,'file')
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
Had = [];
Hag = [];
Ham = dyn_model.dlm.data.Qhh(UMODESIND,UMODESIND,:,MINDEX);
%
if  ~isempty(beam_model.Param.SURFDEF)
  Had = dyn_model.dlm.data.Qhd(UMODESIND,beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF),:, MINDEX);
end
%Had = dyn_model.dlm.data.Qhd(UMODESIND,:,:, MINDEX);
if ~isempty(beam_model.Param.GUST)
  Hag = zeros(nUMODES, nGUST, nk);
  dyn_model.Res.Gust_profile = zeros(nGUST, nt); 
  for i = 1:nGUST
    % assembly gust delay
%    Qhg = exp(-( midPoint(:,1) - beam_model.Gust.X0(beam_model.Param.GUST(i)) )*(1i*dyn_model.dlm.aero.k/cref) )...
%      .* repmat(dyn_model.gust.dwnwash(:,beam_model.Param.GUST(i)),1,nk); 
    Qhg = exp(-( midPoint(:,1) - X0min) *(1i*dyn_model.dlm.aero.k/cref) )...
      .* repmat(dyn_model.gust.dwnwash(:,beam_model.Param.GUST(i)),1,nk); 

    % assembly k column
    for j=1:nk
      Hag(:,i,j) = dyn_model.gust.Qhg(UMODESIND,:,j,MINDEX) * Qhg(:,j);
    end
  end
end
%
k = dyn_model.dlm.aero.k;
Ha = [Ham, Had, Hag];
%
if isempty(AEROFILE)
  filename = [headname,'_qhh_M_',num2str(MREF),'.mat'];
  save(filename,'k','Ha');
  %
  Ha = Had;
  filename = [headname,'_qhh_M_',num2str(MREF),'_surf.mat'];
  save(filename,'k','Ha');

  Ha = Hag;
  filename = [headname,'_qhh_M_',num2str(MREF),'_gust.mat'];
  save(filename,'k','Ha');




  fprintf(fid, '\n\n - Aerodynamic matrix for fitting exported to %s file.', filename);
  fprintf(fid,'\n   Rows: %d.', nUMODES);
  fprintf(fid,'\n   Columns for modes: 1 -> %d.', nUMODES);
  if (nSURF)
    fprintf(fid,'\n   Columns for controls: %d -> %d.', nUMODES+1, nUMODES+nSURF);
  end
  if (nGUST)
    fprintf(fid,'\n   Columns for gusts: %d -> %d.', nUMODES+nSURF+1, nUMODES+nSURF+nGUST);
  end

  fprintf(fid, '\n - Use the panel GUI to fit aerodynamic data.');
  fprintf(fid, '\n - When fitting is concluded, run the solver with ''aero_file'' parameter followed by the output file created.\n');
%
  mygui;
%[Aid, Bid, Cid, Eid] = aero_ss(k, Ha);
  return
else
  data = load(AEROFILE);
  solution = data.solution;
  Aid    = solution.inoutresid.AA;
  Bid{1} = solution.inoutresid.BB{1};
  Bid{2} = solution.inoutresid.BB{2};
  Bid{3} = solution.inoutresid.BB{3};
  Cid    = solution.inoutresid.CC;
  Eid{1} = solution.inoutresid.DD{1};
  Eid{2} = solution.inoutresid.DD{2};
  Eid{3} = solution.inoutresid.DD{3};
end
%
input_delay = zeros(nSURF*3+nGUST*3+nLOAD,1);
for i=1:nSURF
  for j=1:3
    input_delay((j-1)*(nSURF+nGUST)+i) = delay_delta(i);
  end
end 
for i=1:nGUST
  for j=1:3
    input_delay((j-1)*(nSURF+nGUST)+i+nSURF) = delay_vg(i);
  end
end 
for i=1:nLOAD
  input_delay(nSURF*3 + nGUST*3 + i) = delay_f(i);
end 
%
nXa = size(Aid,1); % number of aerodynamic states
% aero matrices
E0 = Eid{1};
E1 = Eid{2}*(cref/VREF)^+1;
E2 = Eid{3}*(cref/VREF)^+2;
B0 = Bid{1}.*(cref/VREF)^-1;
B1 = Bid{2}.*(cref/VREF)^0;
B2 = Bid{3}.*(cref/VREF)^+1;
Aa = Aid.*(cref/VREF)^-1;
Ca = Cid;
% structural matrices
MM = beam_model.Struct.Mmm(UMODESIND,UMODESIND);
KK = beam_model.Struct.Kmm(UMODESIND,UMODESIND);
CC = Bmm(UMODESIND,UMODESIND);
% aeroelastic matrices
Mbar = MM - qinfty*E2(:,1:nUMODES);
Cbar = CC - qinfty*E1(:,1:nUMODES);
Kbar = KK - qinfty*E0(:,1:nUMODES);
%
I = eye(nUMODES);
O = zeros(nUMODES);
Ia = eye(nXa);
Oa = zeros(nUMODES,nXa);
Oi = zeros(nUMODES,nSURF+nGUST);
% ss system in descriptor form
Eae = [ I,                  O, Oa; 
        O,               Mbar, Oa; 
        Oa', -B2(:,1:nUMODES), Ia];
Aae = [ O,              I,              Oa; 
       -Kbar,           -Cbar,          qinfty*Ca; 
       B0(:,1:nUMODES), B1(:,1:nUMODES), Aa];
%
Bae = [];
if (~isempty(beam_model.Param.GUST) || ~isempty(beam_model.Param.SURFDEF))
  Bae = [Oi,                          Oi,                          Oi;
         qinfty*E0(:,nUMODES+1:end), qinfty*E1(:,nUMODES+1:end), qinfty*E2(:,nUMODES+1:end);
         B0(:,nUMODES+1:end),        B1(:,nUMODES+1:end),        B2(:,nUMODES+1:end)];
end
% divide gust terms
%if ~isempty(beam_model.Param.GUST)
%  Bae(nUMODES+1:2*nUMODES,nUMODES+nSURF+1:end) = Bae(nUMODES+1:2*nUMODES,nUMODES+nSURF+1:end)./VREF;
%end
%
if  ~isequal(beam_model.Param.LOAD, 0)
  [Baeh, f_label] = set_modal_extload(beam_model.Struct.NDispl(:,:,UMODESIND), beam_model.Node.ID, beam_model.Param.LOAD, beam_model.Dextload, LoadE(:,i));
  Baeh2 = [zeros(nUMODES,nLOAD); Baeh; zeros(nXa,nLOAD)];
  Bae = [Bae, Baeh2]; 
end
%
% Export state, q, qdot, qddot
%
Cae = zeros(3*nUMODES, 2*nUMODES+nXa);
Cae = [eye(2*nUMODES,2*nUMODES), zeros(2*nUMODES,nXa); Aae(nUMODES+1:2*nUMODES,:)];
Dae = [zeros(2*nUMODES,nSURF*3+nGUST*3+nLOAD); Bae(nUMODES+1:2*nUMODES,:)];
%
%---------------------------------------------------------------------------
% create labels
%
state_name = {};
output_name = {};
for i=1:nUMODES
  state_name{i} = ['q', num2str(beam_model.Param.UMODES(i))];
  state_name{i+nUMODES} = ['q', num2str(beam_model.Param.UMODES(i)),'-dot'];
  output_name{i} = state_name{i};
  output_name{i+nUMODES} = state_name{i+nUMODES};
  output_name{i+2*nUMODES} = ['q', num2str(beam_model.Param.UMODES(i)),'-ddot'];
end
%
for i=1:nXa
  state_name{2*nUMODES + i} = ['xa', num2str(i)];
end
%
input_name = {};
if ~isempty(beam_model.Param.SURFDEF)
  for i = 1: nSURF
    input_name{i} = cell2mat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))));
    input_name{i+nSURF+nGUST} = cell2mat(strcat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))),'-dot'));
    input_name{i+2*nSURF+2*nGUST} = cell2mat(strcat(beam_model.Aero.Trim.MasterSurf(beam_model.Surfdef.LabelID(beam_model.Param.SURFDEF(i))),'-ddot'));
  end
end
if ~isempty(beam_model.Param.GUST)
  for i = 1:nGUST
    input_name{i+3*nSURF} = ['alphag',num2str(i)];
    input_name{i+3*nSURF+nGUST} = ['alphag',num2str(i),'-dot'];
    input_name{i+3*nSURF+2*nGUST} = ['alphag',num2str(i),'-ddot'];
  end
end
%
if  ~isequal(beam_model.Param.LOAD, 0)
  nip = size(Baeh,2);
  for i=1:nip
    input_name{i+3*nSURF+3*nGUST} = f_label{i};
  end
end
%
ss_model = dss(Aae, Bae, Cae, Dae, Eae, 'StateName', state_name, 'OutputName', output_name, 'InputName', input_name, 'InputDelay', input_delay);
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
      U(i+(j-1)*(nSURF+nGUST),:) = input_value(:,j)';
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
      U(i+(j-1)*(nSURF+nGUST)+nSURF,:) = input_value(:,j);
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
    U(i+(nSURF+nGUST)*3,1:ind) = symbolic_function(fun, T(1:inde)).*beam_model.Dextload.Amp(beam_model.Param.LOAD(i));
    dyn_model.Res.Extload_profile(i,:) = [zeros(ndelay,1); input_value(1:inde,1); zeros(nt-nind-ndelay,1)]';
  end
end

Y = lsim(ss_model, U', T);

%save('pippo.mat')
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
function Q = set_modal_extload(NDispl, NID, PLOAD, LOAD, LoadE)
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
