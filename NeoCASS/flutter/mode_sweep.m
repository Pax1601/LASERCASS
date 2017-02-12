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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Routine for sweep on each mode chosen by the parameter IMOD.
% Input: nmodc, nmods, Khh, Chh, Mhh, rho, VMAX
% Output: IWAR, NSTOP
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      ------------ REAL -----------
%
%      Khh = STIFFNESS MATRIX
%      Chh = STRUCTURAL DAMPING MATRIX (imag...)
%      Mhh = MASS MATRIX
%      SVQU   = SOLUTION VECTOR + PARAMETERS
%      OMEP   = NATURAL FREQUENCIES VECTOR (PULSATION)
%      A,INI,LV,S4V = TEMPORARY VECTORS
%
%      ------- COMPLEX --------
%
%
%      TN     = Vector containing Unknowns calculated at that iteration,
%               first nmodc elemnts contain modal data, while element nmodc+1
%               contains damping and pulsation;
%      FORDER = Matrix containing generalized aerodynamic forces
%               derivatives calcualted at a given reduced frequency;
%      FORINT = Matrix containing generalized aerodynamic forces evaluated
%               by interpolation at a given reduced frequency;
%      DERINT = Derivative terms of generalized aerodynamic forces
%               evaluated by interpolation at a given reduced frequency;
%      COEF   = Coefficient Matrix used to solve flutter problem;
%      SVTN   = Vector containing solution evaluated at N-1 step;
%      SVTNFL = Vector containing interpolated solution at Flutter Velocity
%      SVDR   = SOLUZIONI DEL SISTEMA CONTENENTE I DELTA#Q, E I DELTA
%               SIGMA E DELTAOMEGA.

function [FL_DET, risultati, SVTNFL, SVQU, IMODFL, IWAR, NSTOP] = ...
    mode_sweep(nm, Mhh, Chh, Khh, AER, Kfreq, rho, chord, ISWP, IVMODS, OMEP, SVQU, IMODFL);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%      PREPARA IL MODO DI PARTENZA DA SEGUIRE : E' UN VETTORE COMPLESSO
%      DI LUNGHEZZA nmodc1 AVENTE NEI PRIMI nmodc MODI L'AUTOVETTORE
%      SEGUITO (TUTTI GLI ELEMENTI NULLI TRANNE UN 1 NELLA PARTE REALE
%      DELL'ELEMENTO CORRISPONDENTE) E NELLA PARTE IMMAGINARIA DELL'ELE-
%      MENTO nmodc1 LA PULSAZIONE PROPRIA CORRISPONDENTE.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      METHOD : 1 = NEWTON - RAPHSON
%               2 = MODIFIED NEWTON - RAPHSON (constant derivative)
%               3 = VELOCITY DERIVATIVE
FL_DET = false;
risultati = [];

global fl_model;
global beam_model;

%
NKfreq = length(Kfreq);
%
AERSCA = fl_model.param.AERSCA;
MAXITE = fl_model.param.MAXITE;
MAXITS = fl_model.param.MAXITS;
DVMIN = fl_model.param.DVMIN;
DVMAX = fl_model.param.DVMAX;
DVEL = fl_model.param.DVEL;
VMAX = fl_model.param.VMAX;
ERR = fl_model.param.ERR;
DOUBLING = fl_model.param.DOUBLING;
%SVQU  = fl_model.param.SVQU;
%IMODFL = fl_model.param.IMODFL(RUN_INDEX);
nmodc = size(Mhh,1);
SVTNFL = zeros(1, nmodc);
%fl_model.param.IMODEG;
%
nmodc1 = nmodc + 1;
nmodc2 = nmodc + 2;
nmodc3 = nmodc + 3;
IMOD = ISWP;
%
INI = [1,3,1];
LV = [0,4,0];
TOLL = 1.0e-6;
A=zeros(6,1);
EPS = 1.e-15;
FKTS = 0.51444;
PP2 = 2*(pi);
NSTOP = (' ');
IDUETR = 0;
CSAVE = 0+i*0;
IWAR = 0;
VALFRE   = 0;
DISCR    = 0;
VFLINT   = 0;
SVQU(3)  = 0;
%
FORINT = zeros(nmodc,nmodc);
DERINT = zeros(nmodc,nmodc);
COEF = zeros(nmodc1,nmodc3);
TN = zeros(nmodc1,1);
SVTN = zeros(nmodc1,1);
SVDR = zeros(nmodc1);
%
velcyc = 1;
%
IVPOS = find(IMOD == IVMODS);
%
% SVTN'contains always the solution evaluated at the last step
SVTN(IVPOS) = 1+0*i; % set guess eigenvector equal to structural mode in vacuum
SVTN(nmodc1) = i*(OMEP); % set guess eigenvalue ""
%
ISTEP = 0;
ISTOP = 0;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%      INITIAL PARAMETERS SETTING FOR SWEEP: AT FIRST STEP, DVEL VELOCITY
%      IS THE LOWEST SWEEP VELOCITY, WHILE DURING FOLLOWING STEPS DVEL
%      SHOWS INCREMENTAL VELOCITY;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
V       = 0.0;
DV      = DVEL;
ISGDV   = floor(DV/abs(DV));
ERROR   = TOLL + TOLL;
in = 40;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%      AL PUNTO DI INIZIO SWEEP IL MASSIMO NUMERO DI ITERAZIONI VIENE
%      POSTO SCEGLIENDO IL MAGGIORE TRA MAXITE E MAXITS + 1 : POICHE'
%      DURANTE TUTTO LO SWEP SEGUENTE VIENE SEMPRE UTILIZZATO MAXITS
%      E' POSSIBILE IMPORRE UN NUMERO ELEVATO DI ITERAZIONI AL SOLO
%      PUNTO INIZIALE PONENDO MAXITE > MAXITS + 1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
MAXIT   = max(MAXITE, MAXITS+1);
% Initial solution
TN = SVTN;


% Iterative tracking

waith = waitbar(0,'Please wait...');
while (velcyc==1);
    
    waitbar(V / VMAX, waith)
    if V > 0
        temp_model = beam_model;

        beam_model.Aero.state.rho = rho;
        beam_model.Aero.state.AS = V;        
        beam_model.Param.MSOL = 144;
        beam_model.Param.SOL = 144;
        beam_model.Aero.Trim.Select(1) = 1;
        beam_model.Aero.Trim.ID = 1;
        beam_model.Aero.Trim.Type(1) = 1;
        beam_model.Aero.Trim.CID = 0;
        beam_model.Aero.Trim.FM.Fixed = [];
        beam_model.Aero.Trim.FM.Value = [];
        beam_model.Aero.Trim.CS.Fixed = [];
        beam_model.Aero.Trim.CS.Value = [];
        beam_model.Aero.Trim.Link = [];
        beam_model.Aero.Trim.NC = 13;
        beam_model.Aero.Trim.Symm = [];
        beam_model.Aero.Trim.Param(1).data{1, 1} = 'SIDES' ;
        beam_model.Aero.Trim.Param(1).data{1, 2} = 'ROLL' ;
        beam_model.Aero.Trim.Param(1).data{1, 3} = 'PITCH' ;
        beam_model.Aero.Trim.Param(1).data{1, 4} = 'YAW' ;
        beam_model.Aero.Trim.Param(1).data{1, 5} = 'URDD2' ;
        beam_model.Aero.Trim.Param(1).data{1, 6} = 'URDD3' ;
        beam_model.Aero.Trim.Param(1).data{1, 7} = 'URDD4' ;
        beam_model.Aero.Trim.Param(1).data{1, 8} = 'URDD5' ;
        beam_model.Aero.Trim.Param(1).data{1, 9} = 'URDD6' ;
        beam_model.Aero.Trim.Param(1).data{1, 10} = 'CLIMB' ;
        beam_model.Aero.Trim.Param(1).data{1, 11} = 'BANK' ;
        beam_model.Aero.Trim.Param(1).data{1, 12} = 'HEAD' ;
        beam_model.Aero.Trim.Param(1).data{1, 13} = 'THRUST' ;

        beam_model.Aero.Trim.Value(1).data(1, 1) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 2) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 3) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 4) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 5) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 6) = 9.81 ;
        beam_model.Aero.Trim.Value(1).data(1, 7) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 8) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 9) = 0 ;
        beam_model.Aero.Trim.Value(1).data(1, 10) = 0;
        beam_model.Aero.Trim.Value(1).data(1, 11) = 0;
        beam_model.Aero.Trim.Value(1).data(1, 12) = 0;
        beam_model.Aero.Trim.Value(1).data(1, 13) = 0;


        beam_model.Aero.Trim.Ext = [];
        beam_model.Aero.Trim.MINDEX = [];
        beam_model.Aero.Trim.Label_Select{1, 1} = 'Cruise/Climb';
        beam_model.Aero.Trim.MasterSurf{1, 1} = 'elev1r';
        beam_model.Aero.Trim.MasterSurf{1, 2} = 'elev2r';

        fid = beam_model.Param.FID;
        [CAERO.lattice_vlm, CAERO.ref] = vlm_lattice_setup(beam_model.Param.FID, beam_model.Aero.geo, ...
            beam_model.Aero.state, beam_model.Aero.ref);
        CAERO.lattice_vlm.Control = beam_model.Aero.lattice_dlm.Control;

        beam_model.Aero.lattice_vlm = CAERO.lattice_vlm;

        solve_free_lin_trim([1]);

        p0dS = beam_model.Res.Aero.F0_DTrim;

        beam_model = temp_model;

        NMODES = size(beam_model.Res.NDispl, 3);

        K0A = zeros(NMODES, NMODES);

        for j = 1: length(beam_model.Aero.lattice.N)
            K0A = K0A + dot(p0dS(j, :)', beam_model.Aero.lattice.N(j, :)') * beam_model.Aero.Phid{j}' * beam_model.Aero.Nvar{j};     
        end

        Khh = Khh + K0A;
    end
    
    %-------------------- Cycle starts if in=40 ---------------------------
    
    if (in == 40)
        ISTEP = ISTEP + 1;
        V = V + DV;
        if (V > VMAX)
            V = VMAX;
            ISTOP = 1;
        end
    end
    %-------------------- Cycle starts if in=45 ---------------------------
    if ((in == 45)||(in == 40))
        ITER = 0;
        METODO = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      SVQU(12) = AIR DENSITY
    %      DINPRS   = DYNAMIC PRESSURE
    %      CFREQ    = COMPLEX FREQUENCY
    %      FREQ     = NATURAL PULSATION
    %      VALFRE   = ACTUAL REDUCED FREQUENCY VALUE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %-------------------- Cycle starts if in=50 ---------------------------
    CheckExit = 0;
    DINPRS  = 0.5*SVQU(12)*(V*V);
    CFREQ   = TN(nmodc1); % get eigenvalue
    FREQ  = imag(CFREQ);  % extract pulsation
    VALFRE  = FREQ*chord/V; % reduced freq
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %    CHOSES IF INTERPOLATION/EXTRAPOLATION IS NECESSARY
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ((VALFRE < Kfreq(1))||(VALFRE > Kfreq(NKfreq)))
        TIPINT = 'E';
    else
        TIPINT = 'I';
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      EVALUATES AERODYNAMIC INTERPOLATED FORCES
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [FORINT, DERINT] = aero_interp(AER, METODO, VALFRE, Kfreq);
    CFREQ2 = CFREQ * CFREQ;
    CFREQ  = CFREQ + CFREQ; % trick used to have 2*CFREQ which multiplies Mass matrix
    
    C2 = DINPRS*chord/V; % 0.5 * RHOREF * CREF * VEF
    C3 = (.5)*VALFRE;    % K /2
    C4 = 2*DINPRS/V;     % RHOREF * VREF
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      ASSEMBLES COEFFICIENT MATRIX
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Chooses solution method
    
    switch(METODO)
        
        case{1}
            
            % ----------------------- METHOD 1 -------------------------------------
            for I = 1:nmodc
                
                S1 = 0+i*0;
                S2 = 0+i*0;
                S4 = 0+i*0;
                
                for K = 1:nmodc
                    
                    TNT = TN(K); % current eigenvector
                    RIG = Khh(I,K);
                    STDAMP = Chh(I,K);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %      COEFFICIENT MATRIX IS BUILT COUNTING ALSO DAMPING
                    %      TERM (STDAMP)
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    CT = - AERSCA*DINPRS*FORINT(I,K) + Mhh(I,K)*CFREQ2 + (RIG+i*STDAMP);
                    
                    S2 = S2+DERINT(I,K)*TNT;
                    S4 = S4+Mhh(I,K)*TNT;
                    COEF(I,K) = CT;
                    
                    S1 = S1 + CT*TNT; % residual
                    
                end;
                
                S4 = S4*CFREQ;
                
                COEF(I,nmodc1) = S4;
                COEF(I,nmodc2) = (-imag(S4)+i*real(S4)) - AERSCA*C2*S2; % F/S derivative
                COEF(I,nmodc3) = -S1; % residual
                
            end;
            
            S1 = 1+i*0;
            %       normalization condition
            for K = 1:nmodc
                
                TNT = TN(K);
                COEF(nmodc1,K) = TNT + TNT;
                S1 = S1 - TNT*TNT;
                
            end;
            
            COEF(nmodc1,nmodc1) = 0+i*0;
            COEF(nmodc1,nmodc2) = 0+i*0;
            COEF(nmodc1,nmodc3) = S1;
            
            
            %
            %  ----------------------- METHOD 2 -------------------------------------
            %
            %       update residual and keep jacobian constant
        case{2}
            
            for I = 1:nmodc
                
                S1 = 0+i*0;
                
                for K = 1:nmodc
                    
                    TNT = TN(K);
                    RIG = Khh(I,K);
                    STDAMP = Chh(I,K);
                    CT = - AERSCA*DINPRS*FORINT(I,K) + Mhh(I,K)*CFREQ2+ (RIG+i*STDAMP);
                    S1 = S1 + CT*TNT;
                    
                end;
                
                COEF(I,nmodc3) = -S1;
                
            end;
            
            S1 = 1+i*0;
            
            for K=1:nmodc
                
                TNT = TN(K);
                
                S1 = S1 - TNT*TNT;
                
            end;
            
            COEF(nmodc1,nmodc3) = S1;
            
            %
            %----------------------- METHOD 3 -------------------------------------
            %
            
        case{3}
            
            for I=1:nmodc
                
                S1 = 0+i*0;
                S2 = 0+i*0;
                S4 = 0+i*0;
                
                for K=1:nmodc
                    
                    TNT = TN(K);
                    RIG = Khh(I,K);
                    STDAMP = Chh(I,K);
                    
                    CT = - AERSCA*DINPRS*FORINT(I,K) + Mhh(I,K)*CFREQ2+(RIG+i*STDAMP);
                    S2 = S2 + DERINT(I,K)*TNT;
                    S4 = S4 + Mhh(I,K)*TNT;
                    COEF(I,K) = CT;
                    CT = FORINT(I,K);
                    S1 = S1 + CT*TNT;
                    
                end
                
                S4 = S4*CFREQ;
                COEF(I,nmodc1) = S4;
                COEF(I,nmodc2) = (-imag(S4)+i*real(S4)) - AERSCA*C2*S2;
                S1=C4*(S1-C3*S2);
                COEF(I,nmodc3) = AERSCA*S1;
                
            end;
            
            for K=1:nmodc
                
                TNT = TN(K);
                COEF(nmodc1,K) = TNT + TNT;
                
            end;
            
            COEF(nmodc1,nmodc1) = 0+i*0;
            COEF(nmodc1,nmodc2) = 0+i*0;
            COEF(nmodc1,nmodc3) = 0+i*0;
            
    end;
    
    %------------------------------------
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     COEFFICIENT MATRIX FACTORIZATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    if ((METODO == 1)||(METODO == 3))
        
        [COEF1,PERM,KSTOP] = comp_mat_fac(COEF,nmodc,nmodc1);
        COEF=COEF1;
        % Check for error in factorization
        if (KSTOP==1)
            IWAR = 6;
            NSTOP = 'FACTOR';
            velcyc = 0;
            return
        end
    end
    
    L = LV(METODO);
    INIDO = INI(METODO);
    for N = INIDO:3
        
        NGLN = nmodc + N;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  SOLVES COMPLEX SISTEM (nmodc order) GETTING  DELTA#Q AS
        %  FUNCTIONS OF DELTASIGMA AND DELTA OMEGA; SOLUTION IS THEN MOVED
        %  IN THE COLUMN OF KNOWN TERMS...
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        [COEF(:,NGLN)] = comp_linsys_solver(COEF,COEF(:,NGLN),nmodc,nmodc1,PERM);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  SOLVES THE ADDED EQUATION AS A SYSTEM OF TWO REAL EQUATIONS WHERE DELTASIGMA
        %  AND DELTAOMEGA ARE UNKNOWNS, BY SUBSTISTUTION OF DELTAQ OBTAINED BY COMPLEX_LINEAR_SOLVER
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        TEMP1 = real(COEF(nmodc1,NGLN));
        TEMP2 = imag(COEF(nmodc1,NGLN));
        
        for J = 1:nmodc
            
            TEMP1 = TEMP1 - real(COEF(nmodc1,J))*real(COEF(J,NGLN))+ imag(COEF(nmodc1,J))*imag(COEF(J,NGLN));
            TEMP2 = TEMP2-imag(COEF(nmodc1,J))*real(COEF(J,NGLN))- real(COEF(nmodc1,J))*imag(COEF(J,NGLN));
            
        end;
        
        L = L+1;
        A(L) = TEMP1;
        L = L + 1;
        A(L) = TEMP2;
        
        
        
    end;
    
    A11 = A(1);
    A21 = A(2);
    A12 = A(3);
    A22 = A(4);
    A13 = A(5);
    A23 = A(6);
    %-----------------
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     IF METHOD IS 2 DISCRIMINANT IS MAINTAINED CONSTANT;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (METODO == 2)
        
    else
        DISCR = A11*A22 - A12*A21;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %     CHECKS THAT DISCRIMINANT VALUES ARE NOT TOO LITTLE TO CAUSE
        %     ZERO DIVISION ERRORS;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if ( abs(DISCR) >= EPS )
            
        else
            
            IWAR = 7;
            NSTOP = 'DISCR.';
            return;
        end;
    end;
    
    TEMP1 = (A13*A22-A23*A12)/DISCR;
    TEMP2 = (A23*A11-A13*A21)/DISCR;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      TEMP1 AND TEMP2 CONTAINS DELTASIGMA AND DELTAOMEGA RESPECTIVELY,
    %      USED DURING SOLUTION METHOD 1 AND 2. WHEN METHOD 3 IS USED,
    %      SIGMA AND OMEGA DERIVATIVES WITH RESPECT TO VELOCITY ARE HERE
    %      PLACED;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    CT = (TEMP1+i*TEMP2);
    
    if (METODO < 3)
        
        
    else
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      WHEN METHOD IS 3, LOOKS FOR A POINT AT VELOCITY V + DV
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        SVDR(nmodc1) = CT;
        CT = CT*DV;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      WHEN DELTASIGMA AND DELTAOMEGA ARE KNOWN, VALUES ARE
        %      SUBSTITUTED AND FINAL DELTAQ VALUE IS CALCULATED;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end;
    
    TN(nmodc1) = TN(nmodc1) + CT;
    
    for J=1:nmodc
        
        TEM1 = real(COEF(J,nmodc1))*TEMP1 + real(COEF(J,nmodc2))*TEMP2;
        TEM2 = imag(COEF(J,nmodc1))*TEMP1 + imag(COEF(J,nmodc2))*TEMP2;
        CT = -(TEM1+i*TEM2) + COEF(J,nmodc3);
        
        if (METODO < 3)
            
        else
            
            SVDR(J) = CT;
            CT = CT*DV;
            
        end;
        
        TN(J) = TN(J) + CT;
        
    end;
    %---------------------- End of iterative procedure --------------------
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      SAVES DATA FOR METHOD 3
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if (METODO == 3)
        
        SVQU(1) = real(SVTN(nmodc+1)); % get previous damping
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %     SOLUTION AT NEW VELOCITY HAS BEEN CORRECTED, THUS DATA ARE
        %     SAVED AND METHOD 1 IS RESTORED. VELOCITY IS SET = V+DV;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        SVQU(3) = V;
        SVQU(4) = V/FKTS;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %     FREQ    = PULSATION
        %     SVQU(5) = FREQUENZA (HZ When S.I. is used...)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        SVQU(2) = imag(SVTN(nmodc+1));
        FREQ = SVQU(2);
        SVQU(5) = FREQ/PP2;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %     SVQU(6) = EIGENVALUE RATIO (Real/Imag)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        SVQU(6) = SVQU(1)/FREQ;
        TEMP1 = imag(SVDR(nmodc1));
        TEMP2 = real(SVDR(nmodc1));
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %     SVQU(7) = FREQUENCY DERIVATIVE WITH RESPECT TO VELOCITY (KTS),
        %     SVQU(8) = DAMPING DERIVATIVE WITH RESPECT TO VELOCITY (KTS);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        SVQU(7) = TEMP1/PP2*FKTS;
        SVQU(8) = (TEMP2-SVQU(6)*TEMP1)/FREQ*FKTS;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %     CSAVE CONTAINS THE LAST ELEMENT OF SOLUTION VECTOR AND IS
        %     USED AS TERM OF COMPARISON WHED DOUBLING STEP;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        CSAVE = TN(nmodc1);
        
        %        risultati(ISTEP,:)=[VALFRE, SVQU(3), 2*SVQU(6)*PP2, SVQU(5), SVQU(1), SVQU(2), SVQU(8)/FKTS, SVQU(7)/FKTS];
        risultati(ISTEP,:)=[VALFRE, SVQU(3), 2*SVQU(6), SVQU(5), SVQU(1), SVQU(2), SVQU(8)/FKTS, SVQU(7)/FKTS];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      WHEN V < 0 or > VMAX ROUTINE IS ENDED;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if (V <= 0)||(ISTOP == 1)
            velcyc == 0;
            return;
        end;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      STARTS A NEW CYCLE SETTING METHOD = 1 AND V= V+DV;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        velcyc == 1;
        in = 40;
        CheckExit=1;
        
    end;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      THIS CONDITION IS ALWAYS TRUE WHEN SWEEP IS STARTED EXCEPT AT
    %      FIRST STEP;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    if (CheckExit<1)
        
        %------------------- Check after each iteration step --------------
        if (MAXIT == MAXITS)
            
        else
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %      AT FIRST STEP, ERROR CHECK IS DONE ON TOLL PARAMETER;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if (ERROR < TOLL)
                
                IDUETR = 3;
                
            end;
            
            if (ERROR >= TOLL)
                
                IDUETR = 2;
                
            end;
            
            METODO = IDUETR - METODO;
            
            
            if (METODO == 2)
                
                in = 50;
                CheckExit = 1;
                
            end;
        end;
    end;
    
    if (CheckExit<1)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      CONVERGENCY CRITERION: VARIABLE INCREASE,
        %      COMPARISON IS BETWEEN TWO COMPLEX NUMBERS;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        ERROR = abs(TN(nmodc1));
        error = abs((TEMP1+i*TEMP2));
        ERROR = abs((TEMP1+i*TEMP2))/ERROR;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      IF PERCENTAGE VARIATION IS LESS THAN A PRE-DEFINED VALUE,
        %      METHOD IS SWITCHED TO 3 TO START WITH A NEW SWEEP;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %--------------------------- Check % Error -------------------------------
        
        if (ERROR >= ERR)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %      IF ERROR CRITERION IS NOT SATISFIED, ITERATION NUMBER
            %      IS INCREASED, AND A NEW CALCULATION STEP IS TAKEN
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            ITER = ITER + 1;
            
            if ( ITER < MAXIT );
                
                in = 50;
                CheckExit = 1;
                
            else
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %      IF MAXIMUM OTERATIONS NUMBER IS REACHED AND
                %      CONVERGENCE CRITERION IS NOT YET SATISFIED, VELOCITY
                %      IS HALVED AND VELOCITY STEP IS CHECKED...
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                DV = .5*DV;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %      IF DV IS LESS THAN ACCEPTABLE ROUTINE ENDS
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                if (abs(DV) >= DVMIN)
                    %disp('DV >= DVMIN');
                else
                    
                    
                    IWAR = 8;
                    NSTOP = 'DISTEP';
                    velcyc = 0;
                    return;
                end;
                
                V = V - DV;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %      NEW VELOCITY SET. CALCULATES MODE...
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                for I=1:nmodc1
                    
                    TN(I) = SVDR(I)*DV + SVTN(I);
                    
                end;
                
                in = 45;
                CheckExit = 1;
                
            end;
        end;
    end;
    %------------------------------ Check Damping -------------------------
    
    if (CheckExit<1)
        if (ERROR < ERR)
            
            METODO = 3;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %      IF VELOCITY IS LESS THAN A PRE DEFINED VALUE, FLUTTER
            %      VELOCITY IS NOT CALCULATED;
            %      CHECKS DAMPING SINGUM
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            SIGOLD = real(SVTN(nmodc1));
            SIGNEW = real(TN(nmodc1));
            
            if ((SIGNEW > 0)&&(SIGOLD < 0)&&(IMOD ~= IMODFL))
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %      FLUTTER VELOCITY EVALUATED BY LINEAR INTERPOLATION.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                VOLD = SVQU(3);
                VNEW = V;
                DELTAV = VNEW - VOLD;
                DELTAS = SIGNEW - SIGOLD;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %     CHECKING INCREMENTS (IF TOO LITTLE)
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                if (DELTAV < EPS)
                    
                    IWAR = 9;
                    return;
                end;
                
                if (abs(DELTAS) < EPS)
                    
                    IWAR = 10;
                    return;
                end;
                
                if ((IWAR == 9)||(IWAR == 10))
                    
                else
                    
                    VFLINT = (-SIGOLD+DELTAS*VOLD/DELTAV)*(DELTAV/DELTAS);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %      CHECKS IF FLUTTER VELOCITY IS LESS THAN THE LAST FLUTTER VELOCITY OBTAINED;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    if (VFLINT < SVQU(13))
                        SVQU(13) = VFLINT; % save minimum fl speed
                    end
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %      SAVES NEW FLUTTER VELOCITY AND INTERPOLATES
                    %      PULSATION AND MODE AT FLUTTER;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    OMEOLD = imag(SVTN(nmodc1));
                    OMENEW = imag(TN(nmodc1));
                    DELTAO = OMENEW - OMEOLD;
                    OMEFLT = OMEOLD + DELTAO*VFLINT/DELTAV - DELTAO*VOLD/DELTAV;
                    for I = 1:nmodc
                        
                        DELTAQ = TN(I) - SVTN(I);
                        SVTNFL(I) = SVTN(I) + DELTAQ*VFLINT/DELTAV - DELTAQ*VOLD/DELTAV;
                        
                    end;
                    
                    SVTNFL(nmodc1) = (OMEFLT+i*VFLINT);
                    IMODFL = IMOD;
                    FL_DET = true;
                    
                end;
                
            end;
        end;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      SAVES SOLUTION IN VECTOR SVTN
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        SVTN = TN;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      ESEGUE UNA PROIEZIONE IN VELOCITA' CERCANDO DI MANTENERE UN
        %      INCREMENTO DI VELOCITA' DV PIU' ELEVATO POSSIBILE COMPATIBIL-
        %      MENTE COL CRITERIO AMMESSO (DOUBLING)
        %
        %      DOUBLE STEP CRITERION = abs(S(V)-S(V+DV))/CABS(S(V+DV))
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        ERROR = abs(CSAVE-TN(nmodc1))/abs(TN(nmodc1));
        
        if ((ERROR < DOUBLING)&&(MAXIT == MAXITS))
            
            DV = DV + DV;
            
        end;
        
        if (abs(DV) > DVMAX)
            
            DV = ISGDV*DVMAX;
            
        end;
        
        MAXIT = MAXITS;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % WHEN NEW DV HAS BEEN STATED; A CORRECTOR STEP IS TAKEN TO IMPROVE
        % SOLUTION;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        in = 50;
        CheckExit = 1;
        
    end;
    
    %-------------------------- Check 3 End -------------------------------
    
    %----------- Damping check end + Predictor/Corrector -----------------
    
    %------------- Various Check End ---- End 'While ~= CheckExit -------
end
close(waith)
end
