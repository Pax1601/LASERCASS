%**************************************************************************
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
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function       Script called by weight_xml main function
%
%
%   DESCRIPTION:  Execute mass estimation
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%
%
%    REFERENCES:
%
%**************************************************************************
%

if advnmat(n) < 0.0001

    advnmat(n) = 1990; % set to default value

end

% Advanced technology multiplier (atm)
atm = 0.9985^(1.016 * advnmat(n) - 1975);

% VMO speed for bird strike
vmo = FENVVMO(n) * KTAS2MS;

% Dive Mach number at sea level standard
md = FENVVDF(n) * KTAS2MS / 340.3;

% Vehicular lift-curve slope estimate based on Helmbold equation and Prandtl-Glauert compressibility correction
CLaA1 = 5.04 * WINWGAR(n) / (5.04/pi+ (((WINWGAR(n)/cos(DEG2RAD * WINWHSQ(n)))^2)+...
    ((5.04/pi)^2) - (WINWGAR(n) * qxdktms(qxdctks(vmo, 200,0),200,0)^2))^0.5);

% if isnan( MTOWWEI(n) ) || ~MTOWWEI(n)

    % Initial iteration starting point (A380/Antonov An-225 weight, state
    % of art maximum weight for an airplane. This should avoid "complex"
    % weights in the iteration process.
    MTOWWEI(n) = 600000.0;
    mtowi = 0.0;           % assumed initial iteration result

% end

%--------------------------------------------------------------------------
% The weights estimation is tiered into two modes of computation:
% (1) closed form prediction and
% (2) MTOW transcendental prediction
%--------------------------------------------------------------------------
% The closed form mode.
%
%
% compute delta P for cabin pressurisation adjustment
if (opercei(n) < 0.0001 && insttyp(1,n) > 0)

    % default turboprop to 31000 ft service ceiling
    opercei(n) = 310;

elseif (opercei(n) < 0.0001 && insttyp(1,n) < 0.0001)

    % default turbofan to 41000 ft service ceiling
    opercei(n) = 410;

end

if (cabncei(n)<0.0001 && destype(n)<2)

    % default cabin pressure altitude
    cabncei(n) = 8000;

elseif (cabncei(n)<0.0001 && destype(n)==2)

    % default cabin pressure altitude for small bizjets
    cabncei(n) = 7000;

elseif (cabncei(n)<0.0001 && destype(n)>2)

    % default cabin pressure altitude for large bizjets
    cabncei(n) = 6000;
end

% initial guess for maximum altitude
mcbalt = opercei(n);

% quantify the pressure differential
cabnpdf(n) = (qxdthet(cabncei(n) / 100,0)*qxdsigm(cabncei(n)/100,0) -...
    qxdthet(opercei(n),0)*qxdsigm(opercei(n),0)) * SEA_LEVEL_P / PSI2KPA;
deltapc = cabnpdf(n) * PSI2KPA;
%--------------------------------------------------------------------------
% Systems weight estimation.

TSYSWEI(n) = FSYSWEI(n) + FCTLWEI(n) + APUSWEI(n) +...
    INSTWEI(n) + AVIOWEI(n) + HYPNWEI(n) +...
    ELECWEI(n) + ECSAWEI(n) + FURNWEI(n) +...
    MISCWEI(n);

% equivalent internal cabin diameter eqcd
eqcd = (2 * cabnhei(n) + cabnwid(n))/3;

if paxcwei(n) < 0.0001

    % pax coefficient for commercial transport
    kcpax = 55.168 + 10.344 * cabnsab(n) + 13.592 * qxheavy(cabnpas(n), 181,1) +...
        2.616 * qxheavy(cabnpas(n),181,1)*cabnsab(n) - 3.9865 * cabnpas(n)^0.3494;

    % pax coefficient for business jets
    kbpax = 15.561+38.27 * cabnvol(n)/cabnpas(n) + 4297.1*eqcd/cabnpas(n)*(1-4.7923/cabnlgt(n));

    % final value
    paxcwei(n) = kcpax + qxheavy(destype(n),2,1) * (kbpax - kcpax);

end

if TSYSWEI(n)<0.0001

    TSYSWEI(n) = 0.6 * atm * paxcwei(n) * cabnpas(n); % otherwise Scott-Nguyen

end

% LR 18/09/08 added aux landing gear weight
TSYSWEI(n) = TSYSWEI(n) + ALNDGWEI(n);

% Save auxiliary landing gear
PLOTCGS(9, 4, n) = ALNDGWEI(n);

%--------------------------------------------------------------------------
% Paint weight
% LR 17/02/2012 Paint computation moved into a seprated function
% paintwe(n) = 0.12 * TOTLWET(n);
paintwe(n) = paint_weight(TOTLWET(n));

% Completions weight estimation.
% Final completions includes paint and contingency. This is based on Scott-Nguyen

if COPAWEI(n) < 0.0001

    COPAWEI(n) = 0.4 * paxcwei(n) * cabnpas(n) + paintwe(n) + GMEWWEI(n) * contwei(n)/100;

end

PLOTCGS(21,4,n) = COPAWEI(n); % store value for cg analysis

%--------------------------------------------------------------------------
% Payload weight estimation
% The suggested combined weight is 99.7 kg, however, need to recognise that
% typical business jet assumption is 90.7 kg for PAX and baggage.

if (paxwwei(n) < 0.0001)

    paxwwei(n) = 90.719; % assume 200 lb per PAX

end

if (paxiwei(n) < 0.0001)

    paxiwei(n) = 9.07; % assume 20 lb as incremental

end

% note that one passenger person is defined to be at least 88.5 kg


%--------------------------------------------------------------------------
% UPDATE WEIGHT MODULE PARAMS
% PLOTCGS(24,4,n) = cabnpas(n) * 88.5; % store value for cg analysis
PLOTCGS(24,4,n) = cabnpas(n) * paxwwei(n); % store value for cg analysis
MPAXWEI(n) = cabnpas(n) * paxwwei(n); % maximum passenger + baggage weight
MPXIWEI(n) = cabnpas(n) * paxiwei(n); % incremental weight per PAX
PLOTCGS(25,4,n) = MPXIWEI(n) + MPAXWEI(n) - PLOTCGS(24,4,n);% value for cg analysis
MPAYWEI(n) = MPAXWEI(n) + MPXIWEI(n); % combined maximum payload
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
% Fuel weight estimation using Isikveren's method. Scope is given to predict:
% (1) fuel in the wings
% (2) fuel in the centre-tank
% (3) fuel in the auxiliary fuselage tank

DMFWWEI(n) = MFUWWEI(n) + MFCWWEI(n) + MFAWWEI(n); % maximum fuel weight

%
PLOTCGS(18,4,n) = MFUWWEI(n);% store value for cg analysis
PLOTCGS(19,4,n) = MFCWWEI(n);% store value for cg analysis
PLOTCGS(20,4,n) = MFAWWEI(n);% store value for cg analysis

if (mfwdwei(n)<0.0001 && destype(n)<2)

    mfwdwei(n) = DMFWWEI(n) - 2/3*MPAYWEI(n);% estimate max fuel decrement if null

elseif (mfwdwei(n)<0.0001 && destype(n)==2)

    % estimate max fuel decrement if null for small business jet
    mfwdwei(n) = DMFWWEI(n) - MPAYWEI(n) + 362.9;

elseif (mfwdwei(n)<0.0001 && destype(n)>2)

    % estimate max fuel decrement if null for small business jet
    mfwdwei(n) = DMFWWEI(n) - MPAYWEI(n) + 725.8;

end

if (mfwdwei(n)<0.0001)

    mfwdwei(n)=0.0;% cannot increment fuel from max fuel

end

FMPYWEI(n) = DMFWWEI(n) - mfwdwei(n) - rampinc(n); % fuel weight for MZFW to MTOW

%--------------------------------------------------------------------------
% Crew weight estimation. Typical weights from SAWE statistics.
if (cabnatt(n)>0.0001 && cabawei(n)<0.0001)

    cabawei(n) = 75.0;% canned fa weight

end

facrewe(n) = cabawei(n) * cabnatt(n);% compute the fa weight
PLOTCGS(23,4,n) = facrewe(n);% store value for cg analysis

if cabfwei(n)<0.0001

    cabfwei(n) = 85.0;% canned fc weight

end

fccrewe(n) = cabfwei(n) * cabnfcr(n);% compute the fc weight

PLOTCGS(22,4,n) = fccrewe(n);% store value for cg analysis

CREWWEI(n) = facrewe(n) + fccrewe(n);% total crew weight

%--------------------------------------------------------------------------
% Operating items weight estimation.
if OPSIWEI(n)<0.0001

    fmanwei(n) = 18.0; % flight manual weight (40 lb)
    cmiswei(n) = 5.4 * cabnpas(n); % galley consumables/potable water and toilet chem
    OPSIWEI(n) = fmanwei(n) + cmiswei(n) + fuelusu(n); % total operating items

end

% LR 16/10/2012 avoid counting aux landing gear two times
PLOTCGS(17,4,n) = TSYSWEI(n) - ALNDGWEI(n) + mantwei(n) + OPSIWEI(n); % store value for cg analysis

%--------------------------------------------------------------------------
% The transcendental mode. Main convergence algorithm for MTOW estimation.
% Uses simple iteration.
%

mtow1 = WINWARE(n) / REFWARE(n);
mtow2 = WI2WARE(n) / REFWARE(n);
mtowi = 1.0 + MTOWWEI(n);
MZFWWEI(n) = mtowi;% set off the iterations

% Wing no.1 weight prediction
dfac = 1.2051 + 0.0824 * wingplc(n) +...
    0.0241 * spoiler(n)/100 - 0.01755 * qxheavy(undrloc(n),1,1);% design factor for configuration
tfac = 16.5*sin(2*pi * WINWTHB(n));% wing thickness factor

%--------------------------------------------------------------------------
% Fuselage weight prediction
% equivalent diameter if ellipse fuse diameter
equd = (2*fusevma(n) + fusehma(n))/3;
kdp = 1.066 * exp(2.95877e-3 * deltapc);% correction for pressure differential

% correction factor for doors, windows and freight floors
% correction for landing gear, dependent upon landing gear location and wing
% placement
kct = 1.1 + 0.033 * qxheavy(wingplc(1,n),0.25,1)*(1 - qxheavy(undrloc(n),1,1));
% correction for powerplant and landing gear placement
bpic = qxheavy(engeloc(1,n),3,1) * 6.6e-4 * ((forelgt(n) +...
    cabnlgt(n) - 0.4 * fuselgt(n) - 0.65 * WCHDROT(n))^2) / XSECDWF(n) - qxheavy(undrloc(n),1,1) * 0.00331;

%--------------------------------------------------------------------------
% Landing gear weight prediction
lndalph = 587 - 153 * (qxheavy(engeloc(1,n),3,1) +...
    qxheavy(wingplc(1,n),0.25,1) - qxheavy(wingplc(1,n),0.25,1) * qxheavy(undrloc(n),1,1));

%--------------------------------------------------------------------------
% Powerplant weight prediction
PYLNWEI(n) = 0.0;
POWPWEI(n) = 0.0;
NACEWEI(n) = 0.0;

if engenum(2,n)>0.0

    PN=2; % prediction for primary and secondary powerplants

else

    PN=1; % prediction for primary powerplants only

end

for i = 1:PN

    % predict the complete primary and secondary powerplant weight
    [gengewei, gnacewei, gpylnwei, gpropwei] = ewcomp(maxistc, thrtrev, engeloc, insttyp, i, n);
    % prediction for total pylon weight
    PYLNWEI(n) = PYLNWEI(n) + engenum(i,n) * (gpylnwei + gpropwei);
    % prediction for total powerplant weight
    POWPWEI(n) = POWPWEI(n) + engenum(i,n) * gengewei;
    % prediction for total nacelle weight
    NACEWEI(n) = NACEWEI(n) + atm * engenum(i,n) * gnacewei;
    
    % sum the normalised primary powerplant installation weight
    if i<2

        engerel = (PYLNWEI(n) + POWPWEI(n) + NACEWEI(n))/2;
        PLOTCGS(7,4,n) = 2 * engerel; % store value for cg analysis

    else

        PLOTCGS(8,4,n) = PYLNWEI(n) + POWPWEI(n) + NACEWEI(n) - 2*engerel; % store value for cg analysis - S.R.(09/01/11) added 2*  

    end

end

% LR 15/03/10 check on main landing gear presence
if LNDGWEI(n) == 0.
    lndgr_flag = true;
else
    lndgr_flag = false;
    PLOTCGS(6, 4, n) = LNDGWEI(n);    % store value for cg analysis
end

%--------------------------------------------------------------------------
% Start iterative weight refinement

% set an iteration counter
count = 1;

while abs(MTOWWEI(n) - mtowi) > 0.5

    mtowi = MTOWWEI(n);

    %----------------------------------------------------------------------
    % Identify the critical load case, i.e. manouevre or gust

    mu = 2 * wingspn(n) * mtow1 * MZFWWEI(n)/(CLaA1 * WINWARE(n)^2);    % mass parameter
    Kg = 0.88 * mu / (5.3 + mu);    % gust alleviation factor

    % gust load factor at FL 200
    ngust = 1 + Kg * CLaA1 * 1.225 * qxdsigm(200, 0) *...
        15.2 * qxdctks(vmo, 200, 0) * KTAS2MS * WINWARE(n)/(2 * g * mtow1 * MZFWWEI(n));

    % predicted ultimate load factor
    nult = 1.5 * (2.5 * qxheavy(2.5, ngust, 1) + ngust * qxheavy(ngust, 2.5, 1));

    %----------------------------------------------------------------------
    % Wing weight estimation. This is based on Linnell method with some
    % modifications made to the coefficients.
    %
    % Here are the coefficients

    alp = 0.0328;
    bet = 1.5;
    del = 1.5;
    zii = 1.1;
    eps = 1.5;
    phi = 0.656;

    % structural stiffness factor
    sstf = 1 + 1.31 * SEA_LEVEL_D * (vmo^2)/(2*1000*g) * nult^-3;

    % final wing weight prediction
    WINGWEI(n) = atm * alp * dfac * (mtow1 * mtowi * nult *...
        WINWARE(n) * (WINWGAR(n)^bet)*(zii + WINWTAP(n)/2) * (sstf^del)/(tfac * cos(WINWQSW(n)*DEG2RAD)^eps))^phi;

    % single out wing no. 1 weight
    wingmas = WINGWEI(n);

    PLOTCGS(1, 4, n) = WINGWEI(n);% store value for cg analysis

    %----------------------------------------------------------------------
    % Winglet weight estimation. This is based on a simple wetted area
    % premise. An additional contribution accounts for span load revision.

    if wletspn(n)>0.0001

        WLETWEI(n) = 0.5 * WINGWEI(n) * (wletiwt(n) / wletwfc + 0.7 * abs(wgevorx(n)));

    else

        WLETWEI(n) = 0;

    end

    % check if a second wing has been defined
    wi2present(1, n) = aircraft.Wing2.present;
    if wi2present(1, n)
%     if wi2gare(n) > 0.0001

        dfac = 1.2051 + 0.0824 * wi2gplc(n) +...
            0.0241 * spoiler(n)/100 - 0.01755 * qxheavy(undrloc(n),1,1);    % design factor for configuration
        tfac = 16.5 * sin(2 * pi * WI2WTHB(n));   % wing thickness factor

        % structural stiffness factor
        sstf = 1 + 1.31 * SEA_LEVEL_D * (vmo^2)/(2*1000*g) * nult^-3;

        % final wing weight prediction
        WINGWEI(n) = WINGWEI(n) + atm * alp * dfac * (mtow2*mtowi * nult *...
            WI2WARE(n) * (WI2WGAR(n)^bet)* (zii+ WI2WTAP(n)/2)*(sstf^del)/(tfac*cos(WI2WQSW(n)*DEG2RAD)^eps))^phi;
        
        wi2gmas = WINGWEI(n) - wingmas; % single out wing no. 2 weight

        PLOTCGS(2, 4, n) = wi2gmas;   % store value for cg analysis
    else
        PLOTCGS(2, 4, n) = 0.;
    end

    % Landing gear weight estimation. This is based on Linnell method with some
    % modifications made to the coefficients.
    
    if lndgr_flag
        LNDGWEI(n) = atm * lndalph * (mtowi/14000)^1.05;
        PLOTCGS(6, 4, n) = LNDGWEI(n);    % store value for cg analysis
    end
    
    % Predict the engine, nacelle and pylon weights. Only two different
    % types of powerplants are permitted for a given layout.
    % The two sets are called primary and secondary.
%
    % S.R.(09/01/11) changed fixdthw(n) in fixdthw(1,n)
    if fixdthw(1,n) > 0

        PYLNWEI(n)=0.0;
        POWPWEI(n)=0.0;
        NACEWEI(n)=0.0;

        % user has opted for a fixed thrust-to-weight iterative algorithm
        maxistc(1, n) = fixdthw(1,n) * mtowi * 9.81/1000 / (engenum(1,n) + engenum(2,n));
        maxistc(2, n) = maxistc(1, n); % even split with all engines

        %%%%%%%%%%%%%% WARNING %%%%%%%%%%%%%%
        % Here a call to the geotry function is simply replaced
        % The call is commented and the original text of the function is
        % reported inside wb_weight script
        %
        % qgeotry('geom2',n); % call the engine geometric sizing routine
        %

        % ENGINE GEOMETRIC SIZING ROUTINE *********************************

        % Predict the engine and subsequent nacelle diameters.
        % Based on methods developed by Isikveren.

        NACELGT(1:2,n) = 0.0;% initialise for calculation
        MAXISTE = zeros(2,15);% initialise the by-pass ratio emulation variable

        for i=1:PN,

            % length of actual wing chord at engine location

            ENGECHD(i, n) = qxcdcop(abs(engeylc(i,n))*wingspn(n)/2, WSPNMTX, WCHDROT(n), wingtap,n);
            onwings(i, n) = 0;  % initialise variables

            % predict the engine diameter
            ENGEDIA(i, n) = ((2)^-0.5)*(1.73*log(maxistc(i,n))-pi)^0.5;

            nacdase(i, n) = ENGEDIA(i, n) + 0.25;% generic pitot nacelle diameter

            if nacemdi(i, n) < 0.0001

                % For on-wing installations
                if engeloc(i, n) == 1.0 || engeloc(i, n) == 2.0

                    onwings(i, n) = 1;% invoke on-wing installation

                end

                % predicted nacelle maximum diameter
                nacemdi(i, n) = (4-onwings(i,n))*(0.0625 + ENGEDIA(i, n)/4);

            else

                ENGEDIA(i, n) = nacemdi(i,n) - 0.25;% predicted engine max. diameter

            end

            % the propeller diameter has not been given
            if propdia(i, n)<0.0001

                propdia(i, n) = (qxheavy(insttyp(i,n),1,1)*3 +...
                    0.48 * qxheavy(insttyp(i,n),3,1))*ENGEDIA(i,n);

            end

            if insttyp(i, n)>0.0001

                nacetyp(i, n) = 1;% automatically choose long duct for props

            end

            % predicted engine length
            engelgt = (maxistc(i, n)^0.9839)/(2*pi*(1.73*log(maxistc(i, n))-pi));
            nacbase(i, n) = 5/3*engelgt;% generic pitot nacelle length

            if nacefin(i, n) < 0.0001 && engeloc(i, n) > 3

                % predict the nacelle length for S and straight ducts only
                NACELGT(i, n) = (5+nacetyp(i,n)+0.44*qxheavy(insttyp(i,n),3,1) +...
                    7/4*qxheavy(engeloc(i,n),4,1))/3*engelgt + onwings(i,n)*ENGECHD(i,n);

            elseif nacefin(i, n) < 0.0001 && engeloc(i, n) < 4 && insttyp(i, n) < 0.0001

                NACELGT(i, n) = nacbase(i,n);% assume original estimate

            elseif nacefin(i, n) < 0.0001 && engeloc(i, n) < 4 && insttyp(i, n) > 2

                NACELGT(i, n) = 1.81*nacbase(i, n);% assume original estimate

            else

                NACELGT(i, n) = nacefin(i, n) * nacemdi(i, n);% user defined nacelle

            end

            % check to see if a by-pass ratio emulation is required
            if bypasem(i, n) < 0.0001

                % predict the by-pass ratio for turbofans sourced from C.
                % Svoboda, Aircraft Design Journal, 3(2000) 17-31

                bypasem(i, n) = 3.2 + 0.01*(maxistc(i,n)*1000/NEWTON2LB)^0.5;
                MAXISTE(i, n) = maxistc(i, n);

                if insttyp(i, n) > 0.0
                    bypasem(i, n) = 3 * bypasem(i, n);% equivalent by-pass for props
                end

            else

                if insttyp(i, n) > 0.0
                    bypasem(i, n) = bypasem(i, n)/3;% assume fan for modelling purposes
                end

                if bypasem(i, n) < 3.43
                    bypasem(i, n) = 3.43;% minimum fan by-pass ratio for emulation
                end

                MAXISTE(i, n) = 0.001*NEWTON2LB*(100*(bypasem(i,n)-3.2))^2;

                if insttyp(i, n) > 0.0
                    bypasem(i, n) = 3*bypasem(i, n);% assume fan for modelling purposes
                end

            end

            nacefin(i,n) = NACELGT(i,n)/nacemdi(i,n);% compute the nac. fineness
        end

        % end of copy and paste *******************************************

        for i = 1:PN,

            % predict the complete primary and secondary powerplant weight
            [gengewei, gnacewei, gpylnwei, gpropwei] = ewcomp(maxistc, thrtrev, engeloc, insttyp, i, n);

            % prediction for total pylon weight
            PYLNWEI(n) = engenum(i, n)*(gpylnwei+gpropwei) + PYLNWEI(n);

            % prediction for total powerplant weight
            POWPWEI(n) = engenum(i, n)*gengewei + POWPWEI(n);

            % prediction for total nacelle weight
            NACEWEI(n) = atm*engenum(i, n)*gnacewei + NACEWEI(n);

            % sum the normalised primary powerplant installation weight

            if i < 2

                engerel = (PYLNWEI(n) + POWPWEI(n) + NACEWEI(n))/2;
                % store value for cg analysis
                PLOTCGS(7, 4, n) = 2 * engerel;

            else

                % store value for cg analysis
                PLOTCGS(8, 4, n) = PYLNWEI(n) + POWPWEI(n) + NACEWEI(n) - 2*engerel; % S.R.(09/01/11) added 2*  

            end

        end

    end

    %----------------------------------------------------------------------
    % credit given for bending alleviation if TOLS configuration selected

    %     if wingcfg(n) > 0.0001 && wi2gcfg(n) > 0.0001

    if wingcfg(n) ~= 0 && wi2gcfg(n) ~= 0 % 0 = conventional wing

        % wing structural weight effect on bending due to other oblique wing
        wingrel = -0.8 * wi2gmas/mtowi;
        wi2grel = -0.8 * wingmas/mtowi;
        powrrel = -3 * engerel/(mtowi*WINWYBR(n)) * engeylc(1,n)^2;

        wrelief = 1 + wingrel + powrrel;
        wr2lief = 1 + wi2grel + powrrel;

        WINGWEI(n) = wrelief * wingmas + wr2lief * wi2gmas;
        PLOTCGS(1, 4, n) = wrelief * wingmas;% store value for cg analysis
        PLOTCGS(2, 4, n) = wr2lief * wi2gmas;% store value for cg analysis

    end

    %----------------------------------------------------------------------
    % Fuselage weight estimation. This is based on Linnell method with some
    % modifications made to the coefficients.
    % Here are the coefficients

    alp = 0.585;
    bet = 0.5;
    del = 3.75;
    zii = 0.75;
    eps = 0.4;
    phi = 0.4;
    fii = 0.45;
    gam = 0.3;

    mtowb = mtowi - (WINGWEI(n) + WLETWEI(n) + (1-undrloc(n)) * LNDGWEI(n) +...
        (POWPWEI(n)+NACEWEI(n)) * (1-qxheavy(engeloc(1,n),3,1)) + MFUWWEI(n));

    Pcor = bpic * mtowi; % correction for powerplant and landing gear placment

    % fuselage weight prediction
    FUSEWEI(n) = atm * (Pcor + alp * kdp * kct * ((fuselgt(n)/equd)^bet)*...
        ((fuselgt(n)*equd)^zii)*((nult/del)^eps)*(mtowb^phi)*(md/fii)^gam);

    % If complex value is reached exit iteration process and save previous
    % fuselage weight
    if ~isreal(FUSEWEI)
        warning('warn:wb_weight',...
            'Fuselage weight reached a complex weight, exiting process at %2d-th iteration', count);
        fprintf('Fuselage weight value:\n');
        disp(FUSEWEI(n));
        break
    end
    PLOTCGS(5, 4, n) = FUSEWEI(n);% store value for cg analysis
    %----------------------------------------------------------------------
    % Horizontal tail weight estimation.

    if (htalare(n) > 0.0001)

        alp = 4.4;
        bet = 1e4;
        nhtlwei = mtowi * nult * (htalare(n)^2) * htalgar(n);
        dhtlwei = bet * (wingare(n) + wi2gare(n)) * HTALTHB(n) * (cos(HTAWQSW(n)*DEG2RAD))^1.8;

        HTALWEI(n) = atm * alp*(nhtlwei/dhtlwei)^0.56;

    else

        HTALWEI(n)=0.0; % no horizontal tail is specified

    end

    PLOTCGS(3,4,n) = HTALWEI(n); % store value for cg analysis

    %----------------------------------------------------------------------
    % Vertical tail weight estimation.

    if vtalare(n) > 0.0001

        alp = 0.8926 + taillay(n) * 0.6514;% weight penalty adjusted for T-tail

        if htalare(n) < 0.0001

            % If a horizontal tail is not selected, an equivalent method is required to
            % to predict the vertical tail weight based on the above methodology. The
            % idea is to use the horizontal tail weight equation on the vertical tail
            % and use the normalised weight-area relationship for the final vertical tail
            % result.

            nhtlwei = mtowi * nult * (vtalare(n)^2) * vtalgar(n);
            dhtlwei = 1e4 * (wingare(n) + wi2gare(n)) * VTALTHB(n)*(cos(VTAWQSW(n)*DEG2RAD))^1.8;
            bet = (atm*4.4*(nhtlwei/dhtlwei)^0.56)/vtalare(n);
            nvtlwei = VTALTHB(n) * cos(VTAWQSW(n)*DEG2RAD);

        else

            nvtlwei = HTALTHB(n) * cos(HTAWQSW(n)*DEG2RAD);
            bet = HTALWEI(n)/htalare(n);

        end

        dvtlwei = VTALTHB(n) * cos(VTAWQSW(n) * DEG2RAD);
        VTALWEI(n) = atm * alp * bet * vtalare(n) * nvtlwei/dvtlwei;

    else

        VTALWEI(n) = 0.0; % no vertical tail is specified

    end

    PLOTCGS(4, 4, n) = VTALWEI(n);% store value for cg analysis

    % Twin_tail = 0/1 (0 = no twin tail; 1 = twin tail present)
    if Twin_tail

        PLOTCGS(4,  4, n) = PLOTCGS(4, 4, n)/2;
        PLOTCGS(10, 4, n) = PLOTCGS(4, 4, n);

    end

    %----------------------------------------------------------------------
    % Ventral fin weight estimation. This is based on a simple wetted area
    % premise.
    VENTWEI(n) = 0.0;

    %----------------------------------------------------------------------
    % Dorsal fin weight estimation.
    if vtaldlc(n)>0.0001

        dorswei = VTALWEI(n)/VTALWET(n) * dorswet;
        VTALWEI(n) = VTALWEI(n) + dorswei;

    end

    %----------------------------------------------------------------------
    % Canard weight estimation (copied from HT).

    if canard_flag

        alp = 4.4;
        bet = 1e4;
        ncandwei = mtowi * nult * (candare(n)^2) * candgar(n);
        dcandwei = bet * (wingare(n) + wi2gare(n)) * CANDTHB(n) * (cos(CANDWQSW(n)*DEG2RAD))^1.8;

        CANWEI(n) = atm * alp*(ncandwei/dcandwei)^0.56;

    else

        CANWEI(n) = 0.0; % no canard is specified

    end

    PLOTCGS(11, 4, n) = CANWEI(n); % store value for cg analysis
   
    %----------------------------------------------------------------------
    % SR 31/05/2010 - Tailbooms weight estimation (from Howe)

    if tailbooms_flag

        TAILBWEI(n)=0.23*sqrt((FENVVDF(n)*KTAS2MS)*tlbmlen(n)/2*tlbmdim(n))*(pi*tlbmdim(n)*tlbmlen(n)+pi*tlbmdim(n)/2)^1.2;

        if tlbmsys(n) == 1,
            TAILBWEI(n) = TAILBWEI(n)*2;
        end

    else

        TAILBWEI(n)=0;

    end

    PLOTCGS(12, 4, n) = TAILBWEI(n); % store Tailbooms mass value for cg analysis

    %----------------------------------------------------------------------
    % Green Manufacturer's Weight Empty
    GMEWWEI(n) = WINGWEI(n) + WLETWEI(n) + HTALWEI(n) +...
        VTALWEI(n) + VENTWEI(n) + CANWEI(n) + TAILBWEI(n) + LNDGWEI(n) +...
        PYLNWEI(n) + POWPWEI(n) + NACEWEI(n) +...
        FUSEWEI(n) + TSYSWEI(n) + mantwei(n);
    %----------------------------------------------------------------------
    % Predict the spec Operational Weight Empty
    DOEWWEI(n) = GMEWWEI(n) + COPAWEI(n) + CREWWEI(n) + OPSIWEI(n);
    %----------------------------------------------------------------------
    % Predict the spec Maximum Zero Fuel Weight.
    MZFWWEI(n) = DOEWWEI(n) + MPAYWEI(n);
    %----------------------------------------------------------------------
    % Predict the spec Maximum Takeoff Weight.
    MTOWWEI(n) = MZFWWEI(n) + FMPYWEI(n);

    % Give a weight resume at each iteration
    fprintf('\n\tIteration #%2d\n', count);
    fprintf('\tStructural weights:\n');
    fprintf('\tWing weight [Kg]:                       %10.2f\n', WINGWEI(n));
    fprintf('\tHT weight [Kg]:                         %10.2f\n', HTALWEI(n));
    fprintf('\tVT weight [Kg]:                         %10.2f\n', VTALWEI(n));
    fprintf('\tCanard weight [Kg]:                     %10.2f\n', CANWEI(n));
    fprintf('\tTailbooms weight [Kg]:                  %10.2f\n', TAILBWEI(n));
    fprintf('\tFuselage weight [Kg]:                   %10.2f\n', FUSEWEI(n));
    fprintf('\tEngine1 group weight [kg]:              %10.2f\n', PLOTCGS(7, 4, n));
    fprintf('\tEngine2 group weight [kg]:              %10.2f\n', PLOTCGS(8, 4, n));
    fprintf('\tLanding gear weight  [kg]:              %10.2f\n', LNDGWEI(n));
    fprintf('\tOperational empty weight [Kg]:          %10.2f\n', DOEWWEI(n));
    fprintf('\tGreen Manufactures Empty weight [Kg]:   %10.2f\n', GMEWWEI(n));
    fprintf('\tMax zero fuel weight [Kg]:              %10.2f\n', MZFWWEI(n));
    fprintf('\tMax Take-off weight [Kg]:               %10.2f\n', MTOWWEI(n));
    fprintf('\tMax Fuel weight @ MTOW [Kg]:            %10.2f\n', FMPYWEI(n));
    fprintf('\tMax Fuel weight (nominal) [Kg]:         %10.2f\n', DMFWWEI(n));

    % Update counter
    count = count + 1;

end

%--------------------------------------------------------------------------
% Predict the spec Maximum Ramp Weight.
RAMPWEI(n) = MTOWWEI(n) + rampinc(n);

%--------------------------------------------------------------------------
% Merit functions.
%
% maximum payload per PAX
MPAPWEI(n) = paxwwei(n) + paxiwei(n);

% payload to MTOW at MFW
PMFWWEI(n) = RAMPWEI(n) - (DMFWWEI(n) + DOEWWEI(n));

% load factor to MTOW at MFW
% LR 1/3/2012 avoid NaN of Inf when no passengers are specified
if cabnpas(n)
    LMFWWEI(n) = PMFWWEI(n) / (cabnpas(n) * MPAPWEI(n))*100;
else
    LMFWWEI(n) = 0;
end

% structural efficiency
MBOMWEI(n) = DOEWWEI(n) / MTOWWEI(n)*100;

% wing loading W/S(gross)
MWOSWEI(n) = MTOWWEI(n) / (wingare(n)+wi2gare(n));

% thrust-to-weight ratio
MTTWWEI(n) = 101.94 * (engenum(1,n) * maxistc(1,n) +...
    engenum(2,n) * maxistc(2,n)) / MTOWWEI(n);

% end of wb_weight function