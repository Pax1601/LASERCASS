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
%   DESCRIPTION:   Execute the centre of gravity prediction routines
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

if engenum(2, DRWSELN) > 0.0
    PN = 2;% prediction for primary and secondary powerplants
else
    PN = 1;% prediction for primary powerplants only
end


% Added here a call to qgeotry
% This routine computes/checks the cabin & baggage dimensions and properties

% Predict cabin height
if cabnhei(n) < 0.0001 && fusevma(n) > 0.0001

    if fusevmi(n) > 0.5
        cabnhei(n) = fusevmi(n)*fusevma(n) - 0.15;  % double-bubble cabin height
    else
        if wingplc(n) < 0 || wingplc(n) > 1
            cabnhei(n) = fusevma(n) - 0.2;  % wingbox does not go through x-section
        else
            if wingplc(n) > 0.5
                % circular x-section cabin height for high wings
                cabnhei(n) = fusevma(n)*wingplc(n) - wingthk(2,1)*WCHDROT(n)/2 - 0.2;
            else
                % circular x-section cabin height for low wings
                cabnhei(n) = fusevma(n)*(1-wingplc(n)) - wingthk(2,1)*WCHDROT(n)/2 - 0.2;
            end
        end
    end

end

% Predict cabin max width
if cabnwid(n) < 0.0001 && fusehma(n) > 0.0001
    cabnwid(n) = fusehma(n) - 0.2;
end

% Predict cabin floor width
if cabnfwd(n) < 0.0001 && fusevma(n) > 0.0001

    if fusevmi(n) > 0.5

        sweptan = atan(2*fusevma(n)*(fusevmi(n)-0.5)/fusehma(n)) + pi;
        sweptrd = CROSXCF(1) + CROSXCF(2)*sin(sweptan) + CROSXCF(3)*cos(2*sweptan);

        % double-bubble floor width
        cabnfwd(n) = 2*sweptrd*cos(sweptan-pi) - 0.2;

    else

        % predict the floor width
        cabnfwd(n) = ((cabnwid(n)^2)-4*(((fusevma(n) - wingthk(2,1)*WCHDROT(n)- 0.2)/2)^2))^0.5;

    end

end

% Predict cabin volume
if ~cabnvol

    hbar = 0.5*( (cabnwid(n)^2) - cabnfwd(n)^2 )^0.5;
    cabth = atan(2*hbar/cabnfwd(n));
    cabnvol(n) = cabnlgt(n)/4 * (cabnwid(n)*(pi*cabnhei(n) + cabth*cabnwid(n))+...
        hbar*(2*cabnfwd(n) - pi*cabnwid(n)));

end

% Predict baggage apex: baggage is located underfloor and can be
% shifted aft/rear along fuselage following baggtyp (installation type)
% values.
%%%%SR if baggapx(n) < 0.0001
    if baggapx(n) < 0.0001

    if baggtyp(n) ~= 0
        % estimate the underfloor baggage 1 apex
        baggapx(n) = 1.5*forelgt(n)/fuselgt(n);
    else
        % estimate aft baggage apex
        baggapx(n) = (forelgt(n) + cabnlgt(n))/fuselgt(n);
    end

    end

%%%%SR end

% Predict baggage volume and combined length
if baggvol(n) == -1

    % upper limit of baggage vol per PAX = 13.5 cu.ft
    maxibag = 0.382; % m^3
    scalbag = 1;     % scale factor

    % aircraft.miscellaneous.Design_classification = destype
    if destype(n) < 2
        scalbag = 0.48;              % scale for commercial transportation
    elseif destype(n) == 2
        scalbag = 0.63;              % scale for business jets larger than super midsize
    end

    baggvol(n) = cabnpas(n)*scalbag*maxibag;                    % baggage volume prediction

    if baggtyp(n) ~= 0
        baggxsc = 2*(fusevma(n)-cabnhei(n)-0.45)*cabnfwd(n)/3;	% x-sec area u/floor
        bagglgt(n) = baggvol(n)/baggxsc;                        % estimate u/floor total baggage length
    else
        bagglgt(n) = baggvol(n)/cabnvol(n)*cabnlgt(n);          % est. aft baggage length
    end

elseif baggvol(n) < 0.0001 && bagglgt(n) > 0.0001

    if baggtyp(n)~=0
        baggxsc = 2*(fusevma(n)-cabnhei(n)-0.45)*cabnfwd(n)/3;	% x-sec area u/floor
        baggvol(n) = baggxsc*bagglgt(n);                        % estimate u/floor baggage volume
    else
        baggvol(n) = cabnvol(n)/cabnlgt(n)*bagglgt(n);          % estimate baggage volume
    end

elseif baggvol(n) > 0.0001 && bagglgt(n) < 0.0001

    if baggtyp(n) ~= 0
        baggxsc = 2*(fusevma(n)-cabnhei(n)-0.45)*cabnfwd(n)/3;	% x-sec area u/floor
        bagglgt(n) = baggvol(n)/baggxsc;                        % estimate u/floor baggage length
    else
        bagglgt(n) = baggvol(n)/cabnvol(n)*cabnlgt(n);          % estimate baggage length
    end

end



% Wing no.1 longitudinal and vertical cg (hybrid Torenbeek & SAWE)

if REFWYBR(DRWSELN) > wingkln(1, DRWSELN)

    % MAC is located after kink 1 zone
%%%%SR    if WINGBAX(DRWSELN) < 0.0001 && wingcfg(DRWSELN) ~= 1

        WINGBAX(DRWSELN) = ( fuselgt(DRWSELN)*wingapx(DRWSELN) +...
            WSPNMTX(1)*tan(kdrang*winglsw(1, DRWSELN)) +...
            (REFWYBR(DRWSELN) - ...
            wingkln(DRWSELN))*WSPNMTX(2)*tan(kdrang*winglsw(2,DRWSELN)) + ...
            0.49*qxcdcop( REFWYBR(DRWSELN)*wingspn(DRWSELN)/2, WSPNMTX, ...
            WCHDROT(DRWSELN), wingtap, DRWSELN ) )/fuselgt(DRWSELN);

%%%%SR    end

%%%%SR    if WINGBAZ(DRWSELN) < 0.0001

        WINGBAZ(DRWSELN) = ( fusevma(DRWSELN)*(wingplc(DRWSELN)-0.5) + ...
            WSPNMTX(1)*tan(kdrang*wingdih(1,DRWSELN)) + ...
            (REFWYBR(DRWSELN)- ...
            wingkln(DRWSELN))*WSPNMTX(2)*tan(kdrang*wingdih(2, ...
            DRWSELN)) )/fusevma(DRWSELN);

%%%%SR    end

else

    % otherwise MAC is in kink 1 zone
%%%%SR    if WINGBAX(DRWSELN) < 0.0001 && wingcfg(DRWSELN) ~= 1

        WINGBAX(DRWSELN) = ( fuselgt(DRWSELN)*wingapx(DRWSELN) + ...
            REFWYBR(DRWSELN)*WSPNMTX(1)*tan(kdrang*winglsw(1,DRWSELN)) + ...
            0.49*qxcdcop( REFWYBR(DRWSELN)*wingspn(DRWSELN)/2, WSPNMTX, ...
            WCHDROT(DRWSELN), wingtap, DRWSELN ) )/fuselgt(DRWSELN);

%%%%SR    end

%%%%SR    if WINGBAZ(DRWSELN) < 0.0001

        WINGBAZ(DRWSELN) = ( fusevma(DRWSELN)*(wingplc(DRWSELN)-0.5) + ...
            REFWYBR(DRWSELN)*WSPNMTX(1)*tan(kdrang*wingdih(1, ...
            DRWSELN)) )/fusevma(DRWSELN);

%%%%SR    end

end

if wingcfg(DRWSELN) ~= 0

    WINGBAX(DRWSELN) = ( fuselgt(DRWSELN)*wingapx(DRWSELN) + ...
        (wingspf(1,DRWSELN) + ...
        wingspa(1,DRWSELN))/2*WCHDROT(DRWSELN) )/fuselgt(DRWSELN);

end

% Output
PLOTCGS(1, 1, DRWSELN) = WINGBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(1, 2, DRWSELN) = WINGBAY(DRWSELN)*fuselgt(DRWSELN);% y-axis
PLOTCGS(1, 3, DRWSELN) = WINGBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis



%==========================================================================
% Originally commented in the code - modified 06/11/2008

wi2present(1,n) = aircraft.Wing2.present;
if wi2present(1,n) == 1
% if wi2gare(DRWSELN) > 0.0001

    % Added these variable to handle Wing2 case
    WS2NMTX = aircraft.Wing2.Span_matrix_partition_in_mid_outboard;
    wi2glsw(1, n) = aircraft.Wing2.LE_sweep_inboard;
    wi2glsw(2, n) = aircraft.Wing2.LE_sweep_midboard;
    wi2glsw(3, n) = aircraft.Wing2.LE_sweep_outboard;
    wi2gspn(1, n) = aircraft.Wing2.Span;
    WI2WLSW(1, n) = aircraft.Wing2.Reference_LE_sweep;
    wi2gapx(1, n) = aircraft.Wing2.apex_locale;
    WI2WYBR(1, n) = aircraft.Wing2.Reference_non_dim_y_bar;
    WI2WMAC(1, n) = aircraft.Wing2.Reference_MAC;


    wi2apxr = WS2NMTX(1)*tan(kdrang*wi2glsw(1, DRWSELN))+ ...
        WS2NMTX(2)*tan(kdrang*wi2glsw(2, DRWSELN))+ ...
        WS2NMTX(3)*tan(kdrang*wi2glsw(3, DRWSELN));
    wi2apxf = wi2gspn(DRWSELN)/2*tan(kdrang*WI2WLSW(DRWSELN));
    ne2apex = wi2gapx(DRWSELN) + (wi2apxr - wi2apxf)/fuselgt(DRWSELN);

    % wing no.2 longitudinal cg (Torenbeek & SAWE)
    WI2GBAX(DRWSELN) = (ne2apex + WI2WYBR(DRWSELN)*wi2gspn(DRWSELN)/2*tan(kdrang*WI2WLSW(DRWSELN))+ ...
        0.4*WI2WMAC(DRWSELN))/fuselgt(DRWSELN);

    PLOTCGS(2, 1, DRWSELN) = WI2GBAX(DRWSELN)*fuselgt(DRWSELN);

end
%==========================================================================

% h-tail longitudinal and vertical cg (hybrid Torenbeek & SAWE)

if htalare(DRWSELN) > 0.0001

    if HTALYBR(DRWSELN) > htalkln(1, DRWSELN)

        % MAC is located after kink 1 zone
%%%%SR        if HTALBAX(DRWSELN) < 0.0001

            if taillay(1, n)
                htapx = HTAPEXX;
            else
                htapx = fuselgt(DRWSELN)*htalapx(DRWSELN);
            end

            HTALBAX(DRWSELN) = ( htapx + ...
                htalkln(DRWSELN)*htalspn(DRWSELN)/2*tan(kdrang*htallsw(1,DRWSELN)) + ...
                (HTALYBR(DRWSELN)- ...
                htalkln(DRWSELN))*htalspn(DRWSELN)/2*tan(kdrang*htallsw(3,DRWSELN)) + ...
                0.42*tailchord(htaltap(1,DRWSELN),htaltap(3,DRWSELN), HCHDROT(DRWSELN),HTALYBR(DRWSELN),htalkln(DRWSELN)) )/fuselgt(DRWSELN);
%%%%SR        end

%%%%SR        if HTALBAZ(DRWSELN) < 0.0001

            if taillay(1, n)
                htver = HTAPEXZ;
            else
                htver = fusevma(DRWSELN)*htalver(DRWSELN);
            end

            HTALBAZ(DRWSELN) = ( htver + ...
                HSPNMTX(1)*tan(kdrang*htaldih(1,DRWSELN)) + ...
                (HTALYBR(DRWSELN) - ...
                htalkln(DRWSELN))*HSPNMTX(2)*tan(kdrang*htaldih(2, ...
                DRWSELN)) )/fusevma(DRWSELN);

%%%%SR        end

    else

        % otherwise MAC is in kink 1 zone
%%%%SR        if HTALBAX(DRWSELN) < 0.0001

            HTALBAX(DRWSELN) = ( fuselgt(DRWSELN)*htalapx(DRWSELN) + ...
                HTALYBR(DRWSELN)*tan(kdrang*htallsw(1,DRWSELN))*htalspn(DRWSELN)/2 + ...
                0.42*tailchord(htaltap(1,DRWSELN),htaltap(3,DRWSELN), HCHDROT(DRWSELN),HTALYBR(DRWSELN),htalkln(DRWSELN)) )/fuselgt(DRWSELN);

%%%%SR        end

%%%%SR        if HTALBAZ(DRWSELN) < 0.0001

            HTALBAZ(DRWSELN) = ( fusevma(DRWSELN)*htalver(DRWSELN) + ...
                HTALYBR(DRWSELN)*HSPNMTX(1)*tan(kdrang*htaldih(1, ...
                DRWSELN)) )/fusevma(DRWSELN);

%%%%SR        end

    end

end

% Output
PLOTCGS(3, 1, DRWSELN) = HTALBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(3, 2, DRWSELN) = HTALBAY(DRWSELN)*fuselgt(DRWSELN);% y-axis
PLOTCGS(3, 3, DRWSELN) = HTALBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% Vertical tail longitudinal and vertical cg (hybrid Torenbeek & SAWE)
if vtalare(DRWSELN) > 0.0001

    if VTALYBR(DRWSELN) > vtalkln(1, DRWSELN)

        % MAC is located after kink 1 zone
%%%%SR        if VTALBAX(DRWSELN) < 0.0001

            VTALBAX(DRWSELN) = ( fuselgt(DRWSELN)*vtalapx(DRWSELN) + ...
                 vtalkln(DRWSELN)*vtalspn(DRWSELN)*tan(kdrang*vtallsw(1,DRWSELN)) + ...
                (VTALYBR(DRWSELN) - ...
                vtalkln(DRWSELN))*vtalspn(DRWSELN)*tan(kdrang*vtallsw(3,DRWSELN)) + ...
                0.42*tailchord(vtaltap(1,DRWSELN),vtaltap(3,DRWSELN), VCHDROT(DRWSELN),VTALYBR(DRWSELN),vtalkln(DRWSELN)) )/fuselgt(DRWSELN);

%%%%SR        end

    else

        % otherwise MAC is in kink 1 zone
%%%%SR        if VTALBAX(DRWSELN) < 0.0001

            VTALBAX(DRWSELN) = ( fuselgt(DRWSELN)*vtalapx(DRWSELN) + ...
                VTALYBR(DRWSELN)*vtalspn(DRWSELN)*tan(kdrang*vtallsw(1,DRWSELN)) + ...
                0.42*tailchord(vtaltap(1,DRWSELN),vtaltap(3,DRWSELN), VCHDROT(DRWSELN),VTALYBR(DRWSELN),vtalkln(DRWSELN)) )/fuselgt(DRWSELN);

%%%%SR        end

    end

    VTALBAZ(DRWSELN) = ( fusevma(DRWSELN)*vtalver(DRWSELN) + ...
        VTALYBR(DRWSELN)*vtalspn(DRWSELN) )/fusevma(DRWSELN);

end

% Output
PLOTCGS(4, 1, DRWSELN) = VTALBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
% PLOTCGS(4, 2, DRWSELN) = Twin_tail_span*wingspn(DRWSELN);  % y-axis
PLOTCGS(4, 2, DRWSELN) = 0;  % y-axis
PLOTCGS(4, 3, DRWSELN) = VTALBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis

% Twin_tail = 0/1 (0 = no twin tail; 1 = twin tail present)

if Twin_tail

%
% Modified SR 11/06/2010
% Y position of CoG assigned explicitely otherwise always =0

    VTALBAY(DRWSELN) = Twin_tail_span*wingspn/2;

    PLOTCGS(4,  2, DRWSELN) =  Twin_tail_span*wingspn(DRWSELN)/2;% y-axis
    PLOTCGS(10, 1, DRWSELN) =  PLOTCGS(4, 1, DRWSELN);         % x-axis
    PLOTCGS(10, 2, DRWSELN) = -Twin_tail_span*wingspn(DRWSELN)/2;% y-axis
    PLOTCGS(10, 3, DRWSELN) =  PLOTCGS(4, 3, DRWSELN);         % z-axis

end

% LR added on 16/02/2010
% Canard longitudinal and vertical cg (copied from HT)
%
if canard_flag

    if CANDYBR(DRWSELN) > candkln(1, DRWSELN)

        % MAC is located after kink 1 zone
%%%%SR        if CANDBAX(DRWSELN) < 0.0001
          
            candapx = fuselgt(DRWSELN)*candapx(DRWSELN);

            CANDBAX(DRWSELN) = ( candapx + ...
                candkln(DRWSELN)*candspn(DRWSELN)/2*tan(kdrang*candlsw(1,DRWSELN)) + ...        
                (CANDYBR(DRWSELN)- ...
                candkln(DRWSELN))*candspn(DRWSELN)/2*tan(kdrang*candlsw(3,DRWSELN)) + ...
                0.42*tailchord(candtap(1,DRWSELN),candtap(3,DRWSELN), CCHDROT(DRWSELN),CANDYBR(DRWSELN),candkln(DRWSELN)) )/fuselgt(DRWSELN);    
            
%%%                0.42*qxcdcop( CANDYBR(DRWSELN)*candspn(DRWSELN)/2, CSPNMTX, ...
%%%                CCHDROT(DRWSELN), candtap, DRWSELN ) )/fuselgt(DRWSELN);
           
%%%%SR        end

%%%%SR        if CANDBAZ(DRWSELN) < 0.0001

            candver = fusevma(DRWSELN)*candver(DRWSELN);

            CANDBAZ(DRWSELN) = ( candver + ...
                HSPNMTX(1)*tan(kdrang*canddih(1,DRWSELN)) + ...
                (CANDYBR(DRWSELN) - ...
                candkln(DRWSELN))*CSPNMTX(2)*tan(kdrang*canddih(2, ...
                DRWSELN)) )/fusevma(DRWSELN);

%%%%SR        end

    else

        % otherwise MAC is in kink 1 zone
%%%%SR        if CANDBAX(DRWSELN) < 0.0001

            CANDBAX(DRWSELN) = ( fuselgt(DRWSELN)*candapx(DRWSELN) + ...
            CANDYBR(DRWSELN)*tan(kdrang*candlsw(1,DRWSELN))*candspn(DRWSELN)/2 + ...
            0.42*tailchord(candtap(1,DRWSELN),candtap(3,DRWSELN), CCHDROT(DRWSELN),CANDYBR(DRWSELN),candkln(DRWSELN)) )/fuselgt(DRWSELN);    
            
%%%%        0.42*qxcdcop(CANDYBR(DRWSELN)*candspn(DRWSELN)/2, CSPNMTX, ...
%%%%                CCHDROT(DRWSELN), candtap, DRWSELN) )/fuselgt(DRWSELN);
            
%%%%SR        end

%%%%SR        if CANDBAZ(DRWSELN) < 0.0001

            CANDBAZ(DRWSELN) = ( fusevma(DRWSELN)*candver(DRWSELN) + ...
                CANDYBR(DRWSELN)*CSPNMTX(1)*tan(kdrang*canddih(1, ...
                DRWSELN)) )/fusevma(DRWSELN);

%%%%SR        end

    end

end

% Output
PLOTCGS(11, 1, DRWSELN) = CANDBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(11, 2, DRWSELN) = CANDBAY(DRWSELN)*fuselgt(DRWSELN);% y-axis
PLOTCGS(11, 3, DRWSELN) = CANDBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


%--------------------------------------------------------------------------
% SR 31/05/2010 - Added Tailbooms

if tailbooms_flag
    TLBMBAX(DRWSELN) = tlbmlox(DRWSELN)+ 0.5*tlbmlen(DRWSELN)/fuselgt(DRWSELN);
    TLBMBAZ(DRWSELN) = tlbmloz(DRWSELN);
    TLBMBAY(DRWSELN) = tlbmloy(DRWSELN)*wingspn/2;
else
    TLBMBAX(DRWSELN) = 0.0;
    TLBMBAZ(DRWSELN) = 0.0;
    TLBMBAY(DRWSELN) = 0.0;
end

%Output

PLOTCGS(12, 1, n) = TLBMBAX(DRWSELN)* fuselgt(DRWSELN);
PLOTCGS(12, 2, n) = TLBMBAY(DRWSELN)* wingspn(DRWSELN)/2;
PLOTCGS(12, 3, n) = TLBMBAZ(DRWSELN)* fusevma(DRWSELN);


% fuselage longitudinal and vertical cg (Torenbeek)

%%%%SR if FUSEBAX(DRWSELN) < 0.0001

    fuseslt = fuselgt(DRWSELN) - forelgt(DRWSELN) - aftslgt(DRWSELN);
    fusecgi = fuseslt/2 + forelgt(DRWSELN);
    % fuselage longitudinal cg (geometric handbook)
    FUSEBAX(DRWSELN) = ( fusecgi - (forelgt(DRWSELN)^2)/(forelgt(DRWSELN) + ...
        2*fuseslt + aftslgt(DRWSELN))/3 + ...
        (aftslgt(DRWSELN)^2)/(forelgt(DRWSELN) + ...
        2*fuseslt + aftslgt(DRWSELN))/3 )/fuselgt(DRWSELN);% x-axis

    % adjust the isolated fuselage cg due to propulsion installation
    for i = 1:PN

        if engeloc(i, DRWSELN) < 3

            % on-wing nacelle (2) on-wing integrated with undercarraige
            FUSEBAX(DRWSELN) = FUSEBAX(DRWSELN) + 0.03;% based on Torenbeek

        elseif engeloc(i, DRWSELN) > 3

            % aft fuselage mounted striaght and S-ducts
            FUSEBAX(DRWSELN) = FUSEBAX(DRWSELN) + 0.10;% based on Torenbeek

        else

            % aft fuselage mounted on pylons
            FUSEBAX(DRWSELN) = FUSEBAX(DRWSELN) + 0.08;% based on Torenbeek

        end

    end

%%%%SR end

%%%%SR if FUSEBAZ(DRWSELN) < 0.0001

    FUSEBAZ(DRWSELN) = -0.1; % z-axis

%%%%SR end

% Output
PLOTCGS(5, 1, DRWSELN) = FUSEBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(5, 3, DRWSELN) = FUSEBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% Main landing gear: if not specified place the main landing gear CoG on
% the wing one
%

if ~LNDGBAX(DRWSELN)
    PLOTCGS(6, 1, DRWSELN) = PLOTCGS(1, 1, DRWSELN); % x-axis
    LNDGBAX(DRWSELN) = PLOTCGS(6, 1, DRWSELN)/fuselgt(DRWSELN);
else
    PLOTCGS(6, 1, DRWSELN) = LNDGBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
end
if ~LNDGBAZ(DRWSELN)
    PLOTCGS(6, 3, DRWSELN) = PLOTCGS(1, 3, DRWSELN); % z-axis
    LNDGBAZ(DRWSELN) = PLOTCGS(6, 3, DRWSELN)/fusevma(DRWSELN);
else
    PLOTCGS(6, 3, DRWSELN) = LNDGBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis
end


% powerplant longitudinal and vertical cg (Torenbeek, SAWE and geom. handbook)
for i = 1:PN

%%%%SR    if POWPBAX(i, DRWSELN) < 0.0001

        POWPBAX(i, DRWSELN) = ( NACENOX(i, DRWSELN) + ...
            0.4*NACELGT(i, DRWSELN) )/fuselgt(DRWSELN);% x-axis

%%%%SR    end

%%%%SR    if POWPBAY(i, DRWSELN) <= 0.

        % Improve this part!
        POWPBAY(i, DRWSELN) = NACENOY(i, DRWSELN); % y-axis

%%%%SR    end

%%%%SR    if POWPBAZ(i, DRWSELN) < 0.0001

        POWPBAZ(i, DRWSELN) = NACENOZ(i, DRWSELN)/fusevma(DRWSELN);% z-axis

%%%%SR    end

end

% Output
PLOTCGS(7, 1, DRWSELN) = POWPBAX(1,DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(7, 2, DRWSELN) = POWPBAY(1,DRWSELN);% y-axis
PLOTCGS(7, 3, DRWSELN) = POWPBAZ(1,DRWSELN)*fusevma(DRWSELN);% z-axis
PLOTCGS(8, 1, DRWSELN) = POWPBAX(2,DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(8, 2, DRWSELN) = POWPBAY(2,DRWSELN);% y-axis
PLOTCGS(8, 3, DRWSELN) = POWPBAZ(2,DRWSELN)*fusevma(DRWSELN);% z-axis


%==========================================================================
% Commented in the code
%
%FSYSBAX
%FCTLBAX
%APUSBAX
%INSTBAX
%AVIOBAX
%HYPNBAX
%ELECBAX
%ECSGBAX
%==========================================================================

% systems longitudinal and vertical cg (SAWE - assumed corresponding to fuse)

%%%%SR if FURNBAX(DRWSELN) < 0.0001

    FURNBAX(DRWSELN) = FUSEBAX(DRWSELN);% x-axis

%%%%SR end

%%%%SR if FURNBAZ(DRWSELN) < 0.0001

    FURNBAZ(DRWSELN) = FUSEBAZ(DRWSELN);% z-axis

%%%%SR end

% Output
PLOTCGS(17, 1, DRWSELN) = FURNBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(17, 3, DRWSELN) = FURNBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% wing integral tanks horizontal and vertical cg

%%%%SR if FUELBAX(DRWSELN) < 0.0001

    FUELBAX(DRWSELN) = xcgwing; % x-axis

%%%%SR end

%%%%SR if FUELBAY(DRWSELN) < 0.

    FUELBAY(DRWSELN) = ycgwing; % y-axis

%%%%SR end

%%%%SR if FUELBAZ(DRWSELN) < 0.0001

    FUELBAZ(DRWSELN) = zcgwing; % z-axis

%%%%SR end

% Output
PLOTCGS(18, 1, DRWSELN) = FUELBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(18, 2, DRWSELN) = FUELBAY(DRWSELN)*fuselgt(DRWSELN);% y-axis
PLOTCGS(18, 3, DRWSELN) = FUELBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% centre fuel tank(s) horizontal and vertical cg (geometric handbook)

%%%%SR if FCENBAX(DRWSELN) < 0.0001

    FCENBAX(DRWSELN) = xcgfair;% x-axis

%%%%SR end

%%%%SR if FCENBAZ(DRWSELN) < 0.0001

    FCENBAZ(DRWSELN) = zcgfair;% z-axis

%%%%SR end

% Output
PLOTCGS(19, 1, DRWSELN) = FCENBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(19, 3, DRWSELN) = FCENBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% auxiliary tank horizontal and vertical cg (geometric handbook)

%%%%SR if FAUXBAX(DRWSELN) < 0.0001

    FAUXBAX(DRWSELN) = xcgtaux;

%%%%SR end

%%%%SR if FAUXBAZ(DRWSELN) < 0.0001

    FAUXBAZ(DRWSELN) = zcgtaux;

%%%%SR end

% Output
PLOTCGS(20, 1, DRWSELN) = FAUXBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(20, 3, DRWSELN) = FAUXBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% interiors horizontal and vertical cg (geometric handbook)
%%%%SR if INTEBAX(DRWSELN) < 0.0001

    INTEBAX(DRWSELN) = ( forelgt(DRWSELN) + ...
        cabnlgt(DRWSELN)/2 )/fuselgt(DRWSELN);% x axis

%%%%SR end

%%%%SR if INTEBAZ(DRWSELN) < 0.0001

    INTEBAZ(DRWSELN) = ( cabnhei(DRWSELN)*0.4-0.5*((cabnwid(DRWSELN)^2) - ...
        cabnfwd(DRWSELN)^2)^0.5 )/fusevma(DRWSELN);% z axis

%%%%SR end

% Output
PLOTCGS(21, 1, DRWSELN) = INTEBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(21, 3, DRWSELN) = INTEBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% pilots horizontal and vertical cg (airframer data)

%%%%SR if PLOTBAX(DRWSELN) < 0.0001

    PLOTBAX(DRWSELN) = (forelgt(DRWSELN)-0.6)/fuselgt(DRWSELN);

%%%%SR end

%%%%SR if PLOTBAZ(DRWSELN) < 0.0001

    PLOTBAZ(DRWSELN) = ( WCHDROT(DRWSELN)*wingthk(1, DRWSELN)/2 + ...
        0.65 )/fusevma(DRWSELN) - 0.5;

%%%%SR end

% Output
PLOTCGS(22, 1, DRWSELN) = PLOTBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(22, 3, DRWSELN) = PLOTBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis

% Auxiliary landing gear: if not specified place the auxiliary landing gear
% CoG on the pilots' one
%
if ~ALNDGBAX(DRWSELN)
    PLOTCGS(9, 1, DRWSELN) = ALNDGBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
    ALNDGBAX(DRWSELN) = PLOTCGS(9, 1, DRWSELN)/fuselgt(DRWSELN);% x-axis
else
    PLOTCGS(9, 1, DRWSELN) = PLOTCGS(22, 1, DRWSELN);
end
if ~ALNDGBAZ(DRWSELN)
    PLOTCGS(9, 3, DRWSELN) = ALNDGBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis
    ALNDGBAZ(DRWSELN) = PLOTCGS(9, 3, DRWSELN)/fusevma(DRWSELN);
else
    PLOTCGS(9, 3, DRWSELN) = PLOTCGS(22, 3, DRWSELN);
end

% Commented in the code, line missing...
%FATTBAX(DRWSELN)=

% passengers/payload horizontal and vertical centre of gravity (airframer data)

%%%%SR if PASSBAX(DRWSELN) < 0.0001

    PASSBAX(DRWSELN) = ( forelgt(DRWSELN) + ...
        cabnlgt(DRWSELN)/2 )/fuselgt(DRWSELN);% x axis

%%%%SR end

%%%%SR if PASSBAZ(DRWSELN) < 0.0001

    PASSBAZ(DRWSELN) = PLOTBAZ(DRWSELN);% z axis

%%%%SR end

% Output
PLOTCGS(24, 1, DRWSELN) = PASSBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(24, 3, DRWSELN) = PASSBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis


% Crew horizontal and vertical centre of gravity (airframer data)
PLOTCGS(23, 1, DRWSELN) = PASSBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(23, 3, DRWSELN) = PASSBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis

% baggage horizontal and vertical centre of gravity (geometric handbook)

%%%%SR if BAGGBAX(DRWSELN) < 0.0001

    BAGGBAX(DRWSELN) = ( baggapx(DRWSELN)*fuselgt(DRWSELN) + ...
        bagglgt(DRWSELN)/2 )/fuselgt(DRWSELN);% x-axis

%%%%SR end

%%%%SR if BAGGBAZ(DRWSELN) < 0.0001

    BAGGBAZ(DRWSELN) = -( 0.6*cabnhei(DRWSELN) + 0.06 - ...
        fusevma(DRWSELN)/2 )/fusevma(DRWSELN);% z-axis

%%%%SR end

% % Output
PLOTCGS(25, 1, DRWSELN) = BAGGBAX(DRWSELN)*fuselgt(DRWSELN);% x-axis
PLOTCGS(25, 3, DRWSELN) = BAGGBAZ(DRWSELN)*fusevma(DRWSELN);% z-axis
