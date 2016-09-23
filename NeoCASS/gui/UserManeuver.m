%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2012
% 
% Sergio Ricci (sergio.ricci@polimi.it)
% Alessandro De Gaspari (degaspari@aero.polimi.it)
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
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090312      1.3     A. De Gaspari    Creation
%     090824      1.3.7   A. De Gaspari    Added maneuver label
%     090902      1.3.7   A. De Gaspari    Added maneuvers with canard
%     100216      1.4     A. De Gaspari    Force user to insert a positive
%                                          mach number
%     120224      2.0     A. De Gaspari    Unified typical and custom maneuvers,
%                                          new look, popup menu for typical
%                                          maneuvers...
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function state_cell = TypicalManeuver(nstates, labels)
%
function state_cell = UserManeuver(nstates, Tlabels)
%
labels = Tlabels(:,1);
nlabel = length(labels);
init = repmat({'0'},1,nlabel);
state_cell = cell(0, nlabel);

Tlab = TRIMlabels;
CSlabels  = Tlab(17:22,1);
CSflags = ismember(CSlabels, labels);

j = 1;
for i = 1:nstates,
    idmax = max(str2double(state_cell(:,1)));
    if idmax >= i,
        tcell = UserManeuverGUI(init, idmax+1, CSflags);
    elseif j > i,
        tcell = UserManeuverGUI(init, j, CSflags);
    else
        tcell = UserManeuverGUI(init, i, CSflags);
    end
    
    if ~isempty(tcell),
        state_cell(j,:) = tcell;
        j = j + 1;
    end
end

end
%
%
%-------------------------------------------------------------------------------
% Typical Maneuvers GUI Card
%-------------------------------------------------------------------------------
%
function state_1D_cell = UserManeuverGUI(array, ID, CS)
%
CSarray = cell(1,6);
state_1D_cell = {};

if CS(1), flap1 = 'On'; else flap1 = 'Off'; end
if CS(2), flap2 = 'On'; else flap2 = 'Off'; end
if CS(3), aileron = 'On'; else aileron = 'Off'; end
if CS(4), elevator = 'On'; else elevator = 'Off'; end
if CS(5), rudder = 'On'; else rudder = 'Off'; end
if CS(6), canard = 'On'; else canard = 'Off'; end

%----------
fr = 0.78; % Rapporto lati finestra
mywscr = 520;
myhscr = 600;
%dum = get(0,'ScreenSize');
%wscr = dum(3);
%hscr = dum(4);
hf = 650;
wf = hf/fr;
%
hrel = hf/myhscr;
wrel = wf/mywscr;
%
%
marg = 10*wrel;
%
wpb = 85;
hpb = 28;
%
h3p = 0.1;
h1pf = 0.83;
%-----------
%
ppb1 = (wf-marg)/2-wpb;
ppb2 = (wf+marg)/2;
%
wmargp = marg/wf*wrel;
hmargp = marg/hf*hrel;
%
pwp = wmargp;
wp = 1-2*wmargp;
w2p = (1-3*wmargp)/2;
pw2p = 2*wmargp+w2p;
h1p = h1pf*(1-(h3p+hpb/hf+5*hmargp));
h2p = h1p*(1-h1pf)/h1pf;
ph1p = 2*hmargp+hpb/hf;
ph2p = ph1p+h1p+hmargp;
ph3p = ph2p+h2p+hmargp;
%
wIDp = .25*(wp-wmargp);
w3p = .75*(wp-wmargp);
pw3p = 2*wmargp+wIDp;
%
%
hfig = figure('MenuBar','none','Name','Maneuver Definition','NumberTitle','off','Position',[80*wrel,90*hrel,wf,hf], 'IntegerHandle', 'off', 'Resize', 'off', 'color', [.8 .8 .8]);

savebut = uicontrol(hfig, 'Style', 'PushButton', 'String', 'Save', 'Position', [ppb1,marg,wpb,hpb], 'CallBack', @save_Callback);
uicontrol(hfig, 'Style', 'PushButton', 'String', 'Discard', 'Position', [ppb2,marg,wpb,hpb], 'CallBack', @discard_Callback);

pan1 = uipanel(hfig, 'Title', 'Parameters', 'Background', [.8 .8 .8], 'ForegroundColor', [0 0.3 0.55], 'FontWeight', 'bold', 'Position', [pwp,ph1p,wp,h1p]);
pan2 = uipanel(hfig, 'Title', 'Symmetric Maneuvers', 'Background', [.8 .8 .8], 'ForegroundColor', [0 0.3 0.55], 'FontWeight', 'bold', 'Position', [pwp,ph2p,w2p,h2p]);
pan3 = uipanel(hfig, 'Title', 'Anti-Symmetric Maneuvers', 'Background', [.8 .8 .8], 'ForegroundColor', [0 0.3 0.55], 'FontWeight', 'bold', 'Position', [pw2p,ph2p,w2p,h2p]);
pan4 = uipanel(hfig, 'Background', [.8 .8 .8], 'Position', [pwp,ph3p,wIDp,h3p]);
pan5 = uipanel(hfig, 'Background', [.8 .8 .8], 'Position', [pw3p,ph3p,w3p,h3p]);

%---
fID = 2;
%---
wID = wf*wIDp/fID;
hID = hf*h3p/fID;
pwID = (wIDp*wf-wID)/2;
phID = (h3p*hf-hID)/2;

edit_ID = uicontrol('Parent', pan4, 'Style', 'Edit', 'FontSize', 12, 'ForegroundColor', 'White', 'BackgroundColor', [.6 .6 .6], 'String', num2str(ID), 'Position', [pwID,phID,wID,hID], 'Callback', @editID_Callback);

%---
mhmarg = 40*hrel;
wtmach = 45*hrel;
we = 50*wrel;
wth = 85*wrel;
he = 20*hrel;
ht = 20*hrel;
%---
phe = (h3p*hf-he)/2;
pht = (h3p*hf-ht)/2;
ptmach = mhmarg;
pemach = ptmach+wtmach;
pth = pemach+we+mhmarg;
peh = pth+wth;
%
text_mach = uicontrol('Parent', pan5, 'Style', 'Text', 'FontSize', 8, 'Background', [.8 .8 .8], 'String', 'Mach:', 'Position', [ptmach,pht,wtmach,ht]);
edit_mach = uicontrol('Parent', pan5, 'Style', 'Edit', 'ForegroundColor', [0 0.3 0.55], 'Background', 'White', 'String', num2str(0), 'Position', [pemach,phe,we,he], 'Callback', @edit_mach_Callback);

text_h = uicontrol('Parent', pan5, 'Style', 'Text', 'FontSize', 8, 'Background', [.8 .8 .8], 'String', 'Altitude [m]:', 'Position',[pth,pht,wth,ht]);
edit_h = uicontrol('Parent', pan5, 'Style', 'Edit', 'ForegroundColor', [0 0.3 0.55], 'Background', 'White', 'String', num2str(0), 'Position', [peh,phe,we,he], 'Callback', @edit_h_Callback);
%
%
%---------
txt1_hmarg = 35*hrel;
txt1_wmarg = 35*wrel;
edt1_hmarg = 34*hrel;
w1txt  = 125*wrel;
w2txt  = 145*wrel;
htxt  = 20*hrel;
hdist = 10*hrel;
wtedist = 85*wrel;
%---------
wedt = we;
hedt = he;
hpan = hf*h1p;
%
pw1txt = txt1_wmarg;
pw1edt = pw1txt+w1txt;
pw2txt = pw1edt+wtedist;
pw2edt = pw2txt+w2txt;
%
ph1txt = hpan-txt1_hmarg-htxt;
ph2txt = ph1txt-hdist-htxt;
ph3txt = ph2txt-hdist-htxt;
ph4txt = ph3txt-hdist-htxt;
ph5txt = ph4txt-hdist-htxt;
ph6txt = ph5txt-hdist-htxt;
ph7txt = ph6txt-hdist-htxt;
ph8txt = ph7txt-hdist-htxt;
ph9txt = ph8txt-hdist-htxt;
ph10txt = ph9txt-hdist-htxt;
ph11txt = ph10txt-hdist-htxt;

ph1edt = hpan-edt1_hmarg-hedt;
ph2edt = ph1edt-hdist-hedt;
ph3edt = ph2edt-hdist-hedt;
ph4edt = ph3edt-hdist-hedt;
ph5edt = ph4edt-hdist-hedt;
ph6edt = ph5edt-hdist-hedt;
ph7edt = ph6edt-hdist-hedt;
ph8edt = ph7edt-hdist-hedt;
ph9edt = ph8edt-hdist-hedt;
ph10edt = ph9edt-hdist-hedt;
ph11edt = ph10edt-hdist-hedt;
%
lab = TRIMlabels;
lab = lab(:, 1);

text_anglea = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph1txt,w1txt,htxt], 'Enable', 'On', 'String', ['Angle of attack (', lab{6}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_anglea = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [pw1edt,ph1edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_anglea_Callback);

text_sides  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph1txt,w2txt,htxt], 'Enable', 'On', 'String', ['Sideslip angle (', lab{7}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_sides  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [pw2edt,ph1edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_sides_Callback);

text_rollr = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph2txt,w1txt,htxt], 'Enable', 'On', 'String', ['Roll rate (', lab{8}, ') [1/s]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_rollr = uicontrol('Parent', pan1, 'Style', 'Edit', 'Position', [pw1edt,ph2edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_rollr_Callback);

text_pitchr = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph3txt,w1txt,htxt], 'Enable', 'On', 'String', ['Pitch rate (', lab{9}, ') [1/s]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_pitchr = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [pw1edt,ph3edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_pitchr_Callback);

text_yawr = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph4txt,w1txt,htxt], 'Enable', 'On', 'String', ['Yaw rate (', lab{10}, ') [1/s]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_yawr = uicontrol('Parent', pan1, 'Style', 'Edit', 'Position', [pw1edt,ph4edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_yawr_Callback);

text_xacc = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph5txt,w2txt,htxt], 'Enable', 'On', 'String', ['X acc (', lab{11}, ') [m/s^2]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_xacc = uicontrol('Parent', pan1, 'Style', 'Edit', 'Position', [pw2edt,ph5edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_xacc_Callback);

text_yacc = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph6txt,w2txt,htxt], 'Enable', 'On', 'String', ['Y acc (', lab{12}, ') [m/s^2]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_yacc = uicontrol('Parent', pan1, 'Style', 'Edit', 'Position', [pw2edt,ph6edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_yacc_Callback);

text_zacc = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph7txt,w2txt,htxt], 'Enable', 'On', 'String', ['Z acc (', lab{13}, ') [m/s^2]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_zacc = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(9.81), 'Position', [pw2edt,ph7edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_zacc_Callback);

text_prate = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph2txt,w2txt,htxt], 'Enable', 'On', 'String', ['p rate (', lab{14}, ') [1/s^2]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_prate = uicontrol('Parent', pan1, 'Style', 'Edit', 'Position', [pw2edt,ph2edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_prate_Callback);

text_qrate = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph3txt,w2txt,htxt], 'Enable', 'On', 'String', ['q rate (', lab{15}, ') [1/s^2]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_qrate = uicontrol('Parent', pan1, 'Style', 'Edit', 'Position', [pw2edt,ph3edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_qrate_Callback);

text_rrate = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph4txt,w2txt,htxt], 'Enable', 'On', 'String', ['r rate (', lab{16}, ') [1/s^2]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_rrate = uicontrol('Parent', pan1, 'Style', 'Edit', 'Position', [pw2edt,ph4edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_rrate_Callback);

text_flap1  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph9txt,w1txt,htxt], 'Enable', 'On', 'String', ['1st Flap rotation (', lab{17}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_flap1  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Enable', flap1, 'Position', [pw1edt,ph9edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_flap1_Callback);

text_flap2  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph10txt,w1txt,htxt], 'Enable', 'On', 'String', ['2nd Flap rotation (', lab{18}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_flap2  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Enable', flap2, 'Position', [pw1edt,ph10edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_flap2_Callback);

text_ailer  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph7txt,w1txt,htxt], 'Enable', 'On', 'String', ['Aileron rotation (', lab{19}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_ailer  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Enable', aileron, 'Position', [pw1edt,ph7edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_ailer_Callback);

text_elev   = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph5txt,w1txt,htxt], 'Enable', 'On', 'String', ['Elevator rotation (', lab{20}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_elev   = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Enable', elevator, 'Position', [pw1edt,ph5edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_elev_Callback);

text_rud    = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph8txt,w1txt,htxt], 'Enable', 'On', 'String', ['Rudder rotation (', lab{21}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_rud    = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Enable', rudder, 'Position', [pw1edt,ph8edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_rud_Callback);

text_canard  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw1txt,ph6txt,w1txt,htxt], 'Enable', 'On', 'String', ['Canard rotation (', lab{22}, ') [deg]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_canard  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Enable', canard, 'Position', [pw1edt,ph6edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_canard_Callback);

text_vgust  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph8txt,w2txt,htxt], 'Enable', 'On', 'String', ['Vertical speed (', lab{23}, ') [EAS m/s]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_vgust  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [pw2edt,ph8edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_vgust_Callback);

text_lndeff = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph9txt,w2txt,htxt], 'Enable', 'On', 'String', ['Strut efficiency (', lab{26}, ') []:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_lndeff = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [pw2edt,ph9edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_lndeff_Callback);

text_vsink  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph10txt,w2txt,htxt], 'Enable', 'On', 'String', ['Sink speed (', lab{24}, ') [m/s]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_vsink  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [pw2edt,ph10edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_vsink_Callback);

text_stroke = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [pw2txt,ph11txt,w2txt,htxt], 'Enable', 'On', 'String', ['Shock absorber stroke (', lab{25}, ') [m]:'], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_stroke = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [pw2edt,ph11edt,wedt,hedt],  'Background', 'White', 'Callback', @edit_stroke_Callback);



if ~CS(1)
    set(text_flap1, 'ForegroundColor', [.4 .4 .4]);
end

if ~CS(2)
    set(text_flap2, 'ForegroundColor', [.4 .4 .4]);
end

if ~CS(3)
    set(text_ailer, 'ForegroundColor', [.4 .4 .4]);
end

if ~CS(4)
    set(text_elev, 'ForegroundColor', [.4 .4 .4]);
end

if ~CS(5)
    set(text_rud, 'ForegroundColor', [.4 .4 .4]);
end

if ~CS(6)
    set(text_canard, 'ForegroundColor', [.4 .4 .4]);
end





check_wp = 15;
check_w  = 200;
check_hp = 5;
check_h  = 20;

check_sym = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Enable', 'Off', 'Background', [.8 .8 .8], 'String', 'Symmetric maneuver', 'Position', [check_wp,check_hp,check_w,check_h], 'Callback', @check_sym_Callback);

check_custom = uicontrol(hfig, 'Style', 'Checkbox', 'Value', false, 'Background', [.8 .8 .8], 'String', 'User defined maneuver', 'Position', [pwp+1.4*marg+check_wp,1.2*marg,check_w,hpb], 'Callback', @check_custom_Callback);







% Initializing
symlist = {'Cruise/Climb  (AoA, pitch control surfaces)'; ...
           'Climb fixed AoA  (Z acc, pitch control surfaces)'; ...
           'Non standard  (pitch control surfaces)'; ...
           'Vertical gust  (AoA, pitch control surfaces)'; ...
           'Landing  (AoA, pitch control surfaces)'};
       
antilist = {'Sideslip levelled flight'; ...
            'Aileron abrupt input  (p rate)'; ...
            'Aileron steady roll response  (roll rate)'; ...
            'Steady roll pullout maneuver  (roll rate)'; ...
            'Snap roll  (accs)'};

%-------
hmarg = 30*hrel;
wmarg = 13*wrel;
radio_h = 20*hrel;
%-------
hman = h2p*hf;
wman = w2p*wf;
radio_w = wman-2*wmarg;
radio_wp = wmarg;
radio1_hp = hman-hmarg-radio_h;

if CS(4) || CS(6)
    if ~(CS(4) && CS(6))
        symlist(3) = [];
    end
    sym_pop = uicontrol( 'Parent', pan2, 'Style', 'PopupMenu', 'String', symlist, 'Position', [radio_wp,radio1_hp,radio_w,radio_h], 'Callback', @sym_pop_Callback);
else
    sym_pop = uicontrol( 'Parent', pan2, 'Style', 'Text', 'String', 'Control surfaces not available for these maneuvers', 'Position', [radio_wp,radio1_hp,radio_w,radio_h], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
end

if CS(3) || CS(5)
    antirm = false(1, length(antilist));
    if ~CS(3)
        antirm(5) = true;
    end
    
    if ~( CS(3) && CS(5) && (CS(4) || CS(6)) )
        antirm(1) = true;
    end
    
    if ~( (CS(4) || CS(6)) && CS(5) )
        antirm(2) = true;
        antirm(3) = true;
        antirm(4) = true;
    end
    antilist(antirm) = [];
    anti_pop = uicontrol('Parent', pan3, 'Style', 'PopupMenu', 'String', antilist, 'Position', [radio_wp,radio1_hp,radio_w,radio_h], 'Callback', @anti_pop_Callback);
else
    anti_pop = uicontrol( 'Parent', pan3, 'Style', 'Text', 'String', 'Control surfaces not available for these maneuvers', 'Position', [radio_wp,radio1_hp,radio_w,radio_h], 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
end


if CS(4) || CS(6)
    set(sym_pop, 'Value', 1);
    sym_pop_Callback;
    set(check_sym, 'Value', true);
elseif CS(3) && CS(5) && (CS(4) || CS(6))
    set(anti_pop, 'Value', 1);
    anti_pop_Callback;
    set(check_sym, 'Value', false);
elseif (CS(4) || CS(6)) && CS(5)
    set(anti_pop, 'Value', 1);
    anti_pop_Callback;
    set(check_sym, 'Value', false);
elseif CS(3)
    set(anti_pop, 'Value', 1);
    anti_pop_Callback;
    set(check_sym, 'Value', false);
else
    set(sym_pop, 'Background', [.8 .8 .8]);
    set(sym_pop, 'ForegroundColor', [.4 .4 .4]);
    set(anti_pop, 'Background', [.8 .8 .8]);
    set(anti_pop, 'ForegroundColor', [.4 .4 .4]);
    set(edit_anglea, 'Enable', 'Off');
    set(edit_sides, 'Enable', 'Off');
    set(edit_rollr, 'Enable', 'Off');
    set(edit_pitchr, 'Enable', 'Off');
    set(edit_yawr, 'Enable', 'Off');
    set(edit_xacc, 'Enable', 'Off');
    set(edit_yacc, 'Enable', 'Off');
    set(edit_zacc, 'Enable', 'Off');
    set(edit_prate, 'Enable', 'Off');
    set(edit_qrate, 'Enable', 'Off');
    set(edit_rrate, 'Enable', 'Off');
    set(edit_vgust, 'Enable', 'Off');
    set(edit_lndeff, 'Enable', 'Off');
    set(edit_vsink, 'Enable', 'Off');
    set(edit_stroke, 'Enable', 'Off');
    set(check_sym, 'Enable', 'Off');
    set(check_custom, 'Enable', 'Off');
    set(savebut, 'Enable', 'Off');
end
% End initializing







uiwait(hfig);


    function sym_pop_Callback(~, ~)
        
        set(sym_pop, 'Background', [1. 1. 1.]);
        set(sym_pop, 'ForegroundColor', [0. 0. 0.]);
        set(anti_pop, 'Background', [.8 .8 .8]);
        set(anti_pop, 'ForegroundColor', [.4 .4 .4]);
        
        set(check_sym, 'Value', true);
        
        cl = get(sym_pop, 'String');
        cm = cl{get(sym_pop, 'Value')};
        
        switch cm
            
            case 'Cruise/Climb  (AoA, pitch control surfaces)'
                
                set(edit_anglea, 'String', '', 'Enable', 'Off');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', num2str(0.0), 'Enable', 'Off');
                % set(edit_zacc, 'Enable', 'On');
                set(edit_zacc, 'String', num2str(9.81), 'Enable', 'On');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_rud, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Cruise/Climb'; % Label
                
            case 'Climb fixed AoA  (Z acc, pitch control surfaces)'
                
                set(edit_anglea, 'String', num2str(0.0), 'Enable', 'On');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_zacc, 'String', '', 'Enable', 'Off');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_rud, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Climb at fixed AoA'; % Label
                
            case 'Non standard  (pitch control surfaces)'
                
                set(edit_anglea, 'String', num2str(0.0), 'Enable', 'On');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_zacc, 'String', num2str(9.81), 'Enable', 'On');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_rud, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Non standard W canard'; % Label
                
            case 'Vertical gust  (AoA, pitch control surfaces)'
                
                set(edit_anglea, 'String', '', 'Enable', 'Off');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_zacc, 'String', num2str(9.81), 'Enable', 'Off');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'On');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_rud, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Vertical gust'; % Label
                
            case 'Landing  (AoA, pitch control surfaces)'
                
                set(edit_anglea, 'String', '', 'Enable', 'Off');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_zacc, 'String', num2str(2/3*9.81), 'Enable', 'On');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.8), 'Enable', 'On');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'On');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'On');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_rud, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Landing'; % Label
                
        end
        
    end


    function anti_pop_Callback(~, ~)
        
        set(anti_pop, 'Background', [1. 1. 1.]);
        set(anti_pop, 'ForegroundColor', [0. 0. 0.]);
        set(sym_pop, 'Background', [.8 .8 .8]);
        set(sym_pop, 'ForegroundColor', [.4 .4 .4]);
        
        set(check_sym, 'Value', false);
        
        cl = get(anti_pop, 'String');
        cm = cl{get(anti_pop, 'Value')};
        
        switch cm
            
            case 'Sideslip levelled flight'
                
                set(edit_anglea, 'String', '', 'Enable', 'Off');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'On');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', '', 'Enable', 'Off');
                set(edit_zacc, 'String', num2str(9.81), 'Enable', 'Off');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', '', 'Enable', 'Off');
                set(edit_rud, 'String', '', 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Sideslip levelled flight'; % Label
                
            case 'Aileron abrupt input  (p rate)'
                
                set(edit_anglea, 'String', '', 'Enable', 'Off');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', '', 'Enable', 'Off');
                set(edit_zacc, 'String', num2str(9.81), 'Enable', 'Off');
                set(edit_prate, 'String', '', 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', aileron);
                set(edit_rud, 'String', '', 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Aileron abrupt input'; % Label
                
            case 'Aileron steady roll response  (roll rate)'
                
                set(edit_anglea, 'String', '', 'Enable', 'Off');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', '', 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', '', 'Enable', 'Off');
                set(edit_zacc, 'String', num2str(9.81), 'Enable', 'Off');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', aileron);
                set(edit_rud, 'String', '', 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Aileron steady roll response'; % Label
                
            case 'Steady roll pullout maneuver  (roll rate)'
                
                set(edit_anglea, 'String', '', 'Enable', 'Off');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', '', 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'On');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', '', 'Enable', 'Off');
                set(edit_zacc, 'String', num2str(9.81), 'Enable', 'Off');
                set(edit_prate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_qrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rrate, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', aileron);
                set(edit_rud, 'String', '', 'Enable', 'Off');
                set(edit_elev, 'String', '', 'Enable', elevator);
                set(edit_canard, 'String', '', 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Steady roll pullout maneuver'; % Label
                
            case 'Snap roll  (accs)'
                
                set(edit_anglea, 'String', num2str(0.0), 'Enable', 'On');
                set(edit_sides, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_rollr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_pitchr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_yawr, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_xacc, 'String', '', 'Enable', 'Off');
                set(edit_yacc, 'String', '', 'Enable', 'Off');
                set(edit_zacc, 'String', '', 'Enable', 'Off');
                set(edit_prate, 'String', '', 'Enable', 'Off');
                set(edit_qrate, 'String', '', 'Enable', 'Off');
                set(edit_rrate, 'String', '', 'Enable', 'Off');
                set(edit_vgust, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_lndeff, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_vsink, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_stroke, 'String', num2str(0.0), 'Enable', 'Off');
                
                set(edit_ailer, 'String', num2str(0.0), 'Enable', 'Off');
                set(edit_elev, 'String', num2str(0.0), 'Enable', elevator);
                set(edit_rud, 'String', num2str(0.0), 'Enable', rudder);
                set(edit_canard, 'String', num2str(0.0), 'Enable', canard);
                set(edit_flap1, 'String', num2str(0.0), 'Enable', flap1);
                set(edit_flap2, 'String', num2str(0.0), 'Enable', flap2);
                
                array{2}  = 'Snap roll'; % Label
                
        end
        
    end


    function editID_Callback(~, ~)
        
    end

    function edit_mach_Callback(~, ~)
        
    end

    function edit_h_Callback(~, ~)
        
    end


    function edit_anglea_Callback(~, ~)

    end

    function edit_sides_Callback(~, ~)

    end

    function edit_rollr_Callback(~, ~)

    end

    function edit_pitchr_Callback(~, ~)

    end

    function edit_yawr_Callback(~, ~)

    end

    function edit_xacc_Callback(~, ~)

    end

    function edit_yacc_Callback(~, ~)

    end

    function edit_zacc_Callback(~, ~)

    end

    function edit_prate_Callback(~, ~)

    end

    function edit_qrate_Callback(~, ~)

    end

    function edit_rrate_Callback(~, ~)

    end

    function edit_flap1_Callback(~, ~)

    end

    function edit_flap2_Callback(~, ~)

    end

    function edit_ailer_Callback(~, ~)

    end

    function edit_elev_Callback(~, ~)

    end

    function edit_rud_Callback(~, ~)

    end

    function edit_canard_Callback(~, ~)

    end

    function edit_vgust_Callback(~, ~)

    end

    function edit_lndeff_Callback(~, ~)

    end

    function edit_vsink_Callback(~, ~)

    end

    function edit_stroke_Callback(~, ~)

    end


    function check_sym_Callback(~, ~)
        
    end

    function check_custom_Callback(hObject, ~)
        
        array{2}  = 'Custom'; % Label
        
        if get(hObject, 'Value')
            set(sym_pop, 'Enable', 'Off');
            set(anti_pop, 'Enable', 'Off');
            set(edit_anglea, 'Enable', 'On');
            set(edit_sides, 'Enable', 'On');
            set(edit_rollr, 'Enable', 'On');
            set(edit_pitchr, 'Enable', 'On');
            set(edit_yawr, 'Enable', 'On');
            set(edit_xacc, 'Enable', 'On');
            set(edit_yacc, 'Enable', 'On');
            set(edit_zacc, 'Enable', 'On');
            set(edit_prate, 'Enable', 'On');
            set(edit_qrate, 'Enable', 'On');
            set(edit_rrate, 'Enable', 'On');
            set(edit_flap1, 'Enable', flap1);
            set(edit_flap2, 'Enable', flap2);
            set(edit_ailer, 'Enable', aileron);
            set(edit_elev, 'Enable', elevator);
            set(edit_rud, 'Enable', rudder);
            set(edit_canard, 'Enable', canard);
            set(edit_vgust, 'Enable', 'On');
            set(edit_lndeff, 'Enable', 'On');
            set(edit_vsink, 'Enable', 'On');
            set(edit_stroke, 'Enable', 'On');
            set(check_sym, 'Enable', 'On');
        else
            set(sym_pop, 'Enable', 'On');
            set(anti_pop, 'Enable', 'On');
            set(check_sym, 'Enable', 'Off');
            if all(get(sym_pop, 'Background') == [1. 1. 1.]) % ok...anche se non ci sono manovre simmetriche sym_pop è testo con sfondo grigio e questo if salta
                sym_pop_Callback;
            else % ok...se non ci sono nè manovre simmetriche nè antisimmetriche qui non si può arrivare perchè il flag custom è disattivo
                anti_pop_Callback;
            end
        end
        
    end


    function save_Callback(~, ~)
        
        state_1D_cell = {};
        
        mnstr = get(edit_mach, 'String');
        
        if str2double(mnstr) > 0
            
            array{1}  = get(edit_ID, 'String');
            array{3}  = num2str(double(get(check_sym, 'Value'))); % sym
            array{4}  = mnstr;
            array{5}  = get(edit_h, 'String');
            array{6}  = get(edit_anglea, 'String');
            array{7}  = get(edit_sides, 'String');
            array{8}  = get(edit_rollr, 'String');
            array{9}  = get(edit_pitchr, 'String');
            array{10} = get(edit_yawr, 'String');
            array{11} = get(edit_xacc, 'String');
            array{12} = get(edit_yacc, 'String');
            array{13} = get(edit_zacc, 'String');
            array{14} = get(edit_prate, 'String');
            array{15} = get(edit_qrate, 'String');
            array{16} = get(edit_rrate, 'String');
            
            CSarray{1} = get(edit_flap1, 'String');
            CSarray{2} = get(edit_flap2, 'String');
            CSarray{3} = get(edit_ailer, 'String');
            CSarray{4} = get(edit_elev, 'String');
            CSarray{5} = get(edit_rud, 'String');
            CSarray{6} = get(edit_canard, 'String');
            
            array(17:16+sum(CS)) = CSarray(CS); % se si arriva qui almeno il campo 17 è una superficie
            
            cl = get(sym_pop, 'String');
            cm = cl{get(sym_pop, 'Value')};
            
            % Vertical gust
            if ~isempty(strfind(lower(cm), 'vertical gust')) && str2double(get(edit_vgust, 'String')) == 0
                warndlg('When vertical gust is selected, VGUST paramenter must be not zero.', 'NeoCASS - Maneuver definition');
                return
            else
                array{16+sum(CS)+1} = get(edit_vgust, 'String');
            end
            
            % Landing
            if ~isempty(strfind(lower(cm), 'landing')) && ( str2double(get(edit_vsink, 'String')) == 0 || str2double(get(edit_stroke, 'String')) == 0 )
                warndlg('When landing is selected, VSINK and STROKE paramenters must be not zero.', 'NeoCASS - Maneuver definition');
                return
            else
                array{16+sum(CS)+2} = get(edit_vsink, 'String');
                array{16+sum(CS)+3} = get(edit_stroke, 'String');
                array{16+sum(CS)+4} = get(edit_lndeff, 'String');
            end
            
            state_1D_cell = array;
            close(hfig);
            
        else
            
            warndlg('Mach number must be positive.', 'NeoCASS - Maneuver definition');
            
        end
        
    end

    function discard_Callback(~, ~)
        state_1D_cell = {};
        close(hfig);
    end



end



