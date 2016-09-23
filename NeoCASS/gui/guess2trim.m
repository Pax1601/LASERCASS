%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
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
%     090312      1.3     A. De Gaspari    Creation
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function [filename_trim, jwflag, guessstd, MAN] = guess2trim(filename_aircraft)
%
function [filename_trim, jwflag, swflag, guessstd, MAN] = guess2trim(filename_aircraft)
%
MAN = [];
filename_trim = '';
jwflag = false;
swflag = false;
guessstd = false;
MAN = [];
%
%----------
wf = 460;
hf = 730;
%
marg = 8;
%
wpb = 85;
hpb = 28;
h1pf = 0.895;
%----------
%
ppb1 = (wf-marg)/2-3/2*wpb;
ppb3 = wf/2-1/2*wpb;
ppb2 = (wf+marg)/2+1/2*wpb;
%
wmargp = marg/wf;
hmargp = marg/hf;
%
pwp = wmargp;
wp = 1-2*wmargp;
h1p = h1pf*(1-(hpb/hf+5*hmargp));
h2p = h1p*(1-h1pf)/h1pf;
ph2p = 2*hmargp+hpb/hf;
ph1p = ph2p+h2p+hmargp;
%
%
hfig = figure('MenuBar', 'none', 'Name', 'GUESS sizing mode', 'NumberTitle', 'off', 'Position', [250,30,wf,hf], 'IntegerHandle', 'off', 'color', [.8 .8 .8]);

push_ok = uicontrol(hfig, 'Style', 'PushButton', 'Enable', 'On', 'Background', [.8 .8 .8], 'String', 'Ok', 'Position', [ppb1,marg,wpb,hpb], 'CallBack', @ok_Callback);
push_apply = uicontrol(hfig, 'Style', 'PushButton', 'Enable', 'On', 'Background', [.8 .8 .8], 'String', 'Apply', 'Position', [ppb3,marg,wpb,hpb], 'CallBack', @apply_Callback);
uicontrol(hfig, 'Style', 'PushButton', 'Background', [.8 .8 .8], 'String', 'Cancel', 'Position', [ppb2,marg,wpb,hpb], 'CallBack', @cancel_Callback);

pan1 = uipanel(hfig, 'Title', 'Guess/SMARTCAD trim interface', 'Background', [.8 .8 .8], 'FontWeight', 'bold', 'ForegroundColor', [0 0.3 0.55], 'Position', [pwp,ph1p,wp,h1p]);
pan2 = uipanel(hfig, 'Title', 'Solution Method', 'Background', [.8 .8 .8], 'FontWeight', 'bold', 'ForegroundColor', [0 0.3 0.55], 'Position', [pwp,ph2p,wp,h2p]);
%
%
aircraft = neocass_xmlwrapper(filename_aircraft);
if aircraft.Tailbooms.present || aircraft.Vertical_tail.Twin_tail
    checkrigid = 'Off';
else
    checkrigid = 'On';
end
%
%-------------
hmarg = 31;
wmarg = 15;
cmarg = 30;
farmarg = 338;
check_h = 20;
%-------------
h1man = h1p*hf;
h2man = h2p*hf;
wman = wp*wf;
check_w = wman-2*wmarg;
check_wp = wmarg;
pop_w = wman*0.6;
pop_wp = 2.5*wmarg;
check1_hp = h2man-hmarg/1.4-check_h;
check4_hp = h2man-hmarg/0.7-check_h;
check3_hp = h1man-cmarg-check_h;
check2_hp = check3_hp-hmarg-farmarg-check_h;
%
check_far = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', '', 'Position', [check_wp,check3_hp,check_w,check_h], 'Callback', @check_far_Callback);
pop_far = uicontrol('Parent', pan1, 'Style', 'Popup', 'String', {'EASA automatic selection'; 'EASA CS23'; 'EASA CS25'}, 'Background', [1. 1. 1.], 'Position', [pop_wp,check3_hp,pop_w,check_h], 'Callback', @pop_far_Callback);
check_user = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', false, 'Background', [.8 .8 .8], 'String', 'Maneuvers set definition', 'Position', [check_wp,check2_hp,check_w,check_h], 'Callback', @check_user_Callback);
check_JW = uicontrol('Parent', pan2, 'Style', 'Checkbox', 'Value', false, 'Enable', 'On', 'Background', [.8 .8 .8], 'String', 'Joined wing', 'Position', [check_wp+check_w*0.52,check1_hp,check_w/4,check_h], 'Callback', @check_JW_Callback);
pop_SW = uicontrol('Parent', pan2, 'Style', 'Popup', 'Value', 1, 'String', {'No Strut-braced wing'; 'Strut-braced wing (without aero)'; 'Strut-braced wing (with aero)'}, 'Background', [1. 1. 1.], 'Position', [check_wp+check_w*0.52,check4_hp*1.3,check_w/2,check_h], 'Callback', @pop_SW_Callback);
%
check_stand = uicontrol('Parent', pan2, 'Style', 'Checkbox', 'Enable', checkrigid, 'Value', false, 'Background', [.8 .8 .8], 'String', 'Rigid Aircraft', 'Position', [check_wp,check1_hp,check_w/2,check_h], 'Callback', @check_stand_Callback);
check_elast = uicontrol('Parent', pan2, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Elastic Aircraft', 'Position', [check_wp,check4_hp,check_w/2,check_h], 'Callback', @check_elast_Callback);
%
%------
chw147 = 95;
chw36 = 140;
chw = 160;
chh = 20;
chtopmarg = 18;
chrightmarg = 20;
chwmarg = -60;
chhmarg = 10;
%
check_pul = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Pullup', 'Position', [check_wp+chrightmarg,check3_hp-chtopmarg-chhmarg, chw147,chh], 'Callback', @check_pul_Callback);
check_hor = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Horizontal tail/canard', 'Position', [check_wp+chw147+chrightmarg,check3_hp-chtopmarg-chhmarg, chw,chh], 'Callback', @check_hor_Callback);
check_ver = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Vertical tail', 'Position', [check_wp+2*chw+chwmarg+chrightmarg,check3_hp-chtopmarg-chhmarg, chw36,chh], 'Callback', @check_ver_Callback);
check_ail = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Ailerons', 'Position', [check_wp+chrightmarg,check3_hp-chtopmarg-chh-chhmarg, chw147,chh], 'Callback', @check_ail_Callback);
check_gus = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Static Gust', 'Position', [check_wp+chw147+chrightmarg,check3_hp-chtopmarg-chh-chhmarg, chw,chh], 'Callback', @check_gus_Callback);
check_lan = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Taildown Landing', 'Position', [check_wp+2*chw+chwmarg+chrightmarg,check3_hp-chtopmarg-chh-chhmarg, chw36,chh], 'Callback', @check_lan_Callback);
check_eng = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'Engine Out', 'Position', [check_wp+chrightmarg,check3_hp-chtopmarg-2*chh-chhmarg, chw147,chh], 'Callback', @check_eng_Callback);
check_hl  = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', true, 'Background', [.8 .8 .8], 'String', 'High Lift', 'Position', [check_wp+chw147+chrightmarg,check3_hp-chtopmarg-2*chh-chhmarg, chw,chh], 'Callback', @check_hl_Callback);
%
hfarp = check3_hp-chtopmarg-3*chh-chhmarg;
ptopmarg = 11;
eptopmarg = 11*0.78;
edtwpos = 370;
edtw = 53;
edth = 21.8;
txtw = 320;
txth = 22;
%
text_hcru = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Cruise altitude (HCRU) [m]:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_hcru = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_hcru_Callback);

text_mcru  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Min cruise mach number (MCRU) []:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_mcru  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_mcru_Callback);

text_hmax  = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-2*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Max ceiling altitude (HMAX) [m]:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_hmax  = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-2*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_hmax_Callback);

text_clmax = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-3*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Clean max lift coefficient (CLMAX) []:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_clmax = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-3*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_clmax_Callback);

text_clmaxto = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-4*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'All flaps down max CL at Take Off (CLMAXTO) []:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_clmaxto = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-4*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_clmaxto_Callback);

text_clmaxl = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-5*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'All flaps down max CL at Landing (CLMAXLAND) []:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_clmaxl = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-5*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_clmaxl_Callback);

text_clalphad = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-6*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Clean lift curve slope (CLALPHAD) []:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_clalphad = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-6*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_clalphad_Callback);

text_sref = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-7*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Reference surface (USERSREF) [m^2]:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_sref = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-7*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_sref_Callback);

text_flapto = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-8*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Flap deflection for TO (FLAPTO) [deg]:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_flapto = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-8*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_flapto_Callback);

text_flapland = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-9*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Flap deflection for Landing (FLAPLAND) [deg]:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_flapland = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', '', 'Position', [edtwpos,hfarp-9*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_flapland_Callback);

text_vsink = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-10*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Sink speed at landing (VSINK) [m/s]:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_vsink = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [edtwpos,hfarp-10*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_vsink_Callback);

text_stroke = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-11*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Shock absorber stroke at landing (STROKE) [m]:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_stroke = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.0), 'Position', [edtwpos,hfarp-11*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_stroke_Callback);

text_lndeff = uicontrol('Parent', pan1, 'Style', 'Text', 'Position', [check_wp+chrightmarg,hfarp-12*txth-ptopmarg,txtw,txth], 'Enable', 'On', 'String', 'Landing gear efficiency (LNDGEFF) []:', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left');
edit_lndeff = uicontrol('Parent', pan1, 'Style', 'Edit', 'String', num2str(0.8), 'Position', [edtwpos,hfarp-12*edth-eptopmarg,edtw,edth],  'Background', 'White', 'Callback', @edit_lndeff_Callback);
%
%------
wtxt = 180;
wedit = 50;
wpush = 110;
htxt = 22;
hep = 28;
wtxtmarg = 20;
htxtmarg = 32;
epmarg = 13;
wedtmarg = 15;
%------
phteu = check2_hp-htxtmarg;
pwtxt = check_wp+wtxtmarg;
pwedit = pwtxt+wtxt+wedtmarg;
pwpush = pwedit+wedit+epmarg;
%
text_states = uicontrol('Parent', pan1, 'Style', 'Text', 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left', 'String', 'Number of flight conditions:', 'Position', [pwtxt,phteu,wtxt,htxt]);
edit_states = uicontrol('Parent', pan1, 'Style', 'Edit', 'Background', 'white', 'String', num2str(0), 'Position', [pwedit,phteu,wedit,hep], 'Callback', @edit_states_Callback);
push_states = uicontrol('Parent', pan1, 'Style', 'PushButton', 'Enable', 'Off', 'Background', [.8 .8 .8], 'FontWeight', 'bold', 'ForegroundColor', 'red', 'String', 'SELECT Values', 'Position', [pwpush,phteu,wpush,hep], 'CallBack', @push_states_Callback);
%
%------
wftxt = 85;
wfedit = 260;
wfpush = 30;
hftxt = 18;
hfep = 22;
wftxtmarg = 20;
hftxtmarg = 72;
fepmarg = 13;
%------
phfteu = check2_hp-hftxtmarg;
pwftxt = check_wp+wftxtmarg;
pwfedit = pwftxt+wftxt;
pwfpush = pwfedit+wfedit+fepmarg;
%
text_file = uicontrol('Parent', pan1, 'Style', 'Text', 'Background', [.8 .8 .8], 'String', 'Export to file:', 'Position', [check_wp,phfteu,wftxt,hftxt]);
edit_file = uicontrol('Parent', pan1, 'Style', 'Edit', 'Enable', 'On', 'Background', 'white', 'Position', [pwfedit,phfteu,wfedit,hfep], 'HorizontalAlignment', 'left');
push_file = uicontrol('Parent', pan1, 'Style', 'PushButton', 'Enable', 'On', 'Background', [.8 .8 .8], 'FontWeight', 'bold', 'String', '...', 'Position', [pwfpush,phfteu,wfpush,hfep], 'CallBack', @push_file_Callback);
%
%-------------
clmarg = 92;
lhmarg = 10;
lw = 345;
%-------------
check4_hp = check2_hp-clmarg-check_h;
lhp = check4_hp-lhmarg-check_h;
%
check_load = uicontrol('Parent', pan1, 'Style', 'Checkbox', 'Value', false, 'Background', [.8 .8 .8], 'HorizontalAlignment', 'left', 'String', 'Load trim conditions from file', 'Position', [check_wp,check4_hp,check_w,check_h], 'Callback', @check_load_Callback);
edit_load = uicontrol('Parent', pan1, 'Style', 'Edit', 'Enable', 'inactive', 'Background', 'white', 'Position', [pwtxt,lhp,lw,hfep]);
push_load = uicontrol('Parent', pan1, 'Style', 'PushButton', 'Enable', 'Off', 'Background', [.8 .8 .8], 'FontWeight', 'bold', 'String', '...', 'Position', [pwfpush,lhp,wfpush,hfep], 'CallBack', @push_load_Callback);
%
%
filename = [];
set(edit_file, 'String', filename);
set(edit_load, 'String', filename);

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
labels = gui_param.solver.aeros.TRIM_LABELS;

nlabel = length(labels);
state_mat = cell(0, nlabel);

uiwait(hfig);


    function check_stand_Callback(hObject, eventdata)
        set(check_stand, 'Value', true);
        set(check_elast, 'Value', false);
        set(check_JW, 'Value', false);
        set(check_JW, 'Enable', 'Off');
        set(pop_SW, 'Value', 1);
        set(pop_SW, 'Enable', 'Off');
    end


    function check_elast_Callback(hObject, eventdata)
        set(check_elast, 'Value', true);
        set(check_stand, 'Value', false);
        set(check_JW, 'Enable', 'On');
        set(pop_SW, 'Enable', 'On');
    end


    function check_far_Callback(hObject, eventdata)
        faron;
        set(check_far, 'Value', true);
        set(check_user, 'Value', false);
        if ~get(check_stand, 'Value')
            set(check_JW, 'Enable', 'On');
        end
        if ~get(check_stand, 'Value')
            set(pop_SW, 'Enable', 'On');
        end
        set(check_load, 'Value', false);
        set(push_load, 'Enable', 'Off');
        filename = get(edit_file, 'String');
        set(push_states, 'Enable', 'Off');
        set(push_file, 'Enable', 'On');
        set(edit_file, 'Enable', 'On');
        set(push_ok, 'Enable', 'On');
    end


    function pop_far_Callback(hObject, eventdata)
        
    end


    function check_user_Callback(hObject, eventdata)
        faroff;
        set(check_far, 'Value', false);
        set(check_user, 'Value', true);
        if ~get(check_stand, 'Value')
            set(check_JW, 'Enable', 'On');
        end
        if ~get(check_stand, 'Value')
            set(pop_SW, 'Enable', 'On');
        end
        set(check_load, 'Value', false);
        set(push_load, 'Enable', 'Off');
        filename = get(edit_file, 'String');
        if str2double(get(edit_states, 'String')) > 0,
            set(push_states, 'Enable', 'On');
        else
            set(push_states, 'Enable', 'Off');
        end
        
        if isempty(state_mat),
            set(push_file, 'Enable', 'Off');
            set(edit_file, 'Enable', 'Off');
        else
            set(push_file, 'Enable', 'On');
            set(edit_file, 'Enable', 'On');
        end
        
        if isempty(get(edit_file, 'String')),
            set(push_ok, 'Enable', 'Off');
        else
            set(push_ok, 'Enable', 'On');
        end
    end


    function check_JW_Callback(hObject, eventdata)
        if get(hObject, 'Value') == 1
            jwflag = true;
            set(check_stand, 'Enable', 'Off');
        else
            jwflag = false;
            set(check_stand, 'Enable', 'On');
        end
    end


    function pop_SW_Callback(hObject, eventdata)
        swflag = get(hObject, 'Value') - 1;
        if swflag
            set(check_stand, 'Enable', 'Off');
        else
            set(check_stand, 'Enable', 'On');
        end
    end


    function edit_states_Callback(hObject, eventdata)
        if str2double(get(hObject, 'String')) > 0 && get(check_user, 'Value')
            set(push_states, 'Enable', 'On');
        else
            set(push_states, 'Enable', 'Off');
        end
    end


    function push_states_Callback(hObject, eventdata)
        nstates = str2double(get(edit_states, 'String'));
        trim_param = labels(:,1);

        state_mat = UserManeuver(nstates, trim_param);
        
        if ~isempty(state_mat) && get(check_user, 'Value')
            set(push_file, 'Enable', 'On');
            TRIMprintf(state_mat);
        else
            set(push_file, 'Enable', 'Off');
            fprintf(1, '\nNo extra trim conditions defined.\n');
        end
        
        set(push_ok, 'Enable', 'Off');
    end


    function push_file_Callback(hObject, eventdata)

        formats = { '*.inc', 'Trim cards (*.inc)';...
                    '*.*', 'All files (*.*)' };
        [trim_filename, trim_pathname, trim_filterindex] = uiputfile(formats, 'Trim cards');
        
        if (trim_filterindex ~= 0),
            trim_filename = uiputext(trim_filename, trim_filterindex, formats);
            
            filename = [trim_pathname, trim_filename];
        elseif isempty(get(edit_file, 'String')),
            filename = fullfile(path, 'guess_trim.inc');
        end
        set(edit_file, 'String', filename);
        
        set(push_ok, 'Enable', 'On');
 
    end


    function check_load_Callback(hObject, eventdata)
        faroff;
        set(check_load, 'Value', true);
        set(check_far, 'Value', false);
        set(check_user, 'Value', false);
        if ~get(check_stand, 'Value')
            set(check_JW, 'Enable', 'On');
        end
        if ~get(check_stand, 'Value')
            set(pop_SW, 'Enable', 'On');
        end
        set(push_states, 'Enable', 'Off');
        set(push_file, 'Enable', 'Off');
        set(edit_file, 'Enable', 'Off');
        set(push_load, 'Enable', 'On');
        filename = get(edit_load, 'String');
        
        if isempty(get(edit_load, 'String')),
            set(push_ok, 'Enable', 'Off');
        else
            set(push_ok, 'Enable', 'On');
        end
        
    end


    function push_load_Callback(hObject, eventdata)

        formats = { '*.inc', 'Trim cards (*.inc)';...
                    '*.*', 'All files (*.*)' };
        [trim_filename, trim_pathname, trim_filterindex] = uigetfile(formats, 'Trim cards');
        
        if (trim_filterindex ~= 0),
            filename = [trim_pathname, trim_filename];
            set(edit_load, 'String', filename);
            set(push_ok, 'Enable', 'On');
        end
 
    end


    function check_pul_Callback(hObject, eventdata)
        
    end


    function check_hor_Callback(hObject, eventdata)
        
    end


    function check_ver_Callback(hObject, eventdata)
        
    end


    function check_ail_Callback(hObject, eventdata)
        
    end


    function check_gus_Callback(hObject, eventdata)
        
    end


    function check_lan_Callback(hObject, eventdata)
        
    end


    function check_eng_Callback(hObject, eventdata)
        
    end


    function check_hl_Callback(hObject, eventdata)
        
    end


    function edit_hcru_Callback(hObject, eventdata)
        
    end


    function edit_mcru_Callback(hObject, eventdata)
        
    end


    function edit_hmax_Callback(hObject, eventdata)
        
    end


    function edit_clmax_Callback(hObject, eventdata)
        
    end


    function edit_clmaxto_Callback(hObject, eventdata)
        
    end


    function edit_clmaxl_Callback(hObject, eventdata)
        
    end


    function edit_clalphad_Callback(hObject, eventdata)
        
    end


    function edit_sref_Callback(hObject, eventdata)
        
    end


    function edit_flapto_Callback(hObject, eventdata)
        
    end


    function edit_flapland_Callback(hObject, eventdata)
        
    end


    function edit_vsink_Callback(hObject, eventdata)
        
    end


    function edit_stroke_Callback(hObject, eventdata)
        
    end


    function edit_lndeff_Callback(hObject, eventdata)
        
    end
            

    function ok_Callback(hObject, eventdata)
        filename_trim = '';
        
        if ~isempty(filename) && (get(check_user, 'Value') || get(check_load, 'Value') || get(check_far, 'Value')),
            fprintf(1, '\n - GUESS/SMARTCAD trim interface enabled. \n');
        else
            filename = [];
        end
        
        if get(check_far, 'Value')
            if isempty(get(edit_hcru, 'String')) || isempty(get(edit_mcru, 'String'))
                warndlg('In EASA regulations mode at least HCRU and MCRU must be set.', 'NeoCASS - guess2trim');
                return
            end
            %
            filename = get(edit_file, 'String');
            if isempty(filename)
                warndlg('Please, insert trim file name.', 'NeoCASS - guess2trim');
                return
            end
            [ManCheck, HCRU, MCRU, HMAX, ...
            CLMAX, CLMAXTO, CLMAXL, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
            VSINK, STROKE, LNDGEFF, FarIndex] = getfar();
            MAN = norm_flight_points(1, HCRU, MCRU, HMAX, ...
                CLMAX, CLMAXTO, CLMAXL, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
                VSINK, STROKE, LNDGEFF, jwflag, swflag, filename_aircraft, filename, ManCheck, FarIndex);
        elseif get(check_user, 'Value')
            filename = get(edit_file, 'String');
            %
            fp = fopen(filename, 'w');
            % TRIM card
            writeTRIM2file(fp, state_mat);
            %
            fclose(fp);
        end
        
        guessstd = get(check_stand, 'Value');
        filename_trim = filename;
        close(hfig);
    end


    function apply_Callback(hObject, eventdata)
        if get(check_load, 'Value')
            return
        end
        
        filename_trim = '';
        
        if ~isempty(filename) && (get(check_user, 'Value') || get(check_load, 'Value') || get(check_far, 'Value')),
            fprintf(1, '\n - GUESS/SMARTCAD trim interface enabled. \n');
        else
            filename = [];
        end
        
        if get(check_far, 'Value')
            if isempty(get(edit_hcru, 'String')) || isempty(get(edit_mcru, 'String'))
                warndlg('In EASA regulations mode at least HCRU and MCRU must be set.', 'NeoCASS - guess2trim');
                return
            end
            %
            filename = get(edit_file, 'String');
            if isempty(filename)
                warndlg('Please, insert trim file name.', 'NeoCASS - guess2trim');
                return
            end
            [ManCheck, HCRU, MCRU, HMAX, ...
            CLMAX, CLMAXTO, CLMAXL, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
            VSINK, STROKE, LNDGEFF, FarIndex] = getfar();
            MAN = norm_flight_points(1, HCRU, MCRU, HMAX, ...
                CLMAX, CLMAXTO, CLMAXL, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
                VSINK, STROKE, LNDGEFF, jwflag, swflag, filename_aircraft, filename, ManCheck, FarIndex);
        elseif get(check_user, 'Value')
            filename = get(edit_file, 'String');
            %
            fp = fopen(filename, 'w');
            % TRIM card
            writeTRIM2file(fp, state_mat);
            %
            fclose(fp);
        end
        
        guessstd = get(check_stand, 'Value');
        filename_trim = filename;
    end


    function cancel_Callback(hObject, eventdata)
        state_mat = cell(0, nlabel);
        filename_trim = '';
        close(hfig);
    end
    



    function faroff()
        set(pop_far, 'Enable', 'Off');
        set(check_pul, 'Enable', 'Off');
        set(check_hor, 'Enable', 'Off');
        set(check_ver, 'Enable', 'Off');
        set(check_ail, 'Enable', 'Off');
        set(check_gus, 'Enable', 'Off');
        set(check_lan, 'Enable', 'Off');
        set(check_eng, 'Enable', 'Off');
        set(check_hl, 'Enable', 'Off');
        
        set(edit_hcru, 'Enable', 'Off');
        set(edit_mcru, 'Enable', 'Off');
        set(edit_hmax, 'Enable', 'Off');
        set(edit_clmax, 'Enable', 'Off');
        set(edit_clmaxto, 'Enable', 'Off');
        set(edit_clmaxl, 'Enable', 'Off');
        set(edit_clalphad, 'Enable', 'Off');
        set(edit_sref, 'Enable', 'Off');
        set(edit_flapto, 'Enable', 'Off');
        set(edit_flapland, 'Enable', 'Off');
        set(edit_vsink, 'Enable', 'Off');
        set(edit_stroke, 'Enable', 'Off');
        set(edit_lndeff, 'Enable', 'Off');
    end

    function faron()
        set(pop_far, 'Enable', 'On');
        set(check_pul, 'Enable', 'On');
        set(check_hor, 'Enable', 'On');
        set(check_ver, 'Enable', 'On');
        set(check_ail, 'Enable', 'On');
        set(check_gus, 'Enable', 'On');
        set(check_lan, 'Enable', 'On');
        set(check_eng, 'Enable', 'On');
        set(check_hl, 'Enable', 'On');
        
        set(edit_hcru, 'Enable', 'On');
        set(edit_mcru, 'Enable', 'On');
        set(edit_hmax, 'Enable', 'On');
        set(edit_clmax, 'Enable', 'On');
        set(edit_clmaxto, 'Enable', 'On');
        set(edit_clmaxl, 'Enable', 'On');
        set(edit_clalphad, 'Enable', 'On');
        set(edit_sref, 'Enable', 'On');
        set(edit_flapto, 'Enable', 'On');
        set(edit_flapland, 'Enable', 'On');
        set(edit_vsink, 'Enable', 'On');
        set(edit_stroke, 'Enable', 'On');
        set(edit_lndeff, 'Enable', 'On');
    end

    function [ManCheck, HCRU, MCRU, HMAX, ...
            CLMAX, CLMAXTO, CLMAXL, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
            VSINK, STROKE, LNDGEFF, FarIndex] = getfar()
        ManCheck = [get(check_pul, 'Value') ...
                    get(check_hor, 'Value') ...
                    get(check_ver, 'Value') ...
                    get(check_ail, 'Value') ...
                    get(check_gus, 'Value') ...
                    get(check_lan, 'Value') ...
                    get(check_eng, 'Value') ...
                    get(check_hl, 'Value')];
        HCRUstr = get(edit_hcru, 'String');
        if isempty(HCRUstr)
            HCRU = -1;
        else
            HCRU = str2double(HCRUstr);
        end
        MCRUstr = get(edit_mcru, 'String');
        if isempty(MCRUstr)
            MCRU = -1;
        else
            MCRU = str2double(MCRUstr);
        end
        HMAXstr = get(edit_hmax, 'String');
        if isempty(HMAXstr)
            HMAX = -1;
        else
            HMAX = str2double(HMAXstr);
        end
        CLMAXstr = get(edit_clmax, 'String');
        if isempty(CLMAXstr)
            CLMAX = -1;
        else
            CLMAX = str2double(CLMAXstr);
        end
        CLMAXTOstr = get(edit_clmaxto, 'String');
        if isempty(CLMAXTOstr)
            CLMAXTO = -1;
        else
            CLMAXTO = str2double(CLMAXTOstr);
        end
        CLMAXLstr = get(edit_clmaxl, 'String');
        if isempty(CLMAXLstr)
            CLMAXL = -1;
        else
            CLMAXL = str2double(CLMAXLstr);
        end
        CLALPHADstr = get(edit_clalphad, 'String');
        if isempty(CLALPHADstr)
            CLALPHAD = -1;
        else
            CLALPHAD = str2double(CLALPHADstr);
        end
        USERSREFstr = get(edit_sref, 'String');
        if isempty(USERSREFstr)
            USERSREF = -1;
        else
            USERSREF = str2double(USERSREFstr);
        end
        FLAPTOstr = get(edit_flapto, 'String');
        if isempty(FLAPTOstr)
            FLAPTO = -1;
        else
            FLAPTO = str2double(FLAPTOstr);
        end
        FLAPLANDstr = get(edit_flapland, 'String');
        if isempty(FLAPLANDstr)
            FLAPLAND = -1;
        else
            FLAPLAND = str2double(FLAPLANDstr);
        end
        VSINKstr = get(edit_vsink, 'String');
        if isempty(VSINKstr)
            VSINK = -1;
        else
            VSINK = str2double(VSINKstr);
        end
        STROKEstr = get(edit_stroke, 'String');
        if isempty(STROKEstr)
            STROKE = -1;
        else
            STROKE = str2double(STROKEstr);
        end
        LNDGEFFstr = get(edit_lndeff, 'String');
        if isempty(LNDGEFFstr)
            LNDGEFF = -1;
        else
            LNDGEFF = str2double(LNDGEFFstr);
        end
        FarIndex = get(pop_far, 'Value') - 1;
        
    end





end


