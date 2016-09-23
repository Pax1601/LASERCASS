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
%     090312      1.3     A. De Gaspari    Creation
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function sel = select_list(clist, label)
%
function sel = select_list(clist, label, title)
%
%
sel = [];
%
if nargin == 2,
    title = 'Selection List';
end
%
[nlab,l] = size(char(clist));
ch = 8;
dum = get(0,'ScreenSize');
wscr = dum(3);
hscr = dum(4);
lscr = wscr/ch;
lmax = .7*lscr;
% l = 1;
% for i = 1:length(clist),
%     if length(clist{i}) > l,
%         l = length(clist{i});
%     end
% end
if l > lmax,
    l = lmax;
end
wl = (l+1)*ch + 5*ch;
pl = 0.1*wl+3*ch;
wf = wl + 2 * pl;
%m = 1.35;
%wf = m * wl;
%pl = (wf-wl)/2;
hf = .45*hscr;
hl = .70*hf;
hpb = .055*hf;
phf = .2*hscr;
pht = 2*phf;
phl = .375*phf;
phpb = .14*phf;
%
mrat = 2.6*(wl/hpb)^1.3;
spc = .006*wl;
wpb = (wl-spc-2*mrat)/2;
p1wpb = pl+mrat;
pspc = pl+mrat+wpb;
p2wpb = pspc+spc;
%
%

hfig = figure('MenuBar','none','Name',title,'NumberTitle','off','Position',[40,phf,wf,hf], 'Resize', 'Off');

uicontrol(hfig, 'Style', 'Text', 'Background',[.8 .8 .8], 'HorizontalAlignment', 'left', 'String',label,'Position',[pl,pht,wl,20]);
list = uicontrol(hfig, 'Style', 'Listbox', 'Background',[1. 1. 1.], 'Position', [pl,phl,wl,hl], 'FontName', 'FixedWidth');
uicontrol(hfig, 'Style', 'PushButton','String','Ok','Position',[p1wpb,phpb,wpb,hpb],'CallBack',@smode_ok);
uicontrol(hfig, 'Style', 'PushButton','String','Cancel','Position',[p2wpb,phpb,wpb,hpb],'CallBack',@smode_cancel);

set(list, 'Min', 1, 'Max', 3, 'String', clist);

uiwait(hfig);

function smode_ok(hObject, eventdata)
    sel = get(list, 'Value');
    close(hfig);
end

function smode_cancel(hObject, eventdata)
    close(hfig);
end


end


