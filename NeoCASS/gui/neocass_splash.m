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
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080224      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function function neocass_splash
%
%   DESCRIPTION: Display Neocass-LOGO
%
%         INPUT: NAME           TYPE       DESCRIPTION
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************


function neocass_splash

FILENAME = 'neocass_logo.png';
DELAY = 2;
LOGO = imread(FILENAME);
fh = figure('Visible','off','MenuBar','none', 'Name', 'NeoCASS logo', 'NumberTitle','off');

% put an axes in it
ah = axes('Parent',fh,'Visible','off');

% put the image in it
ih = image(LOGO,'parent',ah);
% set the figure size to be just big enough for the image, and centered at
% the center of the screen
imxpos = get(ih,'XData');
imypos = get(ih,'YData');
set(ah,'Unit','Normalized','Position',[0,0,1,1], 'TickDir','out', 'MinorGridLineStyle','none');
figpos = get(fh,'Position');
figpos(3:4) = [imxpos(2) imypos(2)];
set(fh,'Position',figpos);
movegui(fh,'center')
% make the figure visible
set(fh,'Visible','on');
%ht = timer('StartDelay',5,'ExecutionMode','SingleShot');
%set(ht,'TimerFcn','close(fh);stop(ht);delete(ht)');
%start(ht);

pause(DELAY);
close(fh)
