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

    function chord = tailchord(taper_kink,taper_tip,root_chord,ref_span,span_kink)
    %Created by Javier Muñoz in May 2010 in KTH University under
    %supervision of Arthur Rizzi
    %This function computes the local chord length at given span station
    %compute chord on actual planform geometry for HTAIL, VTAIL and CANARD.
    %The purpose is to avoid the mistakes made by the general function
    %qxcdcop.
    %It is only used during the center of gravity estimations.
    %Input parameters:
%     taper_kink = chord at the kink divided by root chord.
%     taper_tip = chord at the tip divided by root chord(Taper ratio).
%     root_chord
%     ref_span = Non dimensional distance from root to the point where the chord is calculated.
%     span_kink = Non dimensional distance form the root to the chord.

    if ref_span > span_kink
        chord = (taper_tip-taper_kink)*(ref_span-span_kink)*root_chord + taper_kink*root_chord;
    else
        chord = (taper_kink-1)*root_chord*ref_span + root_chord;
    end
  