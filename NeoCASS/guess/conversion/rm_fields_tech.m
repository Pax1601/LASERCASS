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

function [pdcylin_new] = rm_fields_tech(pdcylin)
% Remove not-necessary fields in tech file

%--------------------------------------------------------------------------
pdcylin.weight = rmfield(pdcylin.weight, 'wtff');
pdcylin.fact = rmfield(pdcylin.fact, 'nwing_inboard');
pdcylin.fact = rmfield(pdcylin.fact, 'nwing_midboard');
pdcylin.fact = rmfield(pdcylin.fact, 'nwing_outboard');
pdcylin.fact = rmfield(pdcylin.fact, 'nfuse');
pdcylin.fact = rmfield(pdcylin.fact, 'nvtail_inboard');
pdcylin.fact = rmfield(pdcylin.fact, 'nvtail_outboard');
pdcylin.fact = rmfield(pdcylin.fact, 'nhtail_inboard');
pdcylin.fact = rmfield(pdcylin.fact, 'nhtail_outboard');
pdcylin.fact = rmfield(pdcylin.fact, 'icomnd');
pdcylin.fact = rmfield(pdcylin.fact, 'wgno');
pdcylin.fact = rmfield(pdcylin.fact, 'slfmb');
pdcylin.fact = rmfield(pdcylin.fact, 'wmis');
pdcylin.fact = rmfield(pdcylin.fact, 'wsur');
pdcylin.fact = rmfield(pdcylin.fact, 'wcw');
pdcylin.fact = rmfield(pdcylin.fact, 'wca');
pdcylin.load = rmfield(pdcylin.load, 'axac');
pdcylin.load = rmfield(pdcylin.load, 'pg');
pdcylin.load = rmfield(pdcylin.load, 'wfbump');
pdcylin.load = rmfield(pdcylin.load, 'wfland');
pdcylin.wing = rmfield(pdcylin.wing, 'ps');
pdcylin.wing = rmfield(pdcylin.wing, 'kdew');
pdcylin.wing = rmfield(pdcylin.wing, 'kdfw');
pdcylin.wing = rmfield(pdcylin.wing, 'istama');
pdcylin.wing = rmfield(pdcylin.wing, 'cs1');
pdcylin.wing = rmfield(pdcylin.wing, 'cs2');
pdcylin.wing = rmfield(pdcylin.wing, 'ifuel');
pdcylin.wing = rmfield(pdcylin.wing, 'cwman');
pdcylin.wing = rmfield(pdcylin.wing, 'claqr');
pdcylin.ibredt = rmfield(pdcylin.ibredt, 'fus');
pdcylin.fus = rmfield(pdcylin.fus, 'kde');
pdcylin.fus = rmfield(pdcylin.fus, 'kdf');
pdcylin.fus = rmfield(pdcylin.fus, 'clbr1');
pdcylin.fus = rmfield(pdcylin.fus, 'icyl');
pdcylin = rmfield(pdcylin, 'WB_setup');
pdcylin = rmfield(pdcylin, 'opts');
pdcylin = rmfield(pdcylin, 'tail');
pdcylin = rmfield(pdcylin, 'lg');
%--------------------------------------------------------------------------
pdcylin_new = pdcylin;
neocass_xmlunwrapper('pdcylin_new.xml', pdcylin_new);
