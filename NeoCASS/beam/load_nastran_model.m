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
%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     080101      1.0      L.Cavagna        Creation
%     121023      2.1.476  L.Riccobene      Modification
%
%*******************************************************************************
%
% function beam_model = load_nastran_model(filename)
%
%   DESCRIPTION: Load NEOCASS nastran-like file and creates beam_model database 
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                filename       FILE       NEOCASS nastran-like input file
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                beam_model     struct     beam_model database with structural
%                                          and aerodynamic data
%    REFERENCES:
%
%*******************************************************************************

function beam_model = load_nastran_model(filename)

% Recover version
fprintf(1, '\n%s\n', get_neocass_version('NeoCASS'));

[beam_model.Info, beam_model.Param, beam_model.Coord, beam_model.Node, ...
    beam_model.Mat, beam_model.Bar, beam_model.PBar, beam_model.Beam, ...
    beam_model.PBeam, beam_model.F, beam_model.M, beam_model.F_FLW, ...
    beam_model.ConM, beam_model.WB, beam_model.SPC, beam_model.Aero, ...
    beam_model.Optim, beam_model.Celas,beam_model.RBE2, beam_model.Gust,...
    beam_model.SET, beam_model.Surfdef, beam_model.Dextload, beam_model.Damp, ...
    beam_model.DesOPT] = ...
    read_nas_file(filename);

try
    % Compute MAC and its apex and store it into beam_model (MAC here is
    % computed on the actual aerodynamic mesh and not resorting to an
    % equivalent reference wing)
    [MAC, XLEMAC, ~] = RecoverMACInfoFromBeamModel(beam_model);  
    beam_model.Aero.ref.C_mac     = MAC;
    beam_model.Aero.ref.XLE_C_mac = XLEMAC;
catch excep
    beam_model.Aero.ref.C_mac     = 1;
    beam_model.Aero.ref.XLE_C_mac = 0;
end

beam_model.Res = [];
