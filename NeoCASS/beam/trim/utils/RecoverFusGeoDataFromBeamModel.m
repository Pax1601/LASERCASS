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
%     DATE        VERS      PROGRAMMER       DESCRIPTION
%     121018      2.1.472   L.Riccobene      Creation
%
%**************************************************************************
%
% function     [D, L] = RecoverFusGeoDataFromBeamModel(beam_model)
%
%
%   DESCRIPTION: Given the beam model recover fuselage max diameter and
%                total fuselage length
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                beam_model     struct     beam model database
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                D              double     maximum diameter
% 
%                L              double     total fuselage length
%
%    REFERENCES:
%
%**************************************************************************
function [D, L] = RecoverFusGeoDataFromBeamModel(beam_model)
D = [];
L = [];
% Node identifiers and coordinates
Node    = beam_model.Node.Coord;
NodeID  = beam_model.Node.ID;

% Typically fuselage aeronode range from 10000 to 19999
FuseNID = 10000:19999;

% Extract fuselage aerodynamic nodes coordinates (along elastic axis)
Index    = ismember(NodeID, FuseNID);
if ~isempty(find(Index))
  FusANode = Node(Index, :);
  szNode   = size(FusANode, 1);
  % For each section compute the maximum diameter (nodes are supposed to be
  % in clockwise order)
  Dt = NaN(szNode/4, 1);
  id = @(x) (1:4) + 4*(x-1);
  for k = 1:szNode/4,
    SecCoord = FusANode(id(k), 2);
    HSecDiam = abs(SecCoord(2)-SecCoord(4));
    VSecDiam = abs(SecCoord(1)-SecCoord(3));
    Dt(k, 1) = max([HSecDiam VSecDiam]);
  end

  % Maximum fuselage diameter (absolute)
  D = max(Dt);
end
% Typically fuselage node range from 1000 to 1999
FuseNID = 1000:1999;

% Extract fuselage nodes coordinates (along elastic axis)
Index   = ismember(NodeID, FuseNID);
if ~isempty(find(Index))
  FusNode = Node(Index, :);

  % Total fuselage length
  L = norm(FusNode(end, :)-FusNode(1, :));
end