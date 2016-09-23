function paintwe = paint_weight(S)
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
%     120217      2.1     L.Riccobene      Creation
%
%**************************************************************************
%
% function         paintwe = paint_weight(S)
%
%
%   DESCRIPTION:   
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%             
%                S             double      total aircraft wetted area   
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%                paintwe        double     paint mass [Kg]  
%                
%    REFERENCES:
%
%**************************************************************************

paintwe = 0.12 * real(S);