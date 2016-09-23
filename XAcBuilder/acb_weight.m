%
% acb_weight.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	26.11.09	1.0	M. Lahuta	last update
%
%
% calls W&B module from AcBuilder when user selects 'Centers of gravity'
% from 'Weights & Ballance' menu
%
function acb_weight

global ac


op=pwd;
mp=mfilename('fullpath');
%wp=strrep(mp,[ 'Geometry' filesep 'AcBuilder' filesep 'acb_weight' ], [ 'Structure' filesep 'WB' ]);
wp=strrep(mp,'acb_weight', 'WB');
%wp=strrep(mp,[ 'AcBuilder' filesep 'acb_weight' ], [ 'svn/trunk/Structure/NeoCASS_V1.3/WB' ]);

try
            
    acb_geom
       
    disp('acbuilder: Calling W&B module ...');
    
    cd(wp);
    
    ac = weight_xml(ac);
        
    % LR 07/03/2012
    % Landing gear variables moved under weight_balance
    
    ac.miscellaneous.main_landing_gear_x_cg = ac.weight_balance.miscellaneous.main_landing_gear_x_cg;
    ac.miscellaneous.main_landing_gear_y_cg = ac.weight_balance.miscellaneous.main_landing_gear_y_cg;
    ac.miscellaneous.main_landing_gear_z_cg = ac.weight_balance.miscellaneous.main_landing_gear_z_cg;
    ac.miscellaneous.aux_landing_gear_x_cg  = ac.weight_balance.miscellaneous.aux_landing_gear_x_cg;
    %
    % SR 17.03.2012: removed Y position of Aux Landing Gear
    %%%ac.miscellaneous.aux_landing_gear_y_cg  = ac.weight_balance.miscellaneous.aux_landing_gear_y_cg;
    ac.miscellaneous.aux_landing_gear_z_cg  = ac.weight_balance.miscellaneous.aux_landing_gear_z_cg;

    
    % these variables must be copied to _x_cg etc. versions
    %ac.weight_balance.MTOW_CoG_x_cg=ac.weight_balance.MEW_longitudinal_CoG;
    %ac.weight_balance.MTOW_CoG_y_cg=ac.weight_balance.MEW_lateral_CoG;
    %ac.weight_balance.MTOW_CoG_z_cg=ac.weight_balance.MEW_vertical_CoG;
    %ac.weight_balance.MEW_CoG_x_cg=Maximum_payload_at_MTOW_longitudinal_CoG;
    %ac.weight_balance.MEW_CoG_y_cg=Maximum_payload_at_MTOW_lateral_CoG;
    %ac.weight_balance.MEW_CoG_z_cg=Maximum_payload_at_MTOW_vertical_CoG;
    
    ac.weight_balance.MEW_CoG_x_cg=ac.weight_balance.MEW_longitudinal_CoG;
    ac.weight_balance.MEW_CoG_y_cg=ac.weight_balance.MEW_lateral_CoG;
    ac.weight_balance.MEW_CoG_z_cg=ac.weight_balance.MEW_vertical_CoG;
    ac.weight_balance.MTOW_CoG_x_cg=ac.weight_balance.Maximum_payload_at_MTOW_longitudinal_CoG;
    ac.weight_balance.MTOW_CoG_y_cg=ac.weight_balance.Maximum_payload_at_MTOW_lateral_CoG;
    ac.weight_balance.MTOW_CoG_z_cg=ac.weight_balance.Maximum_payload_at_MTOW_vertical_CoG;
    
catch
    disp('error in acb_weight:');
    s=lasterror;
    disp(s.message);
    for i=1:size(s.stack,1)
        disp([ 'file: ''' s.stack(i).file '''  line: ' num2str(s.stack(i).line) ]);
    end
end

cd(op);
