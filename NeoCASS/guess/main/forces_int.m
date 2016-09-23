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

function [internal_forces]=forces_int(aero_forces, inertial_forces_NS, state, stick, distances, internal_forces, aircraft, acc)

% Modified by Travaglini 19/11/2009

%%% Wing_sx
[int_forces] =internal_f(aero_forces.wing_sx-inertial_forces_NS.wing, state, stick, distances.wing);

internal_forces.wing_sx.Tx=[internal_forces.wing_sx.Tx int_forces.Tx];
internal_forces.wing_sx.Ty=[internal_forces.wing_sx.Ty int_forces.Ty];
internal_forces.wing_sx.Tz=[internal_forces.wing_sx.Tz int_forces.Tz];
internal_forces.wing_sx.Mx=[internal_forces.wing_sx.Mx int_forces.Mx];
internal_forces.wing_sx.My=[internal_forces.wing_sx.My int_forces.My];
internal_forces.wing_sx.Mz=[internal_forces.wing_sx.Mz int_forces.Mz];



%%% Wing_dx
[int_forces] =internal_f(aero_forces.wing_dx-inertial_forces_NS.wing, state, stick, distances.wing);

internal_forces.wing_dx.Tx=[internal_forces.wing_dx.Tx int_forces.Tx];
internal_forces.wing_dx.Ty=[internal_forces.wing_dx.Ty int_forces.Ty];
internal_forces.wing_dx.Tz=[internal_forces.wing_dx.Tz int_forces.Tz];
internal_forces.wing_dx.Mx=[internal_forces.wing_dx.Mx int_forces.Mx];
internal_forces.wing_dx.My=[internal_forces.wing_dx.My int_forces.My];
internal_forces.wing_dx.Mz=[internal_forces.wing_dx.Mz int_forces.Mz];



if isequal(stick.model.canr, 1)
    
    %%% canard_sx
    [int_forces] =internal_f(aero_forces.canard_sx-inertial_forces_NS.canard, state, stick, distances.canard);
    
    internal_forces.canard_sx.Tx=[internal_forces.canard_sx.Tx int_forces.Tx];
    internal_forces.canard_sx.Ty=[internal_forces.canard_sx.Ty int_forces.Ty];
    internal_forces.canard_sx.Tz=[internal_forces.canard_sx.Tz int_forces.Tz];
    internal_forces.canard_sx.Mx=[internal_forces.canard_sx.Mx int_forces.Mx];
    internal_forces.canard_sx.My=[internal_forces.canard_sx.My int_forces.My];
    internal_forces.canard_sx.Mz=[internal_forces.canard_sx.Mz int_forces.Mz];
    
    
    %%% canard_dx
    [int_forces] =internal_f(aero_forces.canard_dx-inertial_forces_NS.canard, state, stick, distances.canard);
    
    internal_forces.canard_dx.Tx=[internal_forces.canard_dx.Tx int_forces.Tx];
    internal_forces.canard_dx.Ty=[internal_forces.canard_dx.Ty int_forces.Ty];
    internal_forces.canard_dx.Tz=[internal_forces.canard_dx.Tz int_forces.Tz];
    internal_forces.canard_dx.Mx=[internal_forces.canard_dx.Mx int_forces.Mx];
    internal_forces.canard_dx.My=[internal_forces.canard_dx.My int_forces.My];
    internal_forces.canard_dx.Mz=[internal_forces.canard_dx.Mz int_forces.Mz];
    
end




if isequal(aircraft.Vertical_tail.Twin_tail, 1)
    
    
    %%%Vtail
    [int_forces] =internal_f_vtail_twin(aero_forces.vtail-inertial_forces_NS.vtail, state, stick, distances.vtail, aircraft);
    
    
    internal_forces.vtail.Tx=[internal_forces.vtail.Tx int_forces.Tx];
    internal_forces.vtail.Ty=[internal_forces.vtail.Ty int_forces.Ty];
    internal_forces.vtail.Tz=[internal_forces.vtail.Tz int_forces.Tz];
    internal_forces.vtail.Mx=[internal_forces.vtail.Mx int_forces.Mx];
    internal_forces.vtail.My=[internal_forces.vtail.My int_forces.My];
    internal_forces.vtail.Mz=[internal_forces.vtail.Mz int_forces.Mz];
    
    %Vtail2
    [int_forces] =internal_f_vtail_twin(aero_forces.vtail2-inertial_forces_NS.vtail, state, stick, distances.vtail,  aircraft);
    
    internal_forces.vtail2.Tx=[internal_forces.vtail2.Tx int_forces.Tx];
    internal_forces.vtail2.Ty=[internal_forces.vtail2.Ty int_forces.Ty];
    internal_forces.vtail2.Tz=[internal_forces.vtail2.Tz int_forces.Tz];
    internal_forces.vtail2.Mx=[internal_forces.vtail2.Mx int_forces.Mx];
    internal_forces.vtail2.My=[internal_forces.vtail2.My int_forces.My];
    internal_forces.vtail2.Mz=[internal_forces.vtail2.Mz int_forces.Mz];
    
    
    
    %%% Htail_dx
    
    
    
    
    
    [int_forces] =internal_f_twin_htailr(aero_forces.htail_dx-inertial_forces_NS.htail, state, stick, distances.htail,...
        internal_forces.vtail, aircraft);
    
    
    internal_forces.htail_dx.Tx=[internal_forces.htail_dx.Tx int_forces.Tx];
    internal_forces.htail_dx.Ty=[internal_forces.htail_dx.Ty int_forces.Ty];
    internal_forces.htail_dx.Tz=[internal_forces.htail_dx.Tz int_forces.Tz];
    internal_forces.htail_dx.Mx=[internal_forces.htail_dx.Mx int_forces.Mx];
    internal_forces.htail_dx.My=[internal_forces.htail_dx.My int_forces.My];
    internal_forces.htail_dx.Mz=[internal_forces.htail_dx.Mz int_forces.Mz];
    
    
    %%% Htail_sx
    
    
    [int_forces] =internal_f_twin_htails(aero_forces.htail_sx-inertial_forces_NS.htail, state, stick, distances.htail,...
        internal_forces.vtail2, aircraft  );
    
    
    
    internal_forces.htail_sx.Tx=[internal_forces.htail_sx.Tx int_forces.Tx];
    internal_forces.htail_sx.Ty=[internal_forces.htail_sx.Ty int_forces.Ty];
    internal_forces.htail_sx.Tz=[internal_forces.htail_sx.Tz int_forces.Tz];
    internal_forces.htail_sx.Mx=[internal_forces.htail_sx.Mx int_forces.Mx];
    internal_forces.htail_sx.My=[internal_forces.htail_sx.My int_forces.My];
    internal_forces.htail_sx.Mz=[internal_forces.htail_sx.Mz int_forces.Mz];
    
    
    
    %%% Fuse
    [int_forces] =internal_f_fuse_twin(aero_forces.fuse-inertial_forces_NS.fuse, state, stick, distances.fuse, internal_forces.canard_dx,...
        internal_forces.canard_sx, internal_forces.wing_dx, internal_forces.wing_sx, internal_forces.htail_dx, internal_forces.htail_sx, aircraft, acc);
    
    internal_forces.fuse.Tx=[internal_forces.fuse.Tx int_forces.Tx];
    internal_forces.fuse.Ty=[internal_forces.fuse.Ty int_forces.Ty];
    internal_forces.fuse.Tz=[internal_forces.fuse.Tz int_forces.Tz];
    internal_forces.fuse.Mx=[internal_forces.fuse.Mx int_forces.Mx];
    internal_forces.fuse.My=[internal_forces.fuse.My int_forces.My];
    internal_forces.fuse.Mz=[internal_forces.fuse.Mz int_forces.Mz];
    
    
    
    
    
    
    
    
    
else
    if isequal(stick.model.horr, 1)
        
        %%% Htail_sx
        
        [int_forces] =internal_f(aero_forces.htail_sx-inertial_forces_NS.htail, state, stick, distances.htail);
        
        
        
        internal_forces.htail_sx.Tx=[internal_forces.htail_sx.Tx int_forces.Tx];
        internal_forces.htail_sx.Ty=[internal_forces.htail_sx.Ty int_forces.Ty];
        internal_forces.htail_sx.Tz=[internal_forces.htail_sx.Tz int_forces.Tz];
        internal_forces.htail_sx.Mx=[internal_forces.htail_sx.Mx int_forces.Mx];
        internal_forces.htail_sx.My=[internal_forces.htail_sx.My int_forces.My];
        internal_forces.htail_sx.Mz=[internal_forces.htail_sx.Mz int_forces.Mz];
        
        
        
        %%% Htail_dx
        
        [int_forces] =internal_f(aero_forces.htail_dx-inertial_forces_NS.htail, state, stick, distances.htail);
        
        
        
        internal_forces.htail_dx.Tx=[internal_forces.htail_dx.Tx int_forces.Tx];
        internal_forces.htail_dx.Ty=[internal_forces.htail_dx.Ty int_forces.Ty];
        internal_forces.htail_dx.Tz=[internal_forces.htail_dx.Tz int_forces.Tz];
        internal_forces.htail_dx.Mx=[internal_forces.htail_dx.Mx int_forces.Mx];
        internal_forces.htail_dx.My=[internal_forces.htail_dx.My int_forces.My];
        internal_forces.htail_dx.Mz=[internal_forces.htail_dx.Mz int_forces.Mz];
        
    end
    
    
    
    
    
    %%% Vtail
    
    
    [int_forces] =internal_f_vtail(aero_forces.vtail-inertial_forces_NS.vtail, state, stick, distances.vtail, internal_forces.htail_dx, internal_forces.htail_sx, aircraft);
    
    internal_forces.vtail.Tx=[internal_forces.vtail.Tx int_forces.Tx];
    internal_forces.vtail.Ty=[internal_forces.vtail.Ty int_forces.Ty];
    internal_forces.vtail.Tz=[internal_forces.vtail.Tz int_forces.Tz];
    internal_forces.vtail.Mx=[internal_forces.vtail.Mx int_forces.Mx];
    internal_forces.vtail.My=[internal_forces.vtail.My int_forces.My];
    internal_forces.vtail.Mz=[internal_forces.vtail.Mz int_forces.Mz];
    
    
    
    %%% Fuse
    [int_forces] =internal_f_fuse(aero_forces.fuse-inertial_forces_NS.fuse, state, stick, distances.fuse, internal_forces.canard_dx,...
        internal_forces.canard_sx, internal_forces.wing_dx, internal_forces.wing_sx, internal_forces.vtail, aircraft, acc);
    
    internal_forces.fuse.Tx=[internal_forces.fuse.Tx int_forces.Tx];
    internal_forces.fuse.Ty=[internal_forces.fuse.Ty int_forces.Ty];
    internal_forces.fuse.Tz=[internal_forces.fuse.Tz int_forces.Tz];
    internal_forces.fuse.Mx=[internal_forces.fuse.Mx int_forces.Mx];
    internal_forces.fuse.My=[internal_forces.fuse.My int_forces.My];
    internal_forces.fuse.Mz=[internal_forces.fuse.Mz int_forces.Mz];
    
    
end

