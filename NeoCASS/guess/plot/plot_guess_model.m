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

function plot_guess_model(set, fignr, guess_model)

switch set
        
    case 1
       setplot_fig1(fignr, guess_model);
        
    case 2
        setplot_fig2(fignr, guess_model);
        
    case 3
        setplot_fig3(fignr, guess_model);
        
    case 4
        setplot_fig4(fignr, guess_model);
    
    case 5
        setplot_fig5(fignr, guess_model);

    case 6
        setplot_fig6(fignr, guess_model);    
        
    case 7
        setplot_fig7(fignr, guess_model);
        
    case 8
        setplot_fig8(fignr, guess_model);
        
    case 9
        setplot_fig9(fignr, guess_model);        
        
    case 10
        setplot_fig10(fignr, guess_model);
        
    case 11
        setplot_fig11(fignr, guess_model);
        
    case 12
        setplot_fig12(fignr, guess_model);
        
    case 13
        setplot_fig13(fignr, guess_model);

    case 14
        setplot_fig14(fignr, guess_model);

    case 15
        setplot_fig15(fignr, guess_model);
        
    otherwise        
      fprintf(1, '\n - Required GUESS set not available.\n');
  
end

end
%-------------------------------------------------------------------------------
% fuselage shear
function [] = setplot_fig1(fignr, guess_model)
  figure(fignr); close; figure(fignr); hold on;
  if (guess_model.pdcylin.stick.model.fuse)
    plot(guess_model.geo.fus.x./...
    guess_model.geo.fus.x(end), guess_model.loads.fus.FS/1000, ...
    '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    xlabel('Fuselage axis y'), ylabel('F [KN]')
    title('GUESS - section-wise sizing shear force (multiplied x 1.5)')
  else
    fprintf(1, '\n - No fuselage data available.');
  end
end
%-------------------------------------------------------------------------------
function [] = setplot_fig2(fignr, guess_model)
% fuselage bending
  figure(fignr); close; figure(fignr); hold on;
  if (guess_model.pdcylin.stick.model.fuse)
    plot(guess_model.geo.fus.x./...
    guess_model.geo.fus.x(end), guess_model.loads.fus.M/1000, ...
    '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    xlabel('Fuselage axis y'), ylabel('M [KNm]')
    title('GUESS - section-wise sizing bending (multiplied x 1.5)')
  else
    fprintf(1, '\n - No fuselage data available.');
  end
end
%-------------------------------------------------------------------------------
function [] = setplot_fig3(fignr, guess_model)
% fuselage inertia
  figure(fignr); close; figure(fignr); hold on;
  if (guess_model.pdcylin.stick.model.fuse)
    plot(guess_model.geo.fus.x./...
    guess_model.geo.fus.x(end),guess_model.str_prop.fus.I1, ...
    '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    xlabel('Fuselage axis y'), ylabel('I1 [m^4]')
    title('GUESS - section-wise moment of inertia')
  else
    fprintf(1, '\n - No fuselage data available.');
  end
end
%-------------------------------------------------------------------------------
function [] = setplot_fig4(fignr, guess_model)
% fuselage thickness
  figure(fignr); close; figure(fignr); hold on;
  if (guess_model.pdcylin.stick.model.fuse)
    plot(guess_model.geo.fus.x./...
    guess_model.geo.fus.x(end),guess_model.str_prop.fus.tbar_S.*1000, ...
    '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    plot(guess_model.geo.fus.x./...
    guess_model.geo.fus.x(end),guess_model.str_prop.fus.tbar_F.*1000, ...
    '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    xlabel('Fuselage axis y'), ylabel('t [mm]')
    title('GUESS - thickness')
    legend('Equiv. shell thickness', 'Equiv. frame thickness');
  else
    fprintf(1, '\n - No fuselage data available.');
  end
end
%-------------------------------------------------------------------------------
function [] = setplot_fig6(fignr, guess_model)
% lifting surfaces shear
  figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
       (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.loads.wing.FS/1000, ...
      '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
    plot(guess_model.geo.vtail.y./guess_model.geo.vtail.y(end), ...
       guess_model.loads.vtail.FS/1000, ...
      '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
    plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
       (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.loads.htail.FS/1000, ...
      '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
    LABEL{cont} = 'Htail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.canard_carryth+1;
    plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
       (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.loads.canard.FS/1000, ...
      '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('F [KN]')
  title('GUESS - section-wise sizing shear force (multiplied x 1.5)')
 legend(LABEL);
end
%-------------------------------------------------------------------------------
function [] = setplot_fig7(fignr, guess_model)
% lifting surfaces bending
    figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
       (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.loads.wing.M/1000, ...
      '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
    plot(guess_model.geo.vtail.y./guess_model.geo.vtail.y(end), ...
       guess_model.loads.vtail.M/1000, ...
      '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
    plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
       (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.loads.htail.M/1000, ...
      '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
    LABEL{cont} = 'Htail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.canard_carryth+1;
    plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
       (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.loads.canard.M/1000, ...
      '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('F [KN]')
  title('GUESS - section-wise sizing bending (multiplied x 1.5)')
  legend(LABEL);

end
%-------------------------------------------------------------------------------
function [] = setplot_fig8(fignr, guess_model)
% lifting surfaces torque
    figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
       (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.loads.wing.Mt/1000, ...
      '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
    plot(guess_model.geo.vtail.y./guess_model.geo.vtail.y(end), ...
       guess_model.loads.vtail.Mt/1000, ...
      '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
    plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
       (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.loads.htail.Mt/1000, ...
      '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
    LABEL{cont} = 'Htail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.canard_carryth+1;
    plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
       (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.loads.canard.Mt/1000, ...
      '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('F [KN]')
  title('GUESS - section-wise sizing torque (multiplied x 1.5)')
  legend(LABEL);

end
%-------------------------------------------------------------------------------
function [] = setplot_fig9(fignr, guess_model)
% lifting surfaces skin thickness
  figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    if (guess_model.pdcylin.wing.kcon<=6)
      plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
        (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.tC(offset:end)*1000, ...
          '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    else
      plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
      (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.ttorq(offset:end)*1000, ...
      '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    end
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
    if (guess_model.pdcylin.vtail.kcon<=6)
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.tC*1000, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    else
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.ttorq*1000, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    end
    LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
    if (guess_model.pdcylin.htail.kcon<=6)
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.tC(offset:end)*1000, ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
    else
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.ttorq(offset:end)*1000, ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
    end
    LABEL{cont} = 'Htail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.ncanard_carryth+1;
    if (guess_model.pdcylin.canard.kcon<=6)
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.tC(offset:end)*1000, ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    else
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.ttorq(offset:end)*1000, ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    end
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('t [mm]')
  legend(LABEL);
  title('GUESS - skin thickness')
end
%-------------------------------------------------------------------------------
function [] = setplot_fig10(fignr, guess_model)
% lifting surfaces web thickness
  figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    if (guess_model.pdcylin.wing.kcon<=6)
      plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
        (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.tW(offset:end)*1000, ...
          '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    else
      plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
      (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.twbs(offset:end)*1000, ...
      '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    end
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
    if (guess_model.pdcylin.vtail.kcon<=6)
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.tW*1000, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    else
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.twbs*1000, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
    end
    LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
    if (guess_model.pdcylin.htail.kcon<=6)
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.tW(offset:end)*1000, ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
    else
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.twbs(offset:end)*1000, ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
    end
    LABEL{cont} = 'Htail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.ncanard_carryth+1;
    if (guess_model.pdcylin.canard.kcon<=6)
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.tW(offset:end)*1000, ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    else
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.twbs(offset:end)*1000, ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    end
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('t [mm]')
  legend(LABEL);
  title('GUESS - web thickness')
end
%-------------------------------------------------------------------------------
function [] = setplot_fig11(fignr, guess_model)
% lifting surfaces inertia
figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
        (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.I1(offset:end), ...
        '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.I1, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
      LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.I1(offset:end), ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
      LABEL{cont} = 'Htail'; cont = cont+1;
    end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.ncanard_carryth+1;
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.I1(offset:end), ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('I1 [m^4]')
  legend(LABEL);
  title('GUESS - out of plane moment of inertia')
end
%-------------------------------------------------------------------------------
function [] = setplot_fig12(fignr, guess_model)
% lifting surfaces inertia
figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
        (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.I2(offset:end), ...
        '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.I2, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
      LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.I2(offset:end), ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
      LABEL{cont} = 'Htail'; cont = cont+1;
    end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.ncanard_carryth+1;
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.I2(offset:end), ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('I2 [m^4]')
  legend(LABEL);
    title('GUESS - in-plane moment of inertia')
end
%-------------------------------------------------------------------------------
function [] = setplot_fig13(fignr, guess_model)
% lifting surfaces torsional constant
figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
        (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.J(offset:end), ...
        '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.J, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
      LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.J(offset:end), ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
      LABEL{cont} = 'Htail'; cont = cont+1;
    end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.ncanard_carryth+1;
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.J(offset:end), ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('J [m^4]')
  legend(LABEL);
  title('GUESS - torsional constant')
end
%-------------------------------------------------------------------------------
function [] = setplot_fig14(fignr, guess_model)
% fuselage bending
  figure(fignr); close; figure(fignr); hold on;
  if (guess_model.pdcylin.stick.model.fuse)
    plot(guess_model.geo.fus.x./...
    guess_model.geo.fus.x(end), guess_model.str_prop.fus.m, ...
    '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    xlabel('Fuselage axis y'), ylabel('m [Kg]')
    title('GUESS - mass')
  else
    fprintf(1, '\n - No fuselage data available.');
  end
end
%-------------------------------------------------------------------------------
function [] = setplot_fig15(fignr, guess_model)
% lifting surfaces mass
figure(fignr); close; figure(fignr); hold on;
  LABEL = {}; cont = 1;
  if (guess_model.pdcylin.stick.model.winr)
    offset = guess_model.pdcylin.stick.nwing_carryth+1;
    plot((guess_model.geo.wing.y(offset:end)-guess_model.geo.wing.y(offset))./...
        (guess_model.geo.wing.y(end)-guess_model.geo.wing.y(offset)), guess_model.str_prop.wing.m(offset:end), ...
        '-k.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','k')
    LABEL{cont} = 'Wing'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.vert)
      plot(guess_model.geo.vtail.y./...
        guess_model.geo.vtail.y(end), guess_model.str_prop.vtail.mbox, ...
          '-r.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','r')
      LABEL{cont} = 'Vtail'; cont = cont+1;
  end
  if (guess_model.pdcylin.stick.model.horr)
    offset = guess_model.pdcylin.stick.nhtail_carryth+1;
      plot((guess_model.geo.htail.y(offset:end)-guess_model.geo.htail.y(offset))./...
        (guess_model.geo.htail.y(end)-guess_model.geo.htail.y(offset)), guess_model.str_prop.htail.m(offset:end), ...
          '-b.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','b')
      LABEL{cont} = 'Htail'; cont = cont+1;
    end
  if (guess_model.pdcylin.stick.model.canr)
    offset = guess_model.pdcylin.stick.ncanard_carryth+1;
      plot((guess_model.geo.canard.y(offset:end)-guess_model.geo.canard.y(offset))./...
        (guess_model.geo.canard.y(end)-guess_model.geo.canard.y(offset)), guess_model.str_prop.canard.m(offset:end), ...
          '-c.', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor','c')
    LABEL{cont} = 'Canard'; cont = cont+1;
  end
  xlabel('Spanwise distance y'), ylabel('m [kg]')
  legend(LABEL);
  title('GUESS - mass')
end