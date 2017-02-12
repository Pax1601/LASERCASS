%
% Run optimization probelm to distribute fuselage mass with mass and cg contraints
%
function mass_distr = massNSM_distr_opt(nodes, domain, mass, CG, dq, qtot, l_beam)
TOLL = 1e-3;
% Init vector
  mass_distr = zeros(length(l_beam), 1);
  if (mass>0)
    % discard last node
    % Find nodes inside domain
    indx = find( nodes>=domain(1) & nodes<=domain(2) );
    % Compute total over domain (q can be either a wetted area or a volume)
    qt = sum(dq(indx));
    % Compute non structural mass as initial value linear density [kg/m]
    mass_distr(indx, 1) = mass.*(dq(indx)./qt);%./l_beam(indx);
    cg_prev = sum(mass_distr(indx, 1).* nodes(indx))/mass;
    % Optimization 
    mass_distr(indx, 1) = opt_cg_pos(mass_distr(indx, 1), nodes(indx), mass, CG);
    cg = sum(mass_distr(indx, 1).* nodes(indx))/sum(mass_distr(indx, 1));
    %
    fprintf('\n\t - Optimization problem for fuselage distributed mass:');
    fprintf('\n\t\t- CG required: %g [m].', CG);
    fprintf('\n\t\t- CG trial solution: %g [m].', cg_prev);
    fprintf('\n\t\t- CG determined: %g [m].', cg);
    fprintf('\n\t\t- Mass required: %g [Kg].', mass);
    fprintf('\n\t\t- Mass determined: %g [Kg].', sum(mass_distr(indx, 1)));
    if (abs(cg-cg_prev)/cg_prev>TOLL)
      mass_distr(indx, 1) = opt_cg_pos(ones(length(indx), 1)/length(indx), nodes(indx), mass, CG);
      cg = sum(mass_distr(indx, 1).* nodes(indx))/sum(mass_distr(indx, 1));
      %
      fprintf('\n\t\t  Solution dicarded.');
      fprintf('\n\t\t- Optimization problem for fuselage distributed mass:');
      fprintf('\n\t\t\t- CG required: %g [m].', CG);
      fprintf('\n\t\t\t- CG trial solution: %g [m].', cg_prev);
      fprintf('\n\t\t\t- CG determined: %g [m].', cg);
      fprintf('\n\t\t\t- Mass required: %g [Kg].', mass);
      fprintf('\n\t\t\t- Mass determined: %g [Kg].', sum(mass_distr(indx, 1)));
    end
    % divide
    mass_distr(indx, 1) = mass_distr(indx, 1)./l_beam(indx);
  end
end

function mass_distr = opt_cg_pos(X0, nodes, MTOT, CG)
  nm = length(X0);
  OPTIONS =optimset('Algorithm','sqp', 'Display', 'notify-detailed','tolfun',1e-9);%,'GradObj', 'on','Display', 'notify-detailed','MaxFunEvals',500000, 'LargeScale', 'off', 'tolfun',1e-9);
  mass_distr = X0;
  [mass_distr, fval, flag] = fmincon(@(x) myobj(x, nodes, CG, MTOT),X0, [],[],ones(1,nm),MTOT,zeros(nm,1),ones(nm,1)*100000,[],OPTIONS);
  if flag<0
    fprintf('\n### Warning: optimization process failes for mass distribution.');
    mass_distr = X0;
  end
end

function f = myobj(x, nodes, CG, MTOT)
  f = abs(sum(x.*nodes) - MTOT*CG)/MTOT*CG;
end

