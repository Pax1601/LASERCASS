%
% Run optimization probelm to distribute fuselage mass with mass and cg contraints
%
function mass_distr = massCONC_opt(nodes, domain, mass, CG, X0)
% Init vector
  mass_distr = zeros(1,length(X0));
  if (mass>0)
    % Find nodes inside domain
    indx = find( nodes>=domain(1) & nodes<=domain(2) );
    % Compute non structural mass as initial value linear density [kg/m]
    cg_prev = sum(X0.* nodes)/mass;
    % Optimization 
    mass_distr(1, indx) = opt_cg_pos(X0(indx), nodes(indx), mass, CG);
    cg = sum(mass_distr(indx).* nodes(indx))/sum(mass_distr(indx));
    %
    fprintf('\n\t - Optimization problem for fuselage lumped mass:');
    fprintf('\n\t\t- CG required: %g [m].', CG);
    fprintf('\n\t\t- CG trial solution: %g [m].', cg_prev);
    fprintf('\n\t\t- CG determined: %g [m].', cg);
    fprintf('\n\t\t- Mass required: %g [Kg].', mass);
    fprintf('\n\t\t- Mass determined: %g [Kg].', sum(mass_distr(indx)));
    % divide
  end
end

function mass_distr = opt_cg_pos(X0, nodes, MTOT, CG)
  nm = length(X0);
  OPTIONS =optimset('Algorithm','sqp', 'Display', 'notify-detailed');%,'GradObj', 'on','Display', 'notify-detailed','MaxFunEvals',500000, 'LargeScale', 'off', 'tolfun',1e-9);
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

