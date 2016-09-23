%
% Transform aero-transfer matrix from inertial reference frame to stability axes
% Rigid modes are supposed to be given as positive displacements along inertial axes.
% Inputs:
% - Ham: aero matrix with rigid motion columns only. 5 or 6 colums are allowed. 
%   5 is the case the mode along x-axis is not given.
% - k: list of reduced frequencies
% - cref: reference length used for reduced frequency, jk = jomega * cref / vref
% - ORDER: 2,4 order of approximation at zero frequency value
% Outputs:
% - Has: aerodynamic matrix in stability axes
% - knew: reduced frequency values, 0 value appended in first position
%
% u     = dot(x);
% beta  = dot(y)/V - psi
% alpha = dot(z)/V + theta 
% p     = dot(phi)
% q     = dot(theta)
% r     = dot(psi)

function [Hsa, knew] = iner2sa_neoresp(Ham, k, cref, ORDER)
%
  fid = 1;
  if ORDER == 4
    if (k(2)/k(1) ~= 2)
      fprintf(fid,'\n   ### Warning: cannot used interpolation of fourth order because k(2) is not twice k(1).');
      fprintf(fid,'\n                Second order will be applied.');
      ORDER = 2;
    end
  end
%
  kstep = 0.01;
  kint = [0:kstep:k(end)];
  fid = 1;
  nfig = 1;
  nr = size(Ham,1);
  nc = size(Ham,2);
  nk = length(k);
% colums to pick
  if nc == 5
    lg_dof = [2 4];
    la_dof = [1 3 5];
%
  elseif nc==6
    lg_dof = [3 5];
    la_dof = [2 4 6];
  else
    error('Wrong number of columns for Ham matrix. Five (without x-motion) or six columns must be provided.');
  end
%
  Ham(lg_dof,la_dof,:) = 0.0;
  Ham(la_dof,lg_dof,:) = 0.0;
%
%
% LONGITUDINAL TRANSFORMATION: ALPHA, Q
  fprintf(fid, '\n   Number of rows: %d.', nr);
  fprintf(fid, '\n   Number of columns: %d.', nc);
%
  fprintf(fid, '\n   Interpolation order: %d.', ORDER);
%
%
  fprintf(fid, '\n   Columns used for longitudinal transformation:');
  for i=1:length(lg_dof);
    fprintf(fid, ' %d', lg_dof(i));
  end  
  fprintf(fid, '.');
  fprintf(fid, '\n   Columns used for latero-directional transformation:');
  for i=1:length(la_dof);
    fprintf(fid, ' %d', la_dof(i));
  end  
  fprintf(fid, '.');
  fprintf(fid, '\n   Longitudinal transformation...');
  [Hlg, knew] = lg_transform(Ham(:,lg_dof,:), k, cref, ORDER, nfig);
  fprintf(fid, 'done.');
% LATERO-DIRECTIONAL TRANSFORMATION: BETA, P, R
  fprintf(fid, '\n   Latero-directional transformation...');
  [Hla, knew] = la_transform(Ham(:,la_dof,:), k, cref, ORDER, nfig);
  fprintf(fid, 'done.');
% FINAL ASSEMBLY
  Hsa = zeros(nr, nc, nk+1);
  Hsa(:,lg_dof,:) = Hlg;
  Hsa(:,la_dof,:) = Hla;
%
% copy first colum if given
%  if nc==6
%    for i=1:nk+1
%      Hsa(:,1,i) = Ham(:,1,i);
%    end
%  end
%
  fprintf(fid, '\n');
  flagplot = input('>> Do you want to plot the results? (y/n): ','s');
  flagplot = any( strcmp(flagplot,{'y','Y','yes','YES'}) );
  while flagplot
  % ask for matrix components to plot
    ask = 1;
    while ask
        row = input('>> Input a vector of the matrix rows to plot: ');
        if (row > nr)
            fprintf(1, ' - Warning: required row index exceeds matrix size (%d).\n\n', nr);
        else
            ask = 0;
        end
    end
    ask = 1;
    while ask
        col = input('>> Input a vector of the matrix column to plot: ');
        if (col > nc)
            fprintf(1, ' - Warning: required column index exceeds matrix size (%d).\n\n', nc);
        else
            ask = 0;
        end
    end
  %
    HsaINT = interp_coeff(knew, Hsa, row, col, kstep);
    HamINT = interp_coeff(k, Ham, row, col, kstep);
%
    figure(nfig); close; figure(nfig); hold on
    plot(knew, squeeze(real(Hsa(row,col,:))),'rs','MarkerSize',8,'MarkerFaceColor','r'); 
    plot(k, squeeze(real(Ham(row,col,:))),'ko','MarkerSize',8,'MarkerFaceColor','k'); 
    plot(kint, squeeze(real(HsaINT(1,1,:))),'-r'); 
    plot(kint, squeeze(real(HamINT(1,1,:))),'-k'); 
    axes1 = gca;
    set(axes1,'FontSize',14);
    xlabel('Reduced frequency k','FontSize',16,'FontWeight','Bold');
    ylabel('Real part','FontSize',16,'FontWeight','Bold');
    legend('Hsa','Ham');
    title(['Coefficient ',num2str(row),'-', num2str(col)]);
    print('-depsc2','-r200',['Has_real',num2str(row),'-', num2str(col)]);  
%
    figure(nfig+1); close; figure(nfig+1); hold on
    plot(knew, squeeze(imag(Hsa(row,col,:))),'rs','MarkerSize',8,'MarkerFaceColor','r');
    plot(k, squeeze(imag(Ham(row,col,:))),'ko','MarkerSize',8,'MarkerFaceColor','k');
    plot(kint, squeeze(imag(HsaINT(1,1,:))),'-r'); 
    plot(kint, squeeze(imag(HamINT(1,1,:))),'-k'); 
    axes1 = gca;
    set(axes1,'FontSize',14);
    xlabel('Reduced frequency k','FontSize',16,'FontWeight','Bold');
    ylabel('Imaginary part','FontSize',16,'FontWeight','Bold');
    legend('Hsa','Ham');
    title(['Coefficient ',num2str(row),'-', num2str(col)]);
    print('-depsc2','-r200',['Has_imag',num2str(row),'-', num2str(col)]);  
  %
    figure(nfig+2); close; figure(nfig+2); hold on;
    plot(squeeze(real(Hsa(row,col,:))), squeeze(imag(Hsa(row,col,:))),'rs','MarkerSize',8,'MarkerFaceColor','r');
    plot(squeeze(real(Ham(row,col,:))), squeeze(imag(Ham(row,col,:))),'ko','MarkerSize',8,'MarkerFaceColor','k');
    plot(squeeze(real(HsaINT(1,1,:))), squeeze(imag(HsaINT(1,1,:))),'-r'); 
    plot(squeeze(real(HamINT(1,1,:))),squeeze(imag(HamINT(1,1,:))),'-k'); 
    axes1 = gca;
    set(axes1,'FontSize',14);
    xlabel('Real part','FontSize',16,'FontWeight','Bold');
    ylabel('Imaginary part','FontSize',16,'FontWeight','Bold');
    legend('Hsa','Ham');
    title(['Coefficient ',num2str(row),'-', num2str(col)]);
    print('-depsc2','-r200',['Has',num2str(row),'-', num2str(col)]);  
  %
    flagplot = input('>> Do you want others plots? (y/n): ','s');
    flagplot = any( strcmp(flagplot,{'y','Y','yes','YES'}) );
  end
%
%
end
%
function [Haq, k] = lg_transform(Ham, k, cref, ORDER, nfig)
%
nr  = size(Ham,1);
nc  = size(Ham,2);
nk  = length(k);
dk  = k./cref;
dk0 = dk(1);
%
% change sign to plunge column if positive perturbation along z axis given
% alpha = theta - hdot/VREF
Haq = zeros(nr, 2, nk+1);
%
for j=1:nk
%
% Halpha = c*Hh/jk
  Halpha = Ham(:,1,j)./(dk(j)*1i);
% [Halpha, Hq] = [Halpha, c*(Htheta-Halpha)/jk]
  Haq(:,1:2,j+1) = [Halpha, (Ham(:,2,j)-Halpha)./(dk(j)*1i)];
%
end
%
%
switch(ORDER)
%
  case 2
    % Add zero frequency value
    % c*Hh', c*Htheta'
    D1HAM0 = imag(Ham(:,:,1))/dk0;
    % c*Halpha', c*Hq'
    D1H0 = imag(Haq(:,:,2))/dk0;
    %
    Haq(:,:,1) = [D1HAM0(:,1), (D1HAM0(:, 2) - D1H0(:,1))];
    % exploit Halpha = Htheta for k=0
    % Haq(:,:,1) = [real(Ham(:,2,1)), (D1HAM0(:, 2) - D1H0(:,1))];
  case 4
    if (k(2)/k(1) == 2)
      D1HAM0 = (-imag(Ham(:,:,2)) + 8*imag(Ham(:,:,1)))/(6*dk0);
      D1H0 = (-imag(Haq(:,:,3)) + 8*imag(Haq(:,:,2)))/(6*dk0);
      Haq(:,:,1) = [D1HAM0(:,1), (D1HAM0(:,2) - D1H0(:,1))];
    % Haq(:,:,1) = [real(Ham(:,2,1)), (D1HAM0(:, 2) - D1H0(:,1))];
    else
      error('The ratio of the first two values of reduced frequencies must be 2 when ORDER=4.');
    end
  otherwise
    error('Wrong approximation order. Allowed values are 2 and 4.');
%
end
%
k = [0, k];
%
end
%
%
function [Haq, k] = la_transform(Ham, k, cref, ORDER, nfig)
%
nr  = size(Ham,1);
nc  = size(Ham,2);
nk  = length(k);
dk  = k./cref;
dk0 = dk(1);
%
% change sign to roll and yaw rotations
Haq = zeros(nr, 3, nk+1);
%
for j=1:nk
%
% Hbeta = c*Hy/jk
  Hbeta = Ham(:,1,j)./(dk(j)*1i);
% Hp = c*Hphi/jk
  Hp = Ham(:,2,j)./(dk(j)*1i);
% [Hbeta, Hp, Hr] = [Hbeta, Hp, c*(Hpsi+Hbeta)/jk]
  Haq(:,1:3,j+1) = [Hbeta, Hp, (Ham(:,3,j)+Hbeta)./(dk(j)*1i)];
%
end
%
%
switch(ORDER)
%
  case 2
    % Add zero frequency value
    % c*Hy', c*Hphi', c*Hpsi'
    D1HAM0 = imag(Ham(:,:,1))/dk0;
    % c*Hbeta', c*Hp', c*Hr'
    D1H0 = imag(Haq(:,:,2))/dk0;
    %
    Haq(:,:,1) = [D1HAM0(:,1), D1HAM0(:,2), (D1HAM0(:, 3) + D1H0(:,1))];
    % exploit Halpha = Htheta for k=0
    % Haq(:,:,1) = [-real(Ham(:,3,1)), D1HAM0(:,2), (D1HAM0(:, 3) + D1H0(:,1))];
  case 4
    if (k(2)/k(1) == 2)
      D1HAM0 = (-imag(Ham(:,:,2)) + 8*imag(Ham(:,:,1)))/(6*dk0);
      D1H0 = (-imag(Haq(:,:,3)) + 8*imag(Haq(:,:,2)))/(6*dk0);
      Haq(:,:,1) = [D1HAM0(:,1), D1HAM0(:,2), (D1HAM0(:, 3) + D1H0(:,1))];
    % Haq(:,:,1) = [-real(Ham(:,3,1)), D1HAM0(:,2), (D1HAM0(:, 3) + D1H0(:,1))];
    else
      error('The ratio of the first two values of reduced frequencies must be 2 when ORDER=4.');
    end
  otherwise
    error('Wrong approximation order. Allowed values are 2 and 4.');
%
end
%
k = [0, k];
%
end


function HINT = interp_coeff(knew, Hsa, row, col, kstep)

    nk = length(knew);
    QHH = zeros(1,1,2*nk);
    QHH(1,1,1:2:nk*2) = Hsa(row,col,:);
    HINT = zeros(1,1,nk);
    [dummy, dummy, dummy, QHH] = spline_coefficients_vec(knew, QHH);
    kint = [0:kstep:knew(end)];
    for i=1:length(kint)
      HINT(1,1,i) = aero_interp_vec(QHH, 2, kint(i), knew);
    end

end