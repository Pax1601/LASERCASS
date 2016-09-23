
function str = run_optim_Tcap_spar_10(niter, bk_sol, Zs, tbs, NSPAR, esw, msl, FS, M, Nstr, Aeq, tskin, output)
%
NITN = 4; KTRIAL = 10; PERT_AMPL = 10.0;
%
Nsec = length(FS);
%
save(output)
if (NITN>0)
  CONV_HISTO = zeros(Nsec, NITN+1);
end
if (niter==1)
%
  XBK_SP = ones(8,1); 
  XBK_SP(1) = tbs(1)/115; % maximum web allowable
  Acap = (abs(M(1))/2)/(tbs(1)*msl);

  Acap = (abs(M(1))/2)/(tbs(1)* 5.063441626883254e+08);
  XBK_SP(2) = sqrt(50*Acap/5.1); % assume B1=B2, T1=T2=0.1B1
  XBK_SP(3) = XBK_SP(2)/10; 
  XBK_SP(4) = XBK_SP(2)/5; % B2=B1
  XBK_SP(5) = XBK_SP(2)/100; % T2=T1
  XBK_SP(6) = XBK_SP(1); % BU=B1
  XBK_SP(7) = XBK_SP(1); % TU=TWEB
  XBK_SP(8) = tbs(1)/4;    % SP=H/2
else
  XBK_SP(1) =  bk_sol.tw(1);
  XBK_SP(2) =  bk_sol.B1_cap(1);
  XBK_SP(3) =  bk_sol.T1_cap(1);
  XBK_SP(4) =  bk_sol.B2_cap(1);
  XBK_SP(5) =  bk_sol.T2_cap(1);
  XBK_SP(6) =  bk_sol.Bu_stf(1);
  XBK_SP(7) =  bk_sol.tu_stf(1);
  XBK_SP(8) =  bk_sol.du_stf(1);
end
%
str.Acap     = zeros(Nsec,1);
%
str.tw       = zeros(Nsec,1);
str.B1_cap   = zeros(Nsec,1);
str.T1_cap   = zeros(Nsec,1);
str.B2_cap   = zeros(Nsec,1);
str.T2_cap   = zeros(Nsec,1);
str.Bu_stf   = zeros(Nsec,1);
str.tu_stf   = zeros(Nsec,1);
str.du_stf   = zeros(Nsec,1);
%
for i=1:Nsec
fprintf('\nSection %d', i);
%-------------------------------------------------------------------------------
%  XBK_SP = ones(8,1); 
%  XBK_SP(1) = tbs(i)/115; % maximum web allowable
%  Acap = (abs(M(i))/2)/(tbs(1)*msl);

%  Acap = (abs(M(i))/2)/(tbs(i)* 5.063441626883254e+08);
%  XBK_SP(2) = sqrt(50*Acap/5.1); % assume B1=B2, T1=T2=0.1B1
%  XBK_SP(3) = XBK_SP(2)/10; 
%  XBK_SP(4) = XBK_SP(2)/5; % B2=B1
%  XBK_SP(5) = XBK_SP(2)/100; % T2=T1
%  XBK_SP(6) = XBK_SP(1); % BU=B1
%  XBK_SP(7) = XBK_SP(1); % TU=TWEB
%  XBK_SP(8) = tbs(i)/4;    % SP=H/2
  [SOL, flag, fun, CUNS] = optim_Tcap_spar_10(Zs(i), tbs(i), Nstr(i), Aeq(i), tskin(i), esw, msl, FS(i)/NSPAR, M(i)/NSPAR, [], XBK_SP);
  if (~isempty(CUNS))
    fprintf('\n\n### Warning: optimization failed.');
    fprintf('\n    Constraints will be relaxed. ');
    SOL = XBK_SP;
  end
  K=1;
  while ( (~isempty(CUNS)) && (K<=KTRIAL) )
    fprintf('\n\nTrial %d: ', K);
    CEVAL = CUNS; 
    [SOL, flag, fun, CUNS] = optim_Tcap_spar_10(Zs(i), tbs(i), Nstr(i), Aeq(i), tskin(i), esw, msl, FS(i)/NSPAR, M(i)/NSPAR, CEVAL, SOL);
    [SOL, flag, fun, CUNS] = optim_Tcap_spar_10(Zs(i), tbs(i), Nstr(i), Aeq(i), tskin(i), esw, msl, FS(i)/NSPAR, M(i)/NSPAR, [], SOL);
    K = K + 1;
    if (isempty(CUNS))
      fprintf('feasible.');
    else
      SOL = PERT_AMPL .* SOL;
      fprintf('NOT feasible.');
    end
  end
%
  if ( (~isempty(CUNS)) && (K==11) )
    error('Unable to find an initial feasible solution.');
  end
%
  XBK_SP = SOL;
% perturbation
%
  if (NITN>0)
%
    XL(1) = tbs(i)/1500;
    XL(2) = 0.5*0.0254;
    XL(3) = 0.04*0.0254;
    XL(4) = 0.5*0.0254;
    XL(5) = 0.04*0.0254;
    XL(6) = 0.5*0.0254;
    XL(7) = 0.04*0.0254;
    XL(8) = tbs(i);
%
    coeff = [];
    for NIT = 2:NITN+1
      coeff = [coeff, 1+0.1*NIT];
    end    
%
    SUB_BK      = zeros(8, NITN+1);
    SUB_BK(:,1) = XBK_SP;
    OBJ_BK      = zeros(1, NITN+1);    
    OBJ_BK(1) = fun;
  %
    for NIT = 1:NITN
  %
      fprintf('\n\tPerturbation %d: ', NIT);
  %
      XPERT = XL .* coeff(NIT);
      XPERT(end) = XL(end) / coeff(NIT);
      [SOL, flag, fun, CUNS] = optim_Tcap_spar_10(Zs(i), tbs(i), Nstr(i), Aeq(i), tskin(i), esw, msl, FS(i)/NSPAR, M(i)/NSPAR, [], XPERT);
      if (flag>0)
        SUB_BK(:,NIT+1)   = SOL;
        OBJ_BK(NIT+1)     = fun;
        fprintf('feasible.');
      else
        OBJ_BK(NIT+1)     = Inf;
        fprintf('NOT feasible.');
      end
    end
    [~, index] = min(OBJ_BK);
    SOL = SUB_BK(:,index);
    CONV_HISTO(i, :) = OBJ_BK;
  end
%
  XBK_SP = SOL;
  str.tw(i)       = SOL(1);
  str.B1_cap(i)   = SOL(2);
  str.T1_cap(i)   = SOL(3);
  str.B2_cap(i)   = SOL(4);
  str.T2_cap(i)   = SOL(5);
  str.Bu_stf(i)   = SOL(6);
  str.tu_stf(i)   = SOL(7);
  str.du_stf(i)   = SOL(8);
  str.Acap(i) = str.B1_cap(i) * str.T1_cap(i) + str.B2_cap(i) * str.T2_cap(i);
  str.histo = CONV_HISTO;
%
end
%
end