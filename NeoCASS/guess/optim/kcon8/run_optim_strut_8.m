function [str, strw] = run_optim_strut_8(niter, bksk, tbs, esw, msl, Nax, FS, M, Mt, Lstrut, output)
%
  NITN = 4; KTRIAL = 10; PERT_AMPL = 10.0;
%
  Nsec = length(FS);
%
  save(output);
  if (NITN>0)
    CONV_HISTO = zeros(Nsec, NITN+1);
    XL = 0.001;
  end
  if (niter==1)
    XBK_SK = tbs(1); 
  else
    XBK_SK = bksk.tskin(1); 
  end
  str.tskin = zeros(Nsec,1);
%
  for i=1:Nsec
  fprintf('\nSection %d', i);
%
    [SOL, flag, fun, CUNS] = optim_strut_8(tbs(i), Lstrut, esw, msl, Nax(i), 0.0, FS(i), M(i), Mt(i), [], XBK_SK);
    if (~isempty(CUNS))
      fprintf('\n\n### Warning: optimization failed.');
      fprintf('\n    Constraints will be relaxed. ');
      SOL = XBK_SK;
    end
    K=1;
    while ( (~isempty(CUNS)) && (K<=KTRIAL) )
      fprintf('\n\n\t\tTrial %d: \n', K);
      CEVAL = CUNS; 
      [SOL, flag, fun, CUNS] = optim_strut_8(tbs(i), Lstrut, esw, msl, Nax(i), ...
                    0.0, FS(i), M(i), Mt(i), CEVAL, SOL);
      [SOL, flag, fun, CUNS] = optim_strut_8(tbs(i), Lstrut, esw, msl, Nax(i),...
                    0.0, FS(i), M(i), Mt(i), [], SOL);
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
      XBK_SK = PERT_AMPL * XBK_SK;
      error('Unable to find an initial feasible solution. Try raising strut thickness.');
    end
    XBK_SK = SOL;
%           

    if (NITN>0)
      
      coeff = [];
      for NIT = 2:NITN+1
        coeff = [coeff, 1+0.1*NIT];
      end    
  %
      SUB_BK      = zeros(1, NITN+1);
      SUB_BK(1,1) = XBK_SK;
      OBJ_BK      = zeros(1, NITN+1);    
      OBJ_BK(1) = fun;

      for NIT = 1:NITN
        fprintf('\n\tPerturbation %d: ', NIT);
        XPERT = XL * coeff(NIT);
        [SOL, flag, fun, CUNS] = optim_strut_8(tbs(i), Lstrut, esw, msl, Nax(i),...
                      0.0, FS(i), M(i), Mt(i), [], XPERT);
        if (flag>0)
          SUB_BK(1,NIT+1)   = SOL;
          OBJ_BK(NIT+1)     = fun;
          fprintf('feasible.');
        else
          OBJ_BK(NIT+1)     = Inf;
          fprintf('NOT feasible.');
        end
      end
      [~, index] = min(OBJ_BK);
      SOL = SUB_BK(1,index);
      CONV_HISTO(i, :) = OBJ_BK;
    end
    XBK_SK = SOL;
    str.tskin(i) = SOL(1);
    str.histo    = CONV_HISTO;

end
%
end