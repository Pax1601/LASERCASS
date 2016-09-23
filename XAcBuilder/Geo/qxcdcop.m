function chord=qxcdcop(spanin,spanmtrx,rcrd,tapmtrx,z)
%CHDCOMP This computes the local chord length at given span station
%        compute chord on actual planform geometry
%============================================================================
if spanin==spanmtrx(1) | spanin==spanmtrx(1)+spanmtrx(2)
   correct=-1;% correction to heavyside result
else
   correct=0;% no correction to heavyside required
end
% identify the taper ratio for given wing segment
taprs=tapmtrx(correct+1+qxheavy(spanin,spanmtrx(1),1)+ ...
              qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1),z);
% identify the local span length for given wing segment           
spanc=spanmtrx(correct+1+qxheavy(spanin,spanmtrx(1),1)+ ...
               qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1));
% additional corrections before interpolation is executed            
if spanin>spanmtrx(1) & spanin<spanmtrx(1)+spanmtrx(2)
   spanin=spanin-spanmtrx(1);% correct input span to local datum
   taprs=taprs/tapmtrx(1,z);% correct identified taper to local datum
   rcrd=rcrd*tapmtrx(1,z);% correct for local chord datum   
elseif spanin>=spanmtrx(1)+spanmtrx(2)   
   spanin=spanin-spanmtrx(1)-spanmtrx(2);% correct input span to local datum
   taprs=taprs/tapmtrx(2,z);% correct identified taper to local datum
   rcrd=rcrd*tapmtrx(2,z);% correct for local chord datum   
end
chord=rcrd*(1-(1-taprs)*spanin/spanc);% computed chord result
return