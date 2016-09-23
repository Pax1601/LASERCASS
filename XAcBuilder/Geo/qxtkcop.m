function comp3=qxtkcop(spanin,spanmtrx,thkmtr)
%THKCOMP  this computes the wing thickness distribution for any span 
% identify the matrix coefficients for the computation
if spanin==spanmtrx(1) %| spanin==spanmtrx(1)+spanmtrx(2)
   correct=-1;% correction to heavyside result
else
   correct=0;% no correction to heavyside required
end
tthkc=thkmtr(1,correct+1+qxheavy(spanin,spanmtrx(1),1)+ ...
             qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1));% thickness taper ratio
rthkc=thkmtr(2,correct+1+qxheavy(spanin,spanmtrx(1),1)+ ...
             qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1));% datum thickness
spanc=spanmtrx(correct+1+qxheavy(spanin,spanmtrx(1),1)+ ...
               qxheavy(spanin,spanmtrx(1)+spanmtrx(2),1));
% additional corrections before interpolation is executed            
if spanin>spanmtrx(1) & spanin<spanmtrx(1)+spanmtrx(2)
   spanin=spanin-spanmtrx(1);% correct input span to local datum
elseif spanin>=spanmtrx(1)+spanmtrx(2)   
   spanin=spanin-spanmtrx(1)-spanmtrx(2);% correct input span to local datum
end
comp3=rthkc*(1-(1-tthkc)*spanin/spanc);% computed thickness result
return