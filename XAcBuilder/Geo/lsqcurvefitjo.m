function [par,resnrm]=lsqcurvefitjo(funpx,par0,xdat0,ydat0,xmin,xmax)
%[CROSXCF,resnorm]=lsqcurvefit('qxfours',crosxc0,thedat,raddat,0,2*pi);
% assume find p to minimize sum((fun(p,xj)-yj)^2)
% fun must be affine in p: fun(p1,x)-fun(p2,x) = C(p1-p2)
par   = par0(:);
xdat = xdat0(:);
ydat = ydat0(:);
[n,dum]  = size(par);
[m,dum]=size(ydat);
E =eye(n);
eval(['f0 = ',funpx,'(zeros(n,1),xdat);']);
A = zeros(m,n);
for k = 1:n
    eval(['A(:,k) = ',funpx,'(E(:,k),xdat)-f0;']);
end
par = A\(ydat-f0);
resnrm = norm(ydat-f0-A*par,2);
par = reshape(par,size(par0));
