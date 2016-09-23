function ok = draw_wbox(foil, WING, frac)
%
ok = 1;
%
fid = 1;
%
figure(1); close; figure(1); hold on; axis equal;
naer = size(foil,2)-1;
% 
for k=1:naer+1
  plot3(foil{k}(:,1),foil{k}(:,2),foil{k}(:,3),'-');
end
for k=1:naer
  offset = (k-1)*4 + 1;
  plot3(WING(1,offset:offset+3),WING(2,offset:offset+3),WING(3,offset:offset+3),'-');
end
cont = 2;
p = zeros(3,2*(naer+1));
for k=1:naer
  offset = (k-1)*4 + 1;
  v1 = WING(:,offset+2) - WING(:,offset+1);
  cont = cont+1;
  p(:,cont) = WING(:,offset+1) + v1 * frac(k+1,1);
  cont = cont+1;
  p(:,cont) = WING(:,offset+1) + v1 * frac(k+1,2);
end
v1 = WING(:,4) - WING(:,1);
cont = cont+1;
p(:,1) = WING(:,1) + v1 * frac(1,1);
cont = cont+1;
p(:,2) = WING(:,1) + v1 * frac(1,2);
%
front_spar = p(:,1:2:end);
rear_spar  = p(:,2:2:end);
%
plot3(p(1,1:2:end),p(2,1:2:end),p(3,1:2:end),'-ro')
plot3(p(1,2:2:end),p(2,2:2:end),p(3,2:2:end),'-ro')
%
% check front spar
%
%for k=2:naer+1
%  if (front_spar(1,k)<front_spar(1,k-1))
%    ok = 0;
%    fprintf(fid,'\n### Warning: wing box section increasing along wing span. Check front spar fraction distribution.')
%    break
%  end
%end
%for k=2:naer+1
%  if (rear_spar(1,k)>rear_spar(1,k-1))
%    ok = 0;
%    fprintf(fid,'\n### Warning: wing box section increasing along wing span. Check rear spar fraction distribution.')
%    break
%  end
%end
