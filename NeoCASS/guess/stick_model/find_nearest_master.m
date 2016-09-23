function [ind, distmax] = find_nearest_master(SLAVE_COORD, MASTER_COORD)
%         find nearest point
  distmax = realmax; nsec = length(MASTER_COORD); ind = nsec;
  for (j=1:nsec)
    dist = norm(MASTER_COORD(:,j) - SLAVE_COORD);
    if (dist<distmax)
      distmax = dist; ind = j;
    end
  end
end
