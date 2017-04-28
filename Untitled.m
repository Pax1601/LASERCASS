figure
surf(peaks)
axis vis3d off
for x = -200:5:200
    view([x 0])
    drawnow
end