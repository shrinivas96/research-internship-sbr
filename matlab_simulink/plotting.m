figure
theta_array = squeeze(out.thetaplot.Data(1, 1, :));
plot(out.thetaplot.Time, theta_array)
grid on
% axis equal
%title('theta Plot')
ylabel('theta (rad)')
xlabel('time (s)')