figure
theta_array = squeeze(out.theta_fb.Data);
plot(out.theta_fb.Time, theta_array, "LineWidth", 3)
grid on
% axis equal
%title('theta Plot', 'FontSize', 20)
ylabel('theta (rad)', 'FontSize', 20)
xlabel('time (s)', 'FontSize', 20)
set(gca, 'FontSize', 20)