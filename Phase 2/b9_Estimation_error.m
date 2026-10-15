
error_x = x_real - x_hat;

figure('Name', 'Estimation Error Decay', 'Position', [100, 100, 800, 400]);

% رسم خطای هر ۴ متغیر حالت
plot(t_out, error_x, 'LineWidth', 1.5);
title('Estimation Error Decay (Convergence of Observer)');
xlabel('Time (sec)');
ylabel('Error (x - x_{hat})');
legend('Error in Pos', 'Error in Vel', 'Error in Angle', 'Error in Ang. Vel');
grid on;