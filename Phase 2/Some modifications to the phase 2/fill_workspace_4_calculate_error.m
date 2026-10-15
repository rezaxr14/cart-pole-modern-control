% Extract data from the timeseries
e_data_reduced = out.reduced_order_error.Data; % This is a Matrix of size [N_samples x 4]
e_data_full_oreder = out.full_order_error.Data; % This is a Matrix of size [N_samples x 4]

% 1. L2 Norm (Euclidean norm of the error vector over time)
% We usually calculate the norm at each time step, then find the average or total
l2_norm_per_step_full = sqrt(sum(e_data_full_oreder.^2, 2)); 
total_l2_norm_full = norm(l2_norm_per_step_full); % Cumulative energy

% 2. Linf Norm (Maximum absolute error across all states and all time)
linf_norm_full = max(abs(e_data_full_oreder), [], 'all');

fprintf('The L2 Norm of the full oreder error is: %.4f\n', total_l2_norm_full);
fprintf('The Linf Norm of the full oreder error is: %.4f\n', linf_norm_full);



% 1. L2 Norm (Euclidean norm of the error vector over time)
% We usually calculate the norm at each time step, then find the average or total
l2_norm_per_step_reduced = sqrt(sum(e_data_reduced.^2, 2)); 
total_l2_norm_reduced = norm(l2_norm_per_step_reduced); % Cumulative energy

% 2. Linf Norm (Maximum absolute error across all states and all time)
linf_norm_reduced = max(abs(e_data_reduced), [], 'all');

fprintf('The L2 Norm of the reduced oreder error is: %.4f\n', total_l2_norm_reduced);
fprintf('The Linf Norm of the reduced oreder error is: %.4f\n', linf_norm_reduced);