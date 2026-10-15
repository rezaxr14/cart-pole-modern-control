% --- Part 4: Observability Investigation ---

% Case A: Measure only Cart Position (x)
C_x = [1 0 0 0];
Om_x = obsv(A, C_x)
rank_x = rank(Om_x);
fprintf('Case A (Measure x) Rank: %d -> %s\n', ...
    rank_x, string(original_rank_check(rank_x)));

% Case B: Measure only Pole Angle (theta)
C_theta = [0 0 1 0];
Om_theta = obsv(A, C_theta)
rank_theta = rank(Om_theta);
fprintf('Case B (Measure theta) Rank: %d -> %s\n', ...
    rank_theta, string(original_rank_check(rank_theta)));

% Helper function for display
function s = original_rank_check(r)
    if r == 4
        s = 'Observable';
    else
        s = 'Unobservable';
    end
end