function test_miqp_suite()
% MIQP SOLVER COMPREHENSIVE TEST SUITE
% =====================================

fprintf('MIQP SOLVER COMPREHENSIVE TEST SUITE\n');
fprintf('=====================================\n');

% Run all test cases
test_case_1();
test_case_2();
test_case_3();
test_case_4();
test_case_5();
test_case_6();

% Manual verification for key test cases
fprintf('\n\nMANUAL VERIFICATIONS:\n');
fprintf('=====================\n');

% Manual verification for test case 1
manual_verification_test_case_1();

% Manual verification for test case 2
manual_verification_test_case_2();

end

function test_case_1()
% TEST CASE 1: Simple 2D problem
fprintf('=========================================\n');
fprintf('TEST CASE 1: Simple 2D problem\n');
fprintf('=========================================\n');

N = 2;

% Cost function: minimize 0.5*u'*Q*u + f'*u
Q = eye(N);
f = zeros(N,1);
objective.Q = Q;
objective.f = f;

% Equality constraint: u1 + u2 = 1
A = [1.0, 1.0];
w = 1.0;

% Bounds: if y=1 -> u in [0,1], if y=0 -> u in [-1,0]
u_underbar = [0.0; 0.0];
u_bar = [1.0; 1.0];

% Initial guesses
u_init = 0.5 * ones(N,1);
y_init = 0.5 * ones(N,1);
y_fixed = nan(N,1);

Aeq_on = true;

[u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar(1), u_bar(1), u_init, y_init, y_fixed, objective, Aeq_on);

if flag
    fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
            y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
    
    % Verify constraints
    Au = A * u_best;
    fprintf('Constraint A*u = %g (should be %g)\n', Au, w);
    
    for i = 1:N
        if y_best(i) == 1
            lb = u_underbar(i);
            ub = u_bar(i);
        else
            lb = -u_bar(i);
            ub = -u_underbar(i);
        end
        in_bounds = (u_best(i) >= lb-1e-6 && u_best(i) <= ub+1e-6);
        fprintf('u[%d] in [%g, %g]: %g %s\n', i, lb, ub, u_best(i), char(10003*in_bounds + 10007*(~in_bounds)));
    end
else
    fprintf('No feasible solution found.\n');
end
end

function test_case_2()
% TEST CASE 2: 3D problem with negative cost
fprintf('\n=========================================\n');
fprintf('TEST CASE 2: 3D problem with negative cost\n');
fprintf('=========================================\n');

N = 3;

Q = [4.0, 1.0, 0.5;
     1.0, 3.0, 0.2;
     0.5, 0.2, 2.0];
f = [-3.0; -2.0; -1.0];
objective.Q = Q;
objective.f = f;

A = [1.0, 2.0, 1.0];
w = 1.5;

u_underbar = [0.0; 0.0; 0.0];
u_bar = [1.0; 1.0; 1.0];

u_init = 0.3 * ones(N,1);
y_init = 0.5 * ones(N,1);
y_fixed = nan(N,1);

Aeq_on = true;

[u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar(1), u_bar(1), u_init, y_init, y_fixed, objective, Aeq_on);

if flag
    fprintf('SOLUTION: y = [%g, %g, %g], u = [%g, %g, %g], cost = %g\n', ...
            y_best(1), y_best(2), y_best(3), u_best(1), u_best(2), u_best(3), cost_best);
    
    Au = A * u_best;
    fprintf('Constraint A*u = %g (should be %g)\n', Au, w);
else
    fprintf('No feasible solution found.\n');
end
end

function test_case_3()
% TEST CASE 3: Some variables fixed
fprintf('\n=========================================\n');
fprintf('TEST CASE 3: Some variables fixed\n');
fprintf('=========================================\n');

N = 3;

Q = eye(N);
f = zeros(N,1);
objective.Q = Q;
objective.f = f;

A = [1.0, 1.0, 1.0];
w = 1.0;

u_underbar = [0.0; 0.0; 0.0];
u_bar = [1.0; 1.0; 1.0];

u_init = 0.5 * ones(N,1);
y_init = 0.5 * ones(N,1);

% Fix y3 = 1, others free
y_fixed = [nan; nan; 1.0];

Aeq_on = true;

[u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar(1), u_bar(1), u_init, y_init, y_fixed, objective, Aeq_on);

if flag
    fprintf('SOLUTION: y = [%g, %g, %g], u = [%g, %g, %g], cost = %g\n', ...
            y_best(1), y_best(2), y_best(3), u_best(1), u_best(2), u_best(3), cost_best);
    
    fixed_correctly = (y_best(3) == 1.0);
    fprintf('y[3] should be fixed to 1: %s\n', char(10003*fixed_correctly + 10007*(~fixed_correctly)));
else
    fprintf('No feasible solution found.\n');
end
end

function test_case_4()
% TEST CASE 4: No equality constraints
fprintf('\n=========================================\n');
fprintf('TEST CASE 4: No equality constraints\n');
fprintf('=========================================\n');

N = 2;

Q = eye(N);
f = zeros(N,1);
objective.Q = Q;
objective.f = f;

% No equality constraints
A = zeros(0, N);
w = zeros(0,1);

u_underbar = [0.0; 0.0];
u_bar = [1.0; 1.0];

u_init = 0.5 * ones(N,1);
y_init = 0.5 * ones(N,1);
y_fixed = nan(N,1);

Aeq_on = false; % ignore equality constraints

[u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar(1), u_bar(1), u_init, y_init, y_fixed, objective, Aeq_on);

if flag
    fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
            y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
    
    fprintf('Expected y = [0,0], u = [0,0], cost = 0\n');
else
    fprintf('No feasible solution found.\n');
end
end

function test_case_5()
% TEST CASE 5: Infeasible problem
fprintf('\n=========================================\n');
fprintf('TEST CASE 5: Infeasible problem\n');
fprintf('=========================================\n');

N = 2;

Q = eye(N);
f = zeros(N,1);
objective.Q = Q;
objective.f = f;

% Conflicting constraints
A = [1.0, 1.0;
     1.0, -1.0];
w = [1.0; 2.0]; % u1+u2=1 and u1-u2=2 -> impossible with bounds

u_underbar = [0.0; 0.0];
u_bar = [1.0; 1.0];

u_init = 0.5 * ones(N,1);
y_init = 0.5 * ones(N,1);
y_fixed = nan(N,1);

Aeq_on = true;

[u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar(1), u_bar(1), u_init, y_init, y_fixed, objective, Aeq_on);

if flag
    fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
            y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
else
    fprintf('Correctly detected infeasible problem ✓\n');
end
end

function test_case_6()
% TEST CASE 6: Asymmetric bounds
fprintf('\n=========================================\n');
fprintf('TEST CASE 6: Asymmetric bounds\n');
fprintf('=========================================\n');

N = 2;

Q = eye(N);
f = [1.0; -1.0]; % Mixed cost coefficients
objective.Q = Q;
objective.f = f;

A = [1.0, 1.0];
w = 0.5;

% Different bounds for different variables
u_underbar = [0.0; 0.0];
u_bar = [2.0; 1.0]; % u1 in [0,2] when y1=1, u2 in [0,1] when y2=1

u_init = 0.5 * ones(N,1);
y_init = 0.5 * ones(N,1);
y_fixed = nan(N,1);

Aeq_on = true;

[u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar(1), u_bar(1), u_init, y_init, y_fixed, objective, Aeq_on);

if flag
    fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
            y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
    
    Au = A * u_best;
    fprintf('Constraint A*u = %g (should be %g)\n', Au, w);
else
    fprintf('No feasible solution found.\n');
end
end

function manual_verification_test_case_1()
% Manual verification for test case 1
fprintf('\nTest Case 1 - MANUAL VERIFICATION:\n');

N = 2;
Q = eye(N);
f = zeros(N,1);
objective.Q = Q;
objective.f = f;
A = [1.0, 1.0];
w = 1.0;
u_underbar = [0.0; 0.0];
u_bar = [1.0; 1.0];
u_init = 0.5 * ones(N,1);
y_init = 0.5 * ones(N,1);
y_fixed = nan(N,1);
Aeq_on = true;

% Generate all combinations for N=2
y_combinations = {[0;0], [0;1], [1;0], [1;1]};

best_cost = inf;
best_u = [];
best_y = [];

for i = 1:length(y_combinations)
    y_test = y_combinations{i};
    
    % Solve leaf QP for fixed y
    lb_u = u_underbar .* y_test - u_bar .* (1 - y_test);
    ub_u = u_bar .* y_test - u_underbar .* (1 - y_test);
    
    u0 = max(lb_u, min(ub_u, u_init));
    
    options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
    
    if Aeq_on
        [u_test, fval, exitflag] = quadprog(Q, f, [], [], A, w, lb_u, ub_u, u0, options);
    else
        [u_test, fval, exitflag] = quadprog(Q, f, [], [], [], [], lb_u, ub_u, u0, options);
    end
    
    if exitflag == 1
        fprintf('  y = [%g, %g] -> cost = %g, u = [%g, %g]\n', ...
                y_test(1), y_test(2), fval, u_test(1), u_test(2));
        
        if fval < best_cost
            best_cost = fval;
            best_u = u_test;
            best_y = y_test;
        end
    end
end

if best_cost < inf
    fprintf('  BEST: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
            best_y(1), best_y(2), best_u(1), best_u(2), best_cost);
else
    fprintf('  No feasible integer solution found.\n');
end
end

function manual_verification_test_case_2()
% Manual verification for test case 2
fprintf('\nTest Case 2 - MANUAL VERIFICATION:\n');

N = 3;
Q = [4.0, 1.0, 0.5;
     1.0, 3.0, 0.2;
     0.5, 0.2, 2.0];
f = [-3.0; -2.0; -1.0];
objective.Q = Q;
objective.f = f;
A = [1.0, 2.0, 1.0];
w = 1.5;
u_underbar = [0.0; 0.0; 0.0];
u_bar = [1.0; 1.0; 1.0];
u_init = 0.3 * ones(N,1);
y_init = 0.5 * ones(N,1);
y_fixed = nan(N,1);
Aeq_on = true;

% Generate all combinations for N=3
y_combinations = {[0;0;0], [0;0;1], [0;1;0], [0;1;1], ...
                  [1;0;0], [1;0;1], [1;1;0], [1;1;1]};

best_cost = inf;
best_u = [];
best_y = [];

for i = 1:length(y_combinations)
    y_test = y_combinations{i};
    
    % Solve leaf QP for fixed y
    lb_u = u_underbar .* y_test - u_bar .* (1 - y_test);
    ub_u = u_bar .* y_test - u_underbar .* (1 - y_test);
    
    u0 = max(lb_u, min(ub_u, u_init));
    
    options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
    
    if Aeq_on
        [u_test, fval, exitflag] = quadprog(Q, f, [], [], A, w, lb_u, ub_u, u0, options);
    else
        [u_test, fval, exitflag] = quadprog(Q, f, [], [], [], [], lb_u, ub_u, u0, options);
    end
    
    if exitflag == 1
        fprintf('  y = [%g, %g, %g] -> cost = %g, u = [%g, %g, %g]\n', ...
                y_test(1), y_test(2), y_test(3), fval, u_test(1), u_test(2), u_test(3));
        
        if fval < best_cost
            best_cost = fval;
            best_u = u_test;
            best_y = y_test;
        end
    end
end

if best_cost < inf
    fprintf('  BEST: y = [%g, %g, %g], u = [%g, %g, %g], cost = %g\n', ...
            best_y(1), best_y(2), best_y(3), best_u(1), best_u(2), best_u(3), best_cost);
else
    fprintf('  No feasible integer solution found.\n');
end
end