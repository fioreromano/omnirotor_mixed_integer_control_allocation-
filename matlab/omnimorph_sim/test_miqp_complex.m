function test_miqp_solver()
    fprintf('=========================================\n');
    fprintf('        MIQP Solver Test Suite           \n');
    fprintf('=========================================\n');

    % Esegui ogni test
    % test_case_1();   % QP semplice in 2D
    % test_case_2();   % Costi negativi, 3D
    % test_case_3();   % Alcune variabili fissate
    % test_case_4();   % Nessun vincolo di uguaglianza
    % test_case_5();   % Problema infeasible

    test_random_complex();

    fprintf('\n=========================================\n');
    fprintf('        Tutti i test completati          \n');
    fprintf('=========================================\n');
end

function test_case_1()
    fprintf('=========================================\n');
    fprintf('TEST CASE 1: Simple 2D problem\n');
    fprintf('=========================================\n');
    
    N = 2;

    % Cost function: minimize 0.5*u'*Q*u + f'*u
    Q = eye(N);
    f = zeros(N, 1);
    objective = struct('Q', Q, 'f', f);
    
    % Equality constraint: u1 + u2 = 1
    A = [1.0, 1.0];
    w = 1.0;

    % Bounds
    u_underbar = 0.0; 
    u_bar = 1.0;

    % Initial guesses
    u_init = 0.5 * ones(N, 1);
    y_init = 0.5 * ones(N, 1);
    y_fixed = nan(N, 1);

    % Chiamata al solver MIQP
    [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar, u_bar, ...
                                                              u_init, y_init, y_fixed, objective, true);

if flag
        fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
                y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
        
        % Verify constraints
        Au = A * u_best;
        fprintf('Constraint A*u = %g (should be %g)\n', Au, w);
        
        for i = 1:N
            if y_best(i) == 1.0
                lb = u_underbar;
                ub = u_bar;
            else
                lb = -u_bar;
                ub = -u_underbar;
            end
            
            if u_best(i) >= lb-1e-6 && u_best(i) <= ub+1e-6
                check = '✓';
            else
                check = '✗';
            end
            fprintf('u[%d] in [%g, %g]: %g %s\n', i-1, lb, ub, u_best(i), check);
        end
    else
        fprintf('No feasible solution found.\n');
    end
end

function test_case_2()
    fprintf('\n=========================================\n');
    fprintf('TEST CASE 2: 3D problem with negative cost\n');
    fprintf('=========================================\n');
    
    N = 3;

    Q = [4.0, 1.0, 0.5;
         1.0, 3.0, 0.2;
         0.5, 0.2, 2.0];
    f = [-3.0; -2.0; -1.0];
    objective = struct('Q', Q, 'f', f);

    A = [1.0, 2.0, 1.0];
    w = 1.5;

    u_underbar = 0.0;
    u_bar = 1.0;

    u_init = 0.3 * ones(N, 1);
    y_init = 0.5 * ones(N, 1);
    y_fixed = nan(N, 1);

    [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar, u_bar, ...
                                                              u_init, y_init, y_fixed, objective, true);

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
    fprintf('\n=========================================\n');
    fprintf('TEST CASE 3: Some variables fixed\n');
    fprintf('=========================================\n');
    
    N = 3;

    Q = eye(N);
    f = zeros(N, 1);
    objective = struct('Q', Q, 'f', f);

    A = [1.0, 1.0, 1.0];
    w = 1.0;

    u_underbar = 0.0;
    u_bar = 1.0;

    u_init = 0.5 * ones(N, 1);
    y_init = 0.5 * ones(N, 1);
    
    % Fix y3 = 1, others free
    y_fixed = [nan; nan; 1.0];

    [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar, u_bar, ...
                                                              u_init, y_init, y_fixed, objective, true);

    if flag
        fprintf('SOLUTION: y = [%g, %g, %g], u = [%g, %g, %g], cost = %g\n', ...
                y_best(1), y_best(2), y_best(3), u_best(1), u_best(2), u_best(3), cost_best);
        
        if y_best(3) == 1.0
            check = '✓';
        else
            check = '✗';
        end
        fprintf('y[2] should be fixed to 1: %s\n', check);
    else
        fprintf('No feasible solution found.\n');
    end
end

function test_case_4()
    fprintf('\n=========================================\n');
    fprintf('TEST CASE 4: No equality constraints\n');
    fprintf('=========================================\n');
    
    N = 2;

    Q = eye(N);
    f = zeros(N, 1);
    objective = struct('Q', Q, 'f', f);

    % No equality constraints
    A = zeros(0, N);
    w = zeros(0, 1);

    u_underbar = 0.0; 
    u_bar = 1.0;

    u_init = 0.5 * ones(N, 1);
    y_init = 0.5 * ones(N, 1);
    y_fixed = nan(N, 1);

    % Aeq_on = false to ignore equality constraints
    [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar, u_bar, ...
                                                              u_init, y_init, y_fixed, objective, false);

    if flag
        fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
                y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
        
        fprintf('Expected y = [0,0], u = [0,0], cost = 0\n');
    else
        fprintf('No feasible solution found.\n');
    end
end

function test_case_5()
    fprintf('\n=========================================\n');
    fprintf('TEST CASE 5: Infeasible problem\n');
    fprintf('=========================================\n');
    
    N = 2;

    Q = eye(N);
    f = zeros(N, 1);
    objective = struct('Q', Q, 'f', f);

    % Conflicting constraints
    A = [1.0, 1.0;
         1.0, -1.0];
    w = [1.0; 2.0];  % u1+u2=1 and u1-u2=2 -> impossible with bounds

    u_underbar = 0.0;
    u_bar = 1.0;

    u_init = 0.5 * ones(N, 1);
    y_init = 0.5 * ones(N, 1);
    y_fixed = nan(N, 1);

    [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar, u_bar, ...
                                                              u_init, y_init, y_fixed, objective, true);

    if flag
        fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
                y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
    else
        fprintf('Correctly detected infeasible problem ✓\n');
    end
end

function test_case_6()
    fprintf('\n=========================================\n');
    fprintf('TEST CASE 6: Asymmetric bounds\n');
    fprintf('=========================================\n');
    
    N = 2;

    Q = eye(N);
    f = [1.0; -1.0];  % Mixed cost coefficients
    objective = struct('Q', Q, 'f', f);

    A = [1.0, 1.0];
    w = 0.5;

    % Different bounds for different variables
    u_underbar = 0.0;
    u_bar = [2.0; 1.0];  % u1 in [0,2] when y1=1, u2 in [0,1] when y2=1

    u_init = 0.5 * ones(N, 1);
    y_init = 0.5 * ones(N, 1);
    y_fixed = nan(N, 1);

    [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w, A, Q, u_underbar, u_bar, ...
                                                              u_init, y_init, y_fixed, objective, true);

    if flag
        fprintf('SOLUTION: y = [%g, %g], u = [%g, %g], cost = %g\n', ...
                y_best(1), y_best(2), u_best(1), u_best(2), cost_best);
        
        Au = A * u_best;
        fprintf('Constraint A*u = %g (should be %g)\n', Au, w);
    else
        fprintf('No feasible solution found.\n');
    end
end
function test_random_complex()
    fprintf('\n=========================================\n');
    fprintf('TEST CASE: Random Complex Problem (Improved)\n');
    fprintf('=========================================\n');
    
    % Parametri
    N = 8;   % Numero di variabili
    M = 3;   % Numero di vincoli

    % Matrice A con valori esatti forniti
    A = [-0.93306,  -0.155027,   0.273117,  -0.950152,  -0.364279,   0.521343,   0.129304,    -0.5623;
         -0.340072,   -0.58747,   0.727245,  -0.270015,  -0.728543,  -0.836655,   0.486541, -0.0912082;
          0.381271,  -0.499743,  -0.396689,   0.530762,  -0.786509,   0.101912,    0.96352,   0.037308];
    
    
    % u_feasible con valori esatti forniti
    u_feasible = [0.147683; 0.110462; 0.457093; 0.257748; 0.592256; 0.167432; 0.450467; 0.659196];
    
    % Genera w compatibile con la soluzione fattibile
    w = A * u_feasible;

    % Matrice Q positiva definita
    Q = eye(N); % Usa identità per semplicità

    f = zeros(N, 1); % Zero costi lineari per semplicità
    objective = struct('Q', Q, 'f', f);

    % Limiti
    u_underbar = 0.5 ;
    u_bar = 2.0;

    % Punti iniziali
    u_init = ones(N, 1);
    y_init = 0.5 * ones(N, 1);
    y_fixed = nan(N, 1);

    Aeq_on = true;

    % Misura il tempo di esecuzione
    start_time = tic;
    
    fprintf('Risoluzione MIQP (N=%d, M=%d)...\n', N, M);
    [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(...
        w, A, Q, u_underbar, u_bar, u_init, y_init, y_fixed, objective, Aeq_on, 1);

    end_time = toc(start_time);

    % Visualizza risultati
    if flag
        fprintf('✅ Soluzione trovata\n');
        fprintf('Tempo di esecuzione: %.2f ms\n', end_time * 1000);
        fprintf('Costo ottimo: %g\n', cost_best);
        fprintf('Numero di y=1: %d su %d\n', sum(y_best > 0.5), N);
        fprintf('y_best: [%s]\n', sprintf('%g ', y_best));
        fprintf('u_best: [%s]\n', sprintf('%g ', u_best));

        % Controlla vincoli
        residuo = A * u_best - w;
        fprintf('Norma residuo A*u - w: %g\n', norm(residuo));
        
        % Verifica ammissibilità dei bounds
        feasible = true;
        for i = 1:N
            if y_best(i) == 1.0
                lb = u_underbar;
                ub = u_bar;
            else
                lb = -u_bar;
                ub = -u_underbar;
            end
            
            if u_best(i) < lb - 1e-6 || u_best(i) > ub + 1e-6
                feasible = false;
                fprintf('✗ Vincolo violato per u[%d]: %g non in [%g, %g]\n', ...
                        i-1, u_best(i), lb, ub);
            end
        end
        
        if feasible
            fprintf('✓ Tutti i vincoli rispettati\n');
        end
        
        % Verifica vincoli fissi
        fprintf('Verifica vincolo A*u = w:\n');
        fprintf('  A*u_best = [%s]\n', sprintf('%g ', A * u_best));
        fprintf('  w        = [%s]\n', sprintf('%g ', w));
        
    else
        fprintf('❌ Nessuna soluzione trovata\n');
        fprintf('Tempo di esecuzione: %.2f ms\n', end_time * 1000);
        
        % Prova con Aeq_on = false per vedere se è un problema di uguaglianza
        fprintf('\nProvando senza vincoli di uguaglianza...\n');
        
        [u_best_noeq, y_best_noeq, cost_best_noeq, flag_noeq] = solve_miqp_bb_stack_2(...
            w, A, Q, u_underbar, u_bar, u_init, y_init, y_fixed, objective, false);
            
        if flag_noeq
            fprintf('✅ Soluzione trovata SENZA vincoli di uguaglianza!\n');
            fprintf('Costo: %g\n', cost_best_noeq);
            fprintf('y: [%s]\n', sprintf('%g ', y_best_noeq));
            fprintf('u: [%s]\n', sprintf('%g ', u_best_noeq));
            
            % Verifica quanto si discosta dal vincolo
            residuo_noeq = A * u_best_noeq - w;
            fprintf('Norma residuo A*u - w (senza vincolo): %g\n', norm(residuo_noeq));
        else
            fprintf('❌ Nessuna soluzione anche senza vincoli di uguaglianza\n');
        end
    end
end
% Funzione helper per generare matrici definite positive
function Q = generatePositiveDefiniteMatrix(N)
    R = randn(N, N);
    Q = R' * R + eye(N) * N;
end