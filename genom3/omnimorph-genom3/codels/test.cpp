#include <iostream>
#include "miqp_solver.h"
#include <vector>
#include <random>
#include <limits>
#include <cmath>
#include <cassert>
#include <chrono>

using namespace Eigen;
using std::cout; using std::endl;

using namespace miqp;

/*void test_case_1() {
    std::cout << "=========================================\n";
    std::cout << "TEST CASE 1: Simple 2D problem\n";
    std::cout << "=========================================\n";
    
    int N = 2;

    // Cost function: minimize 0.5*u'*Q*u + f'*u
    MatrixXd Q = MatrixXd::Identity(N, N);
    VectorXd f = VectorXd::Zero(N);
    Objective obj{Q, f};

    // Equality constraint: u1 + u2 = 1
    MatrixXd A(1, N);
    A << 1.0, 1.0;
    VectorXd w(1);
    w << 1.0;

    // Bounds: if y=1 -> u in [0,1], if y=0 -> u in [-1,0]
    VectorXd u_underbar(N), u_bar(N);
    u_underbar << 0.0, 0.0;
    u_bar << 1.0, 1.0;

    // Initial guesses
    VectorXd u_init = VectorXd::Constant(N, 0.5);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    VectorXd y_fixed = VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());

    MIQPSolver solver;    
    MIQPResult res = solver.solveMIQP(w, A, obj, u_underbar, u_bar, 
                                     u_init, y_init, y_fixed, true);

    if (res.flag) {
        std::cout << "SOLUTION: y = " << res.y_best.transpose() 
                  << ", u = " << res.u_best.transpose() 
                  << ", cost = " << res.cost_best << "\n";
        
        // Verify constraints
        VectorXd Au = A * res.u_best;
        std::cout << "Constraint A*u = " << Au(0) << " (should be " << w(0) << ")\n";
        
        for (int i = 0; i < N; ++i) {
            double lb = (res.y_best(i) == 1.0) ? u_underbar(i) : -u_bar(i);
            double ub = (res.y_best(i) == 1.0) ? u_bar(i) : -u_underbar(i);
            std::cout << "u[" << i << "] in [" << lb << ", " << ub << "]: " 
                      << res.u_best(i) << " " 
                      << ((res.u_best(i) >= lb-1e-6 && res.u_best(i) <= ub+1e-6) ? "✓" : "✗") << "\n";
        }
    } else {
        std::cout << "No feasible solution found.\n";
    }
}

void test_case_2() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE 2: 3D problem with negative cost\n";
    std::cout << "=========================================\n";
    
    int N = 3;

    MatrixXd Q(N, N);
    Q << 4.0, 1.0, 0.5,
         1.0, 3.0, 0.2,
         0.5, 0.2, 2.0;
    VectorXd f(N);
    f << -3.0, -2.0, -1.0;
    Objective obj{Q, f};

    MatrixXd A(1, N);
    A << 1.0, 2.0, 1.0;
    VectorXd w(1);
    w << 1.5;

    VectorXd u_underbar(N), u_bar(N);
    u_underbar << 0.0, 0.0, 0.0;
    u_bar << 1.0, 1.0, 1.0;

    VectorXd u_init = VectorXd::Constant(N, 0.3);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    VectorXd y_fixed = VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());

    MIQPSolver solver;    
    MIQPResult res = solver.solveMIQP(w, A, obj, u_underbar, u_bar, 
                                     u_init, y_init, y_fixed, true);

    if (res.flag) {
        std::cout << "SOLUTION: y = " << res.y_best.transpose() 
                  << ", u = " << res.u_best.transpose() 
                  << ", cost = " << res.cost_best << "\n";
        
        VectorXd Au = A * res.u_best;
        std::cout << "Constraint A*u = " << Au(0) << " (should be " << w(0) << ")\n";
    } else {
        std::cout << "No feasible solution found.\n";
    }
}

void test_case_3() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE 3: Some variables fixed\n";
    std::cout << "=========================================\n";
    
    int N = 3;

    MatrixXd Q = MatrixXd::Identity(N, N);
    VectorXd f = VectorXd::Zero(N);
    Objective obj{Q, f};

    MatrixXd A(1, N);
    A << 1.0, 1.0, 1.0;
    VectorXd w(1);
    w << 1.0;

    VectorXd u_underbar(N), u_bar(N);
    u_underbar << 0.0, 0.0, 0.0;
    u_bar << 1.0, 1.0, 1.0;

    VectorXd u_init = VectorXd::Constant(N, 0.5);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    
    // Fix y3 = 1, others free
    VectorXd y_fixed(N);
    y_fixed << std::numeric_limits<double>::quiet_NaN(),
               std::numeric_limits<double>::quiet_NaN(),
               1.0;

    MIQPSolver solver;    
    MIQPResult res = solver.solveMIQP(w, A, obj, u_underbar, u_bar, 
                                     u_init, y_init, y_fixed, true);

    if (res.flag) {
        std::cout << "SOLUTION: y = " << res.y_best.transpose() 
                  << ", u = " << res.u_best.transpose() 
                  << ", cost = " << res.cost_best << "\n";
        
        std::cout << "y[2] should be fixed to 1: " << (res.y_best(2) == 1.0 ? "✓" : "✗") << "\n";
    } else {
        std::cout << "No feasible solution found.\n";
    }
}

void test_case_4() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE 4: No equality constraints\n";
    std::cout << "=========================================\n";
    
    int N = 2;

    MatrixXd Q = MatrixXd::Identity(N, N);
    VectorXd f = VectorXd::Zero(N);
    Objective obj{Q, f};

    // No equality constraints
    MatrixXd A(0, N);
    VectorXd w(0);

    VectorXd u_underbar(N), u_bar(N);
    u_underbar << 0.0, 0.0;
    u_bar << 1.0, 1.0;

    VectorXd u_init = VectorXd::Constant(N, 0.5);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    VectorXd y_fixed = VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());

    MIQPSolver solver;
    
    // Aeq_on = false to ignore equality constraints
    MIQPResult res = solver.solveMIQP(w, A, obj, u_underbar, u_bar, 
                                     u_init, y_init, y_fixed, false);

    if (res.flag) {
        std::cout << "SOLUTION: y = " << res.y_best.transpose() 
                  << ", u = " << res.u_best.transpose() 
                  << ", cost = " << res.cost_best << "\n";
        
        // Optimal solution should be y = [0,0] and u = [0,0] for zero cost
        std::cout << "Expected y = [0,0], u = [0,0], cost = 0\n";
    } else {
        std::cout << "No feasible solution found.\n";
    }
}

void test_case_5() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE 5: Infeasible problem\n";
    std::cout << "=========================================\n";
    
    int N = 2;

    MatrixXd Q = MatrixXd::Identity(N, N);
    VectorXd f = VectorXd::Zero(N);
    Objective obj{Q, f};

    // Conflicting constraints
    MatrixXd A(2, N);
    A << 1.0, 1.0,
         1.0, -1.0;
    VectorXd w(2);
    w << 1.0, 2.0;  // u1+u2=1 and u1-u2=2 -> impossible with bounds

    VectorXd u_underbar(N), u_bar(N);
    u_underbar << 0.0, 0.0;
    u_bar << 1.0, 1.0;

    VectorXd u_init = VectorXd::Constant(N, 0.5);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    VectorXd y_fixed = VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());

    MIQPSolver solver;
    
    MIQPResult res = solver.solveMIQP(w, A, obj, u_underbar, u_bar, 
                                     u_init, y_init, y_fixed, true);

    if (res.flag) {
        std::cout << "SOLUTION: y = " << res.y_best.transpose() 
                  << ", u = " << res.u_best.transpose() 
                  << ", cost = " << res.cost_best << "\n";
    } else {
        std::cout << "Correctly detected infeasible problem ✓\n";
    }
}

void test_case_6() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE 6: Asymmetric bounds\n";
    std::cout << "=========================================\n";
    
    int N = 2;

    MatrixXd Q = MatrixXd::Identity(N, N);
    VectorXd f(N);
    f << 1.0, -1.0;  // Mixed cost coefficients
    Objective obj{Q, f};

    MatrixXd A(1, N);
    A << 1.0, 1.0;
    VectorXd w(1);
    w << 0.5;

    // Different bounds for different variables
    VectorXd u_underbar(N), u_bar(N);
    u_underbar << 0.0, 0.0;
    u_bar << 2.0, 1.0;  // u1 in [0,2] when y1=1, u2 in [0,1] when y2=1

    VectorXd u_init = VectorXd::Constant(N, 0.5);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    VectorXd y_fixed = VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());

    MIQPSolver solver;
    
    MIQPResult res = solver.solveMIQP(w, A, obj, u_underbar, u_bar, 
                                     u_init, y_init, y_fixed, true);

    if (res.flag) {
        std::cout << "SOLUTION: y = " << res.y_best.transpose() 
                  << ", u = " << res.u_best.transpose() 
                  << ", cost = " << res.cost_best << "\n";
        
        VectorXd Au = A * res.u_best;
        std::cout << "Constraint A*u = " << Au(0) << " (should be " << w(0) << ")\n";
    } else {
        std::cout << "No feasible solution found.\n";
    }
}*/

void test_random_complex() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE: Random Complex Problem (Improved)\n";
    std::cout << "=========================================\n";
    
    // Parametri più rilassati per aumentare la probabilità di fattibilità
    int N = 8;   // Riduci il numero di variabili
    int M = 3;   // Riduci il numero di vincoli

    // Seed deterministico
    std::srand(42);

    // Costruisci matrici casuali
    MatrixXd A(M, N);
    A <<-0.93306,  -0.155027,   0.273117,  -0.950152,  -0.364279,   0.521343,   0.129304,    -0.5623,
         -0.340072,   -0.58747,   0.727245,  -0.270015,  -0.728543,  -0.836655,   0.486541, -0.0912082,
          0.381271,  -0.499743,  -0.396689,   0.530762,  -0.786509,   0.101912,    0.96352,   0.037308;
    
    // Genera w in modo che il problema sia più probabilmente fattibile
    VectorXd u_feasible(N);
    u_feasible << 0.147683, 0.110462, 0.457093, 0.257748, 0.592256, 0.167432, 0.450467, 0.659196;
    VectorXd w = A * u_feasible; // w compatibile con una soluzione

    // Matrice Q positiva definita
    MatrixXd Q = MatrixXd::Identity(N, N); // Usa identità per semplicità

    VectorXd f = VectorXd::Zero(N); // Zero costi lineari per semplicità

    // Limiti più ragionevoli
    VectorXd u_underbar = VectorXd::Constant(N, 0.5);
    VectorXd u_bar = VectorXd::Constant(N, 2.0);

    // Punti iniziali
    VectorXd u_init = VectorXd::Constant(N, 1.0);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    VectorXd y_fixed = VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());
    y_fixed(1) = 0.0;

    // Costruisci oggetto obiettivo
    Objective objective;
    objective.Q = Q;
    objective.f = f;

    // Costruisci solver con parametri più conservativi
    MIQPSolver solver;  // max_depth=30, verbose=true

    bool Aeq_on = true;

    // Misura il tempo di esecuzione    
    std::cout << "Risoluzione MIQP (N=" << N << ", M=" << M << ")..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    MIQPResult result = solver.solveMIQP(
    w, A, objective, u_underbar, u_bar,
    u_init, y_init, y_fixed, Aeq_on, 0.0);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Visualizza risultati
    if (result.flag) {
        std::cout << "✅ Soluzione trovata\n";
        std::cout << "Tempo di esecuzione: " << duration.count() << " ms\n";
        std::cout << "Costo ottimo: " << result.cost_best << "\n";
        std::cout << "Numero di y=1: " << (result.y_best.array() > 0.5).count()
                  << " su " << N << "\n";
        std::cout << "u_best: " << result.u_best.transpose() << "\n";

        // Controlla vincoli
        VectorXd residuo = A * result.u_best - w;
        std::cout << "Norma residuo A*u - w: " << residuo.norm() << std::endl;
        
        // Verifica ammissibilità dei bounds
        bool feasible = true;
        for (int i = 0; i < N; ++i) {
            double lb = (result.y_best(i) == 1.0) ? u_underbar(i) : -u_bar(i);
            double ub = (result.y_best(i) == 1.0) ? u_bar(i) : -u_underbar(i);
            if (result.u_best(i) < lb - 1e-6 || result.u_best(i) > ub + 1e-6) {
                feasible = false;
                std::cout << "✗ Vincolo violato per u[" << i << "]: " 
                          << result.u_best(i) << " non in [" << lb << ", " << ub << "]\n";
            }
        }
        if (feasible) {
            std::cout << "✓ Tutti i vincoli rispettati\n";
        }
        
    } else {
        std::cout << "❌ Nessuna soluzione trovata\n";
        std::cout << "Tempo di esecuzione: " << duration.count() << " ms\n";
    }
}

void test_real_case_from_log() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE: Real data from log\n";
    std::cout << "=========================================\n";

    constexpr int N = 8;
    constexpr int M = 6;

    Eigen::MatrixXd A(M, N);
    A <<
        -3.24394e-05,  6.17596e-05,  7.08026e-05, -2.33964e-05,  7.08026e-05, -2.33964e-05, -3.24394e-05,  6.17596e-05,
        -1.42802e-05, -3.51333e-05,  5.61002e-05,  7.69533e-05,  5.61002e-05,  7.69533e-05, -1.42802e-05, -3.51333e-05,
         8.52320e-05,  5.89249e-05,  1.89868e-05,  4.52939e-05,  1.89868e-05,  4.52939e-05,  8.52320e-05,  5.89249e-05,
         9.17398e-04,  2.33759e-03, -9.17398e-04, -2.33759e-03,  1.21568e-03,  2.33759e-03, -1.21568e-03, -2.33759e-03,
        -2.33759e-03,  1.21568e-03,  2.33759e-03, -1.21568e-03, -2.33759e-03,  9.17398e-04,  2.33759e-03, -9.17398e-04,
         1.39619e-03,  1.14591e-03,  1.39619e-03,  1.14591e-03, -1.14591e-03, -1.39619e-03, -1.14591e-03, -1.39619e-03;

    Eigen::VectorXd w(M);
    w <<
        -0.307954,
         0.0801745,
         9.91072,
        -2.21015,
         0.846444,
         0.801699;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(N, N);

    // f dal log (nota: è l'opposto di u_prev)
    Eigen::VectorXd f(N);
    f <<
        -36147.7,
        -21121.0,
          -900.0,
        -14801.7,
           900.0,
        -18416.9,
        -32703.6,
        -21884.3;

    Eigen::VectorXd u_prev(N);
    u_prev <<
        36147.7,
        21121.0,
          900.0,
        14801.7,
         -900.0,
        18416.9,
        32703.6,
        21884.3;

    Eigen::VectorXd y_prev(N);
    y_prev <<
        1, 1, 1, 1, 0, 1, 1, 1;

    // Bounds richiesti
    const double ubar_val = 220.0 * 220.0;   // 48400
    const double uunder_val = 30.0 * 30.0;  // 900
    Eigen::VectorXd u_bar = Eigen::VectorXd::Constant(N, ubar_val);
    Eigen::VectorXd u_underbar = Eigen::VectorXd::Constant(N, uunder_val);

    // Init come da log
    Eigen::VectorXd u_init = u_prev;
    Eigen::VectorXd y_init = y_prev;

    // y_fixed: NaN ovunque, tranne indice 4 = 0
    Eigen::VectorXd y_fixed = Eigen::VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());
    y_fixed(4) = 0.0;

    miqp::Objective objective;
    objective.Q = Q;
    objective.f = f;

    miqp::MIQPSolver solver;
    solver.setVerbose(true);

    // Se vuoi evitare timeout nel test, alza qui:
    // solver.setTimeout(10.0); // ms

    bool Aeq_on = true;

    std::cout << "Risoluzione MIQP (N=" << N << ", M=" << M << ")...\n";
    auto start_time = std::chrono::high_resolution_clock::now();

    miqp::MIQPResult result = solver.solveMIQP(
        w, A, objective,
        u_underbar, u_bar,
        u_init, y_init, y_fixed,
        Aeq_on, 0.0
    );

    auto end_time = std::chrono::high_resolution_clock::now();
    double dt_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    std::cout << "Status: " << (int)result.status
              << "  flag=" << result.flag
              << "  cost_best=" << result.cost_best
              << "  solve_time_ms=" << result.solve_time_ms
              << "  wall_ms=" << dt_ms << "\n";

    if (result.flag) {
        std::cout << "u_prev: " << u_prev.transpose() << "\n";
        std::cout << "u_best: " << result.u_best.transpose() << "\n";
        std::cout << "y_fixed: " << y_fixed.transpose() << "\n";
        std::cout << "y_init: " << y_init.transpose() << "\n";
        std::cout << "y_best: " << result.y_best.transpose() << "\n";
    }
}


/*void test_random_complex2() {
    std::cout << "\n=========================================\n";
    std::cout << "TEST CASE: Random Complex Problem (Improved)\n";
    std::cout << "=========================================\n";
    
    // Parametri più rilassati per aumentare la probabilità di fattibilità
    int N = 8;   // Riduci il numero di variabili
    int M = 3;   // Riduci il numero di vincoli

    // Seed deterministico
    std::srand(42);

    // Costruisci matrici casuali
    MatrixXd A(M, N);
    A <<-0.93306,  -0.155027,   0.273117,  -0.950152,  -0.364279,   0.521343,   0.129304,    -0.5623,
         -0.340072,   -0.58747,   0.727245,  -0.270015,  -0.728543,  -0.836655,   0.486541, -0.0912082,
          0.381271,  -0.499743,  -0.396689,   0.530762,  -0.786509,   0.101912,    0.96352,   0.037308;
    
    // Genera w in modo che il problema sia più probabilmente fattibile
    VectorXd u_feasible(N);
    u_feasible << 0.147683, 0.110462, 0.457093, 0.257748, 0.592256, 0.167432, 0.450467, 0.659196;
    VectorXd w = A * u_feasible; // w compatibile con una soluzione

    // Matrice Q positiva definita
    MatrixXd Q = MatrixXd::Identity(N, N); // Usa identità per semplicità

    VectorXd f = VectorXd::Zero(N); // Zero costi lineari per semplicità

    // Limiti più ragionevoli
    VectorXd u_underbar = VectorXd::Constant(N, 0.5);
    VectorXd u_bar = VectorXd::Constant(N, 2.0);

    // Punti iniziali
    VectorXd u_init = VectorXd::Constant(N, 1.0);
    VectorXd y_init = VectorXd::Constant(N, 0.5);
    VectorXd y_fixed = VectorXd::Constant(N, std::numeric_limits<double>::quiet_NaN());

    // Flag per i vincoli di uguaglianza
    bool Aeq_on = true;

    // Misura il tempo di esecuzione    
    std::cout << "Risoluzione MIQP (N=" << N << ", M=" << M << ")..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // CHIAMA LA FUNZIONE C++ solve_miqp_bb_stack_2
    MIQPResult result = solve_miqp_bb_stack_2(
        w, A, Q, u_underbar, u_bar,
        u_init, y_init, y_fixed,
        Q, f,  // objective_Q e objective_f
        true   // Aeq_on
    );
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Visualizza risultati
    if (result.flag) {
        std::cout << "✅ Soluzione trovata\n";
        std::cout << "Tempo di esecuzione: " << duration.count() << " ms\n";
        std::cout << "Costo ottimo: " << result.cost_best << "\n";
        std::cout << "Numero di y=1: " << (result.y_best.array() > 0.5).count()
                  << " su " << N << "\n";
        std::cout << "u_best: " << result.u_best.transpose() << "\n";

        // Controlla vincoli
        VectorXd residuo = A * result.u_best - w;
        std::cout << "Norma residuo A*u - w: " << residuo.norm() << std::endl;
        
        // Verifica ammissibilità dei bounds
        bool feasible = true;
        for (int i = 0; i < N; ++i) {
            double lb = (result.y_best(i) == 1.0) ? u_underbar(i) : -u_bar(i);
            double ub = (result.y_best(i) == 1.0) ? u_bar(i) : -u_underbar(i);
            if (result.u_best(i) < lb - 1e-6 || result.u_best(i) > ub + 1e-6) {
                feasible = false;
                std::cout << "✗ Vincolo violato per u[" << i << "]: " 
                          << result.u_best(i) << " non in [" << lb << ", " << ub << "]\n";
            }
        }
        if (feasible) {
            std::cout << "✓ Tutti i vincoli rispettati\n";
        }
        
    } else {
        std::cout << "❌ Nessuna soluzione trovata\n";
        std::cout << "Tempo di esecuzione: " << duration.count() << " ms\n";
    }
}*/

// Funzione per generare una matrice simmetrica definita positiva casuale
MatrixXd generatePositiveDefiniteMatrix(int N) {
    MatrixXd R = MatrixXd::Random(N, N);
    // Q = R^T * R + I (rende Q definita positiva)
    MatrixXd Q = R.transpose() * R + MatrixXd::Identity(N, N) * N; 
    return Q;
}


int main() {

    // Ogni test verifica un aspetto diverso del solver
    /*test_case_1();   // QP semplice in 2D
    test_case_2();   // Costi negativi, 3D
    test_case_3();   // Alcune variabili fissate
    test_case_4();   // Nessun vincolo di uguaglianza
    test_case_5();   // Problema infeasible
    test_case_6();   // Limiti asimmetrici*/

    test_real_case_from_log();  // Problema casuale più complesso

    return 0;
}