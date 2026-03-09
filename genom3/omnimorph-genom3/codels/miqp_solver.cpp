// miqp_solver.cpp (versione con try-catch e debug)
#include <iostream>
#include "miqp_solver.h"
#include "proxsuite/proxqp/dense/dense.hpp"
#include <algorithm>
#include <limits>

using namespace proxsuite::proxqp;
using namespace Eigen;

namespace miqp {

// --- Globals / cached dims ---
int qp_leaf_dim_ = -1;
int qp_leaf_neq_ = -1;
int qp_leaf_nin_ = -1;
int qp_relax_dim_ = -1;
int qp_relax_neq_ = -1;
int qp_relax_nin_ = -1;
bool qp_leaf_initialized_;
bool qp_relax_initialized_;
const double LARGE_VALUE = 1e6; 
const double lambda_y = 1e4;

namespace {
    int current_N = -1;
    VectorXd cached_u_underbar;
    VectorXd cached_u_bar;

    // Relax problem buffers
    MatrixXd H_relax;
    VectorXd f_relax;
    MatrixXd Aeq_relax;
    VectorXd beq_relax;
    MatrixXd Aineq_relax;
    VectorXd l_ineq_relax;
    VectorXd u_ineq_relax;
    VectorXd lb_relax;
    VectorXd ub_relax;
    VectorXd z0_relax;

    // Buffers for leaf problem
    MatrixXd H_leaf;
    VectorXd f_leaf;
    MatrixXd Aeq_leaf;
    VectorXd beq_leaf;
    VectorXd lb_u_leaf;
    VectorXd ub_u_leaf;

    // Temporary outputs
    VectorXd tmp_u;
    VectorXd tmp_y;
}

// --- MIQPSolver methods ---

MIQPSolver::MIQPSolver() : MIQPSolver(13, true)
{
}

MIQPSolver::MIQPSolver(int max_depth, bool verbose)
{
    max_depth_ = max_depth;
    verbose_ = verbose;
    qp_relax_initialized_ = false;
    qp_leaf_initialized_ = false;
}

// Helper: ensure buffers sized for N
static void ensure_buffers_for_N(int N)
{
    if (current_N == N) return;
    current_N = N;
    int nVar = 2 * N;    
    try {
        H_relax.setZero(nVar, nVar);
        f_relax.setZero(nVar);
        Aeq_relax.resize(0, nVar);
        beq_relax.resize(0);
        Aineq_relax.setZero(2 * N, nVar);
        l_ineq_relax = VectorXd::Constant(2 * N, -LARGE_VALUE);
        u_ineq_relax.setZero(2 * N);
        lb_relax = VectorXd::Constant(nVar, -LARGE_VALUE);
        ub_relax = VectorXd::Constant(nVar, LARGE_VALUE);
        z0_relax.setZero(nVar);
        // leaf buffers
        H_leaf.setZero(N, N);
        f_leaf.setZero(N);
        Aeq_leaf.resize(0, N);
        beq_leaf.resize(0);
        lb_u_leaf.setZero(N);
        ub_u_leaf.setZero(N);
        tmp_u.setZero(N);
        tmp_y.setZero(N);
        // invalidate cached u_underbar/u_bar
        cached_u_underbar.resize(0);
        cached_u_bar.resize(0);
    } catch (const std::exception& e) {
        std::cerr << "ERROR in ensure_buffers_for_N: " << e.what() << std::endl;
        throw;
    }
}


// --- solveMIQP con try-catch ---
MIQPResult MIQPSolver::solveMIQP(
    const Eigen::VectorXd& w,
    const Eigen::MatrixXd& A,
    const Objective& objective,
    const Eigen::VectorXd& u_underbar,
    const Eigen::VectorXd& u_bar,
    const Eigen::VectorXd& u_init,
    const Eigen::VectorXd& y_init,
    const Eigen::VectorXd& y_fixed,
    bool Aeq_on,
    double previous_time)
{
    MIQPResult res;
    auto start_time = std::chrono::high_resolution_clock::now();   
    auto timed_out = [&](){
        auto now = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(now - start_time).count();
        return (ms + previous_time) > timeout_ms_;
    };
    int N = A.cols(); 
    int nVar = 2*N;        
    // Ensure buffers sized
    ensure_buffers_for_N(N);

    // Fill H_relax and f_relax una sola volta all'inizio
    H_relax.topLeftCorner(N, N).noalias() = objective.Q;
    H_relax.bottomRightCorner(N,N).diagonal().array() = 1e-3;
    
    f_relax.head(N) = objective.f;
    for (int i = 0; i < N; ++i) {
        f_relax(N + i) = lambda_y * (1.0 - 2.0 * y_init(i));
    }

    // Setup Aeq_relax una volta sola
    if (Aeq_on && A.rows() > 0 ){
        Aeq_relax.resize(A.rows(), nVar);
        Aeq_relax.setZero();
        Aeq_relax.block(0, 0, A.rows(), N).noalias() = A;
        beq_relax = w;
        Aeq_leaf.resize(A.rows(), N);
        Aeq_leaf.setZero();
        Aeq_leaf.noalias() = A;
        beq_leaf = w;
    }
    else {
        Aeq_relax.resize(0, nVar);
        beq_relax.resize(0);
        Aeq_leaf.resize(0, N);
        beq_leaf.resize(0); 
    }
    
    // Precompute Aineq_relax una volta sola
    for (int i = 0; i < N; ++i) {
        Aineq_relax(i, i) = -1.0;
        Aineq_relax(i, N + i) = u_underbar(i) + u_bar(i);
        u_ineq_relax(i) = u_bar(i);
        
        int row = N + i;
        Aineq_relax(row, i) = 1.0;
        Aineq_relax(row, N + i) = - u_underbar(i) - u_bar(i);
        u_ineq_relax(row) = -u_underbar(i);
    }
    cached_u_underbar = u_underbar;
    cached_u_bar = u_bar;
    
    // Setup leaf H_leaf e f_leaf una volta sola
    H_leaf.noalias() = objective.Q;
    f_leaf.noalias() = objective.f;    

    // Best solution tracking
    double cost_best = LARGE_VALUE;
    VectorXd u_best, y_best;
    VectorXd y_temp = y_init;
    bool flag = false;

    // Initial bounds on y
    VectorXd y_lb0 = VectorXd::Zero(N);
    VectorXd y_ub0 = VectorXd::Ones(N);

    // Use stack for iterative B&B
    std::vector<BBNode> stack;
    stack.reserve( max_depth_);

    // Push root node
    BBNode root;    
    // Apply fixed y entries (one-time)
    for (int i = 0; i < N; ++i) {
        if (!std::isnan(y_fixed(i))) {
            y_lb0(i) = y_fixed(i);
            y_ub0(i) = y_fixed(i);
            y_temp(i) = y_fixed(i);
            if (y_fixed(i) != 0.0 && y_fixed(i) != 1.0) {
                throw std::runtime_error("y_fixed entries must be NaN, 0 or 1.");
            }            
        }
    }    
    root.u_start = u_init;
    root.y_start = y_temp;
    root.y_lb = y_lb0;
    root.y_ub = y_ub0;
    root.is_pre_solved = false;
    root.depth = 0;
    stack.push_back(root);    

    while (!stack.empty()) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(current_time - start_time).count();            
        if (elapsed_ms + previous_time > timeout_ms_) {
            if (verbose_) {
                std::cout << "MIQP timeout after " << elapsed_ms + previous_time<< " ms (limit: " << timeout_ms_  << " ms)" << std::endl;
            }
            res.flag = false;
            res.status = MIQPSolverStatus::TIMEOUT;
            res.solve_time_ms = elapsed_ms;
            return res;            
        }
        BBNode node = std::move(stack.back()); // Muovi fuori dallo stack
        stack.pop_back();

        // guard recursion depth
        if (node.depth > max_depth_) {
            if (verbose_) std::cout << "Warning: max depth reached\n";
            continue;
        }

        // check if leaf (all fixed)
        bool all_fixed = true;
        for (int i = 0; i < N; ++i) {
            if (std::abs(node.y_lb(i) - node.y_ub(i)) > MIQP_TOL) {
                all_fixed = false;
                break;
            }
        }
        if (all_fixed) {
            VectorXd y_fixed_now = node.y_lb.array().round();
            VectorXd u_opt;
            double cost_u;
            if (timed_out()) continue; 
            //std::cout<<"tutti fissi, risolvo foglia"<<std::endl;
            bool ok = solveLeafQP(y_fixed_now, node.u_start, y_init, u_opt, cost_u);
            if (ok && cost_u < cost_best) {
                cost_best = cost_u;
                u_best = u_opt;
                y_best = (u_best.array() > 0).cast<double>();
                flag = true;
            }
            continue;
        }
        if (timed_out()) continue; 
        // Solve relaxed QP for this node
        VectorXd u_relax(N), y_relax(N);
        double cost_relax;
        bool ok_relax = false;
        if (node.is_pre_solved) {
                // Se ho già risolto questo nodo quando era un figlio
                u_relax = node.u_sol;
                y_relax = node.y_sol;
                cost_relax = node.pre_solved_cost;
                ok_relax = true; 
            } 
        else {
            // Nodo vergine (es. radice), devo risolvere
            if (timed_out()) continue; 
            ok_relax = solveRelaxedQP(node.y_lb, node.y_ub, node.u_start, node.y_start,
                                        u_relax, y_relax, cost_relax);
            if (timed_out()) continue; 
            //std::cout<<"nodo vergine " << y_relax.transpose()<<std::endl;
        }
        
        if (!ok_relax) continue;
        VectorXd y_round = y_relax.array().round();
        VectorXd u_leaf; double cost_leaf;
        if (timed_out()) continue;
        if (solveLeafQP(y_round, u_relax, y_init, u_leaf, cost_leaf)) {
            if (cost_leaf + 1e-6 < cost_best) {
                cost_best = cost_leaf;
                u_best = u_leaf;
                y_best = y_round;
                flag = true;
            }
        }
        // prune by bound
        if (cost_relax  >= cost_best - 1e-6) continue;

        // Check if all y are integral within tolerance
        bool all_integral = true;
        for (int i = 0; i < N; ++i) {
            if (std::abs(y_relax(i) - std::round(y_relax(i))) > MIQP_TOL) {
                all_integral = false;
                break;
            }
        }        
        if (all_integral) {
            VectorXd y_int = y_relax.array().round();
            VectorXd u_leaf;
            double cost_leaf;
            if (solveLeafQP(y_int, u_relax, y_init, u_leaf, cost_leaf)) {
                if (cost_leaf + 1e-6 < cost_best) {
                    cost_best = cost_leaf;
                    u_best = u_leaf;
                    y_best = y_int;
                    flag = true;
                }
            }
            continue;            
        }
        
        // Pick index to branch: most fractional among unfixed variables
        std::vector<int> unfixed;
        for (int i = 0; i < N; ++i) {
            if (node.y_lb(i) + 1e-4 < node.y_ub(i)) {
                unfixed.push_back(i);
            }
        }        
        if (unfixed.empty()) {
            continue;
        }        
        int branch_idx = -1;
        double max_fractional = -1.0;
        for (int i : unfixed) {
            double fractional = std::abs(y_relax(i) - std::round(y_relax(i)));
            if (fractional > max_fractional) {
                max_fractional = fractional;
                branch_idx = i;
            }
        }
        if (branch_idx == -1) continue;
        if (node.y_lb(branch_idx) == node.y_ub(branch_idx)) continue;

        BBNode child0;
        child0.u_start = u_relax;
        child0.y_start = y_relax;
        child0.y_lb = node.y_lb;   
        child0.y_ub = node.y_ub;     
        child0.y_lb(branch_idx) = 0.0;
        child0.y_ub(branch_idx) = 0.0;
        child0.y_start(branch_idx) = 0.0;
        for (int i = 0; i < N; ++i) {
            double v = child0.y_start(i);
            if (v < child0.y_lb(i)) child0.y_start(i) = child0.y_lb(i);
            else if (v > child0.y_ub(i)) child0.y_start(i) = child0.y_ub(i);
        }
        child0.depth = node.depth + 1;
        if (timed_out()) continue; 

        BBNode child1;
        child1.u_start = u_relax;
        child1.y_start = y_relax; 
        child1.y_lb = node.y_lb;   
        child1.y_ub = node.y_ub; 
        child1.y_lb(branch_idx) = 1.0;
        child1.y_ub(branch_idx) = 1.0;
        child1.y_start(branch_idx) = 1.0;
        for (int i = 0; i < N; ++i) {
            double v = child1.y_start(i);
            if (v < child1.y_lb(i)) child1.y_start(i) = child1.y_lb(i);
            else if (v > child1.y_ub(i)) child1.y_start(i) = child1.y_ub(i);
        }
        child1.depth = node.depth + 1;

        VectorXd u0(N), y0(N), u1(N), y1(N);
        double cost0 = LARGE_VALUE;
        double cost1 = LARGE_VALUE;
        bool ok0, ok1;

        if (timed_out()) continue;     
        // Risolvi Child 0
        ok0 = solveRelaxedQP(child0.y_lb, child0.y_ub, child0.u_start, child0.y_start,
                                    u0, y0, cost0);
        if (timed_out()) continue; 
        // Salva risultato in Child 0
        if (ok0) {
            child0.is_pre_solved = true;
            child0.pre_solved_cost = cost0;
            child0.u_sol = u0;
            child0.y_sol = y0;
        }

        // Risolvi Child 1
        if (timed_out()) continue; 
        ok1 = solveRelaxedQP(child1.y_lb, child1.y_ub, child1.u_start, child1.y_start,
                                    u1, y1, cost1);
        if (timed_out()) continue; 
        // Salva risultato in Child 1
        if (ok1) {
            child1.is_pre_solved = true;
            child1.pre_solved_cost = cost1;
            child1.u_sol = u1;
            child1.y_sol = y1;
        }

        // --- PUSH STRATEGICO (Best-First) ---
        // Pushiamo nello stack. Lo stack è LIFO (ultimo entrato, primo uscito).
        // Vogliamo che il MIGLIORE esca PRIMA. Quindi pushiamo il MIGLIORE per ULTIMO.

        if (ok0 && ok1) {
            if (cost0 <= cost1) {
                // 0 è meglio.
                stack.push_back(std::move(child1)); // 1 (peggiore) in fondo
                stack.push_back(std::move(child0)); // 0 (migliore) in cima 
            } else {
                // 1 è meglio.
                stack.push_back(std::move(child0));
                stack.push_back(std::move(child1));
            }
        } else if (ok0) {
            stack.push_back(std::move(child0));
        } else if (ok1) {
            stack.push_back(std::move(child1));
        } 
        else continue;
    }    
    // set results
    if (flag) {
        res.u_best = u_best;
        res.y_best = y_best;
        res.cost_best = cost_best;
        res.flag = true;
        res.status = MIQPSolverStatus::SOLVED;
    } else {
        res.u_best = VectorXd::Zero(N); 
        res.y_best = VectorXd::Zero(N);
        res.cost_best = LARGE_VALUE;
        res.flag = false;
        res.status = MIQPSolverStatus::INFEASIBLE;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    res.solve_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    return res;
}

// ---------- solveRelaxedQP con try-catch e controlli ----------
bool MIQPSolver::solveRelaxedQP(
    const Eigen::VectorXd& y_lb,
	const Eigen::VectorXd& y_ub,
	const Eigen::VectorXd& u_start,
	const Eigen::VectorXd& y_start,
	Eigen::VectorXd& u_opt,
	Eigen::VectorXd& y_opt,
	double& cost_opt)
{
    int N = y_lb.rows();
    int nVar = 2 * N;

    // bounds
    for (int i = 0; i < N; ++i) {
        lb_relax(i) = -cached_u_bar(i);
        ub_relax(i) = cached_u_bar(i);
        lb_relax(N + i) = y_lb(i);
        ub_relax(N + i) = y_ub(i);
    }

    // initial point
    z0_relax.head(N) = u_start;
    z0_relax.tail(N) = y_start;

    for (int i = 0; i < nVar; ++i) {
        if (z0_relax(i) < lb_relax(i)) {
            z0_relax(i) = lb_relax(i); 
        } else if (z0_relax(i) > ub_relax(i)) {
            z0_relax(i) = ub_relax(i);
        }
    }
    // Create/resize solver
    if (!qp_relax_initialized_ ||
        qp_relax_dim_ != nVar ||
        qp_relax_neq_ != Aeq_relax.rows() ||
        qp_relax_nin_ != Aineq_relax.rows())
    {
        qp_relax_.reset(new proxsuite::proxqp::dense::QP<double>(nVar, Aeq_relax.rows(), Aineq_relax.rows(), true, proxsuite::proxqp::DenseBackend::PrimalDualLDLT, proxsuite::proxqp::HessianType::Dense));    
        qp_relax_initialized_ = true;
        qp_relax_dim_ = nVar;
        qp_relax_neq_ = Aeq_relax.rows();
        qp_relax_nin_ = Aineq_relax.rows();
        qp_relax_->settings.verbose = false;
        qp_relax_->settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        qp_relax_->settings.eps_abs = 1e-5;
        qp_relax_->settings.eps_rel = 1e-5;
        qp_relax_->settings.max_iter = 15;
    }

    // update solver
    qp_relax_->update(H_relax, f_relax, Aeq_relax, beq_relax,
                        Aineq_relax, l_ineq_relax, u_ineq_relax, lb_relax, ub_relax);
    // warm-start
    qp_relax_->results.x = z0_relax;
    qp_relax_->results.y.setZero();
    qp_relax_->results.z.setZero();
    // solve
    qp_relax_->solve();
    if (qp_relax_->results.info.status == QPSolverOutput::PROXQP_SOLVED) {
        u_opt = qp_relax_->results.x.head(N);
        y_opt = qp_relax_->results.x.tail(N);
        for (int i=0;i<N;++i){
        if (y_opt(i) < 1e-5) y_opt(i)=0.0;
        if (y_opt(i) > 1.0-1e-5) y_opt(i)=1.0;
        }
        cost_opt = qp_relax_->results.info.objValue;
        return true;
    } else {
        return false;
    }
}

bool MIQPSolver::solveLeafQP(
    const Eigen::VectorXd& y_fixed,
	const Eigen::VectorXd& u_start,
    const Eigen::VectorXd& y_start,
	Eigen::VectorXd& u_opt,
    double& cost_opt)
{
    int N = y_fixed.rows();
    //ensure_buffers_for_N(N);

    for (int i = 0; i < N; ++i) {
        if (y_fixed(i) == 1.0) {
            lb_u_leaf(i) = cached_u_underbar(i);
            ub_u_leaf(i) = cached_u_bar(i);
        } else {
            lb_u_leaf(i) = -cached_u_bar(i);
            ub_u_leaf(i) = -cached_u_underbar(i);
        }            
    }

    // initial point
    tmp_u.head(N) = u_start;
    for (int i = 0; i < N; ++i) {
        double v = tmp_u(i);
        if (v < lb_u_leaf(i)) tmp_u(i) = lb_u_leaf(i);
        else if (v > ub_u_leaf(i)) tmp_u(i) = ub_u_leaf(i);
    }

    // Create/reuse leaf solver
    if (!qp_leaf_initialized_ ||
        qp_leaf_dim_ != N ||
        qp_leaf_neq_ != Aeq_leaf.rows() ||
        qp_leaf_nin_ != 0)
    {
        qp_leaf_.reset(new proxsuite::proxqp::dense::QP<double>(N, Aeq_leaf.rows(), 0, true, proxsuite::proxqp::DenseBackend::PrimalDualLDLT, proxsuite::proxqp::HessianType::Dense));
        qp_leaf_initialized_ = true;
        qp_leaf_dim_ = N;
        qp_leaf_neq_ = Aeq_leaf.rows();
        qp_leaf_nin_ = 0;

        qp_leaf_->settings.verbose = false;
        qp_leaf_->settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        qp_leaf_->settings.eps_abs = 1e-5;
        qp_leaf_->settings.eps_rel = 1e-5;
        qp_leaf_->settings.max_iter = 20;

    }  
    // update solver
    qp_leaf_->update(H_leaf, f_leaf, Aeq_leaf, beq_leaf,
                 MatrixXd(0, N), VectorXd(0), VectorXd(0),
                 lb_u_leaf, ub_u_leaf);

    // warm-start
    qp_leaf_->results.x = tmp_u;
    qp_leaf_->results.y.setZero();
    qp_leaf_->results.z.setZero();
    // solve
    qp_leaf_->solve();

    if (qp_leaf_->results.info.status == QPSolverOutput::PROXQP_SOLVED) {
        u_opt = qp_leaf_->results.x;
        cost_opt = qp_leaf_->results.info.objValue + lambda_y * (y_fixed - y_start).cwiseAbs().sum();;
        return true;
    } else {
        cost_opt = LARGE_VALUE;
        return false;
    }
}

} // namespace miqp