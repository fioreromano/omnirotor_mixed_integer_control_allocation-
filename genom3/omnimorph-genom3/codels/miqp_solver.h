#ifndef MIQP_SOLVER_H
#define MIQP_SOLVER_H

#include <Eigen/Dense>
#include <vector>
#include <stack>
#include <limits>
#include <cmath>
#include <memory> 
#include "proxsuite/proxqp/dense/dense.hpp" // Aggiunto

using namespace proxsuite::proxqp;
using namespace Eigen;

namespace miqp {

// --- Strutture Dati ---
enum class MIQPSolverStatus {
    SOLVED,           // Soluzione trovata
    TIMEOUT,          // Tempo superato
    INFEASIBLE,       // Problema infattibile
    ERROR             // Errore generico
};

struct Objective {
    Eigen::MatrixXd Q;
    Eigen::VectorXd f;
};

struct BBNode {
    Eigen::VectorXd y_lb;
    Eigen::VectorXd y_ub;
    Eigen::VectorXd u_start;
    Eigen::VectorXd y_start;
    int depth;
    bool is_pre_solved = false; 
    double pre_solved_cost = 1e6;
    Eigen::VectorXd u_sol;     
    Eigen::VectorXd y_sol;
};

struct MIQPResult {
    Eigen::VectorXd u_best;
    Eigen::VectorXd y_best;
    double cost_best;
    bool flag;
    MIQPSolverStatus status; 
    double solve_time_ms;
};

// --- Classe MIQPSolver ---

class MIQPSolver {
private: 
    bool verbose_ = true;
    int max_depth_ = 13;    
    double timeout_ms_ = 2;
    const double MIQP_TOL = 1e-2;
    std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_relax_;
    std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_leaf_;


public:
    MIQPSolver();
    MIQPSolver(int max_depth, bool verbose);

    MIQPResult solveMIQP(
        const Eigen::VectorXd& w,
        const Eigen::MatrixXd& A,
        const Objective& objective,
        const Eigen::VectorXd& u_underbar,
        const Eigen::VectorXd& u_bar,
        const Eigen::VectorXd& u_init,
        const Eigen::VectorXd& y_init,
        const Eigen::VectorXd& y_fixed,
        bool Aeq_on,
        double previous_time);    

    bool solveRelaxedQP(
	const Eigen::VectorXd& y_lb,
	const Eigen::VectorXd& y_ub,
	const Eigen::VectorXd& u_start,
	const Eigen::VectorXd& y_start,
	Eigen::VectorXd& u_opt,
	Eigen::VectorXd& y_opt,
	double& cost_opt);

    bool solveLeafQP(
	 const Eigen::VectorXd& y_fixed,
	 const Eigen::VectorXd& u_start,
	 const Eigen::VectorXd& y_start,
	 Eigen::VectorXd& u_opt,
	 double& cost_opt);
       
    void setVerbose(bool verbose) { verbose_ = verbose; }
    void setMaxDepth(int max_depth) { max_depth_ = max_depth; }
    void setTimeout(double timeout_ms) { timeout_ms_ = timeout_ms; }      
    double getTimeoutMs() const { return timeout_ms_; }   
};

} // namespace miqp

#endif

