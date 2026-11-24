#include <Eigen/Core>
#include <cassert>
#include <type_traits>
#include <utility>

template <typename Fn, typename StateVec, typename = void> struct is_state_derivative : std::false_type {};

template <typename Fn, typename StateVec>
struct is_state_derivative<
    Fn, StateVec,
    std::void_t<decltype(std::declval<Fn>()(std::declval<typename StateVec::Scalar>(), std::declval<const StateVec&>()))>>
    : std::is_convertible<
          decltype(std::declval<Fn>()(std::declval<typename StateVec::Scalar>(), std::declval<const StateVec&>())), StateVec> {
};

template <typename Fn, typename StateVec> constexpr bool is_state_derivative_v = is_state_derivative<Fn, StateVec>::value;

/// Preallocated scratch buffers for an X-by-1 Eigen state vector in C++17.
/// Works for both fixed-size (e.g., Eigen::Matrix<double, 6, 1>) and
/// dynamic-size (e.g., Eigen::VectorXd) column vectors.
template <typename StateVec> struct Rk4Workspace {
    static_assert(StateVec::ColsAtCompileTime == 1, "StateVec must be an X-by-1 column vector");
    using Scalar = typename StateVec::Scalar;

    StateVec x_curr{};
    StateVec x_stage{};
    StateVec k1{};
    StateVec k2{};
    StateVec k3{};
    StateVec k4{};

    explicit Rk4Workspace(Eigen::Index rows = (StateVec::RowsAtCompileTime > 0 ? StateVec::RowsAtCompileTime : 0)) {
        resize(rows);
    }

    void resize(Eigen::Index rows) {
        if constexpr (StateVec::RowsAtCompileTime == Eigen::Dynamic) {
            assert(rows >= 0);
            x_curr.resize(rows);
            x_stage.resize(rows);
            k1.resize(rows);
            k2.resize(rows);
            k3.resize(rows);
            k4.resize(rows);
        } else {
            assert(rows == StateVec::RowsAtCompileTime);
            (void)rows;
        }
    }
};

/// Zero-allocation RK4 step using caller-provided workspace storage.
/// Writes the propagated state directly into `x_out` (safe even if `x_out` aliases `x0`).
template <typename Derived, typename StateVec, typename DerivFn,
          std::enable_if_t<(Derived::ColsAtCompileTime == 1) && (StateVec::ColsAtCompileTime == 1) &&
                               is_state_derivative_v<DerivFn, StateVec>,
                           int> = 0>
inline void rk4_step(const Eigen::MatrixBase<Derived>& x0, typename StateVec::Scalar t0, typename StateVec::Scalar dt,
                     DerivFn&& f, Rk4Workspace<StateVec>& ws, StateVec& x_out) {
    using Scalar = typename StateVec::Scalar;

    assert(x0.rows() == ws.x_curr.rows());
    assert(x0.rows() == x_out.rows());

    const Scalar half_t0  = t0 + static_cast<Scalar>(0.5) * dt;
    const Scalar full_t0  = t0 + dt;
    const Scalar half_dt  = static_cast<Scalar>(0.5) * dt;
    const Scalar sixth_dt = dt / static_cast<Scalar>(6.0);

    // Materialize input once (handles Eigen expressions like x.head<6>() safely)
    ws.x_curr.noalias()   = x0;

    // Stage 1
    ws.k1.noalias()       = f(t0, ws.x_curr);

    // Stage 2
    ws.x_stage.noalias()  = ws.x_curr + half_dt * ws.k1;
    ws.k2.noalias()       = f(half_t0, ws.x_stage);

    // Stage 3
    ws.x_stage.noalias()  = ws.x_curr + half_dt * ws.k2;
    ws.k3.noalias()       = f(half_t0, ws.x_stage);

    // Stage 4
    ws.x_stage.noalias()  = ws.x_curr + dt * ws.k3;
    ws.k4.noalias()       = f(full_t0, ws.x_stage);

    // Final weighted sum
    x_out.noalias() =
        ws.x_curr + sixth_dt * (ws.k1 + static_cast<Scalar>(2.0) * ws.k2 + static_cast<Scalar>(2.0) * ws.k3 + ws.k4);
}