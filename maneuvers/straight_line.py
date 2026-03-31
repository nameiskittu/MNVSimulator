from scipy.integrate import solve_ivp
import numpy as np

def run_straight_line(ship, init_state, n_cmd, t_end=200):
    t_span_sl = (0, 200)
    t_eval_sl = np.linspace(0, 200, 2000)
    sol_sl    = solve_ivp(ship.dynamics, t_span_sl, init_state,
                          args=(n_cmd, 0.0), t_eval=t_eval_sl)
    return sol_sl