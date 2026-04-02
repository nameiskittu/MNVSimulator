from scipy.integrate import solve_ivp
import numpy as np

def run_straight_line(ship, init_state, n_cmd, t_end=200):
    # BUG FIX 3: t_end was ignored — t_span and t_eval were hardcoded to 200.
    # Now uses the t_end parameter correctly.
    t_span_sl = (0, t_end)
    t_eval_sl = np.linspace(0, t_end, max(2000, int(t_end * 10)))
    sol_sl    = solve_ivp(ship.dynamics, t_span_sl, init_state,
                          args=(n_cmd, 0.0), t_eval=t_eval_sl)
    return sol_sl