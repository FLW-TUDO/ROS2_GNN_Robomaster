"""Final thesis numerical contracts for per-plan PUC and executed-route cost."""
import math


KEYS = ('t', 'o', 'c', 's', 'e')
PUC_WEIGHTS = dict(zip(KEYS, (1.0, 1.0, 1.0, 2.0, 0.5)))
TIME_BUDGET_MULTIPLE = 1.207356997594271
ENERGY_ALPHA_WH_PER_M = 0.009803992713305888
ENERGY_BETA_WH_PER_TURN = 0.003177606424396346
ROUTE_COEFFICIENTS = {
    'time': 0.21000911811244674,
    'obs': 2.2120576740455227,
    'comms': 1.0751520482905212,
    'safety': 8.432645250901363,
    'energy': 0.37028242098654696,
}


def time_confidence(length_m, modeled_time_s, jitter_ms=None,
                    budget_multiple=TIME_BUDGET_MULTIPLE, v_ref_mps=0.6,
                    sigma_per_meter=0.05, sigma_per_ms_jitter=0.01):
    if not all(math.isfinite(x) for x in (length_m, modeled_time_s)):
        return math.nan
    if length_m < 0 or modeled_time_s < 0 or v_ref_mps <= 0 or budget_multiple <= 0:
        raise ValueError('Time inputs and budget must be physically valid')
    jitter = jitter_ms if jitter_ms is not None and math.isfinite(jitter_ms) else 0.0
    sigma = max(1e-3, sigma_per_meter * length_m + sigma_per_ms_jitter * jitter)
    budget = budget_multiple * length_m / v_ref_mps
    return 0.5 * math.erfc(-(budget - modeled_time_s) / sigma / math.sqrt(2.0))


def energy_confidence(length_m, n_turns, soc_fraction,
                      alpha=ENERGY_ALPHA_WH_PER_M, beta=ENERGY_BETA_WH_PER_TURN,
                      capacity_wh=25.92, reserve_wh=5.184000000000001, scale_wh=5.0):
    if soc_fraction is None or not math.isfinite(soc_fraction):
        return math.nan
    if not 0 <= soc_fraction <= 1 or length_m < 0 or n_turns < 0 or scale_wh <= 0:
        raise ValueError('Energy inputs must be physically valid')
    margin = (soc_fraction * capacity_wh - alpha * length_m - beta * n_turns - reserve_wh) / scale_wh
    if margin >= 0:
        return 1.0 / (1.0 + math.exp(-margin))
    exp_margin = math.exp(margin)
    return exp_margin / (1.0 + exp_margin)


def fuse_puc(components, weights=None):
    weights = PUC_WEIGHTS if weights is None else weights
    if any(not math.isfinite(weights[k]) or weights[k] < 0 for k in KEYS):
        raise ValueError('PUC weights must be finite and non-negative')
    total_weight = sum(weights[k] for k in KEYS)
    if total_weight <= 0:
        raise ValueError('At least one PUC weight must be positive')
    terms = []
    for k in KEYS:
        if weights[k] == 0:
            continue
        value = components[k]
        if value is None or not math.isfinite(value):
            return math.nan
        if not 0 <= value <= 1:
            raise ValueError('PUC components must be probabilities')
        terms.append(weights[k] * math.log(max(1e-6, value)))
    return math.exp(sum(terms) / total_weight)


def executed_route_cost(raw_terms, coefficients=None):
    """Aggregate the five supported raw terms of a completed execution interval.

    Energy here is the geometric effort proxy L + 0.5 * turn_events, not Wh.
    Per-plan predicted terms must not be substituted for executed-interval terms.
    """
    coefficients = ROUTE_COEFFICIENTS if coefficients is None else coefficients
    if any(raw_terms[k] is None or not math.isfinite(raw_terms[k]) for k in ROUTE_COEFFICIENTS):
        return math.nan
    return sum(raw_terms[k] * coefficients[k] for k in ROUTE_COEFFICIENTS)
