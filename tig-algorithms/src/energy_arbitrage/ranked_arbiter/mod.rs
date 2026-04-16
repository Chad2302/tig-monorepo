// TIG's UI uses the pattern `tig_challenges::<challenge_name>` to automatically detect your algorithm's challenge
use anyhow::{anyhow, Result};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use tig_challenges::energy_arbitrage::*;

#[derive(Serialize, Deserialize)]
pub struct Hyperparameters {}

pub fn help() {
    println!("v12_unified: Efficiency-adjusted order-statistic arbitrage with sigma-aware spike detection,");
    println!("exact congestion probability, expected RT ranking, and spike-size-weighted reserve management.");
}

pub fn solve_challenge(
    challenge: &Challenge,
    save_solution: &dyn Fn(&Solution) -> Result<()>,
    _hyperparameters: &Option<Map<String, Value>>,
) -> Result<()> {
    let cong_prob = precompute_exact_congestion_prob(challenge);
    let expected_rt = precompute_expected_rt(challenge, &cong_prob);

    let solution = challenge.grid_optimize(&|c, s| {
        unified_policy(c, s, &expected_rt, &cong_prob)
    })?;
    save_solution(&solution)?;
    Ok(())
}

// ─── Pre-computation ────────────────────────────────────────────────────────

const EPS: f64 = 1e-12;

/// Exact congestion probability per node per timestep using the actual model formula.
/// congestion_prob[t][node] = probability that RT prices at step t+1 see congestion at node.
fn precompute_exact_congestion_prob(challenge: &Challenge) -> Vec<Vec<f64>> {
    let num_nodes = challenge.network.num_nodes;
    let num_steps = challenge.num_steps;
    let tau = challenge.network.congestion_threshold;

    let mut cong_prob = vec![vec![0.0; num_nodes]; num_steps];

    for t in 0..num_steps {
        let inj = &challenge.exogenous_injections[t];
        let mut full_inj = inj.clone();
        let mut sum = 0.0;
        for i in 0..num_nodes {
            if i != challenge.network.slack_bus { sum += full_inj[i]; }
        }
        full_inj[challenge.network.slack_bus] = -sum;

        let mut line_prob = vec![0.0f64; challenge.network.num_lines];
        for l in 0..challenge.network.num_lines {
            let flow: f64 = (0..num_nodes)
                .map(|k| challenge.network.ptdf[l][k] * full_inj[k])
                .sum();
            let ratio = flow.abs() / (tau * challenge.network.flow_limits[l]);
            line_prob[l] = ratio.powf(10.0).min(1.0);
        }

        for node in 0..num_nodes {
            let mut prob_not_congested = 1.0;
            for &l in &challenge.network.node_incident_lines[node] {
                prob_not_congested *= 1.0 - line_prob[l];
            }
            cong_prob[t][node] = 1.0 - prob_not_congested;
        }
    }
    cong_prob
}

/// Compute expected RT price at each (step, node) pair.
/// NOTE: cong_prob[t] predicts congestion for step t+1's prices.
fn precompute_expected_rt(challenge: &Challenge, cong_prob: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let num_nodes = challenge.network.num_nodes;
    let num_steps = challenge.num_steps;
    let da = &challenge.market.day_ahead_prices;
    let rho_jump = challenge.market.params.jump_probability;
    let alpha = challenge.market.params.tail_index;

    let expected_pareto = if alpha > 1.0 { alpha / (alpha - 1.0) } else { 10.0 };
    let expected_half_normal = 1.0 / (2.0 * std::f64::consts::PI).sqrt();

    let mut expected_rt = vec![vec![0.0; num_nodes]; num_steps];

    for t in 0..num_steps {
        for n in 0..num_nodes {
            let da_price = da[t][n];
            let base = da_price;
            let spike_premium = rho_jump * da_price * expected_pareto;
            let cong_premium = if t > 0 {
                cong_prob[t - 1][n] * constants::GAMMA_PRICE * expected_half_normal
            } else {
                0.0
            };
            expected_rt[t][n] = base + spike_premium + cong_premium;
        }
    }
    expected_rt
}

// ─── Flow Feasibility ──────────────────────────────────────────────────────

const MAX_FLOW_ADJUST_ITERS: usize = 64;
const GLOBAL_SCALE_BSEARCH_ITERS: usize = 32;

#[derive(Clone, Copy)]
struct Violation { line: usize, flow: f64, amount: f64 }

fn compute_flows(challenge: &Challenge, state: &State, action: &[f64]) -> Vec<f64> {
    let injections = challenge.compute_total_injections(state, action);
    (0..challenge.network.num_lines)
        .map(|l| (0..challenge.network.num_nodes)
            .map(|k| challenge.network.ptdf[l][k] * injections[k]).sum::<f64>())
        .collect()
}

fn most_violated_line(challenge: &Challenge, flows: &[f64]) -> Option<Violation> {
    let mut best: Option<Violation> = None;
    for (l, &flow) in flows.iter().enumerate() {
        let limit = challenge.network.flow_limits[l];
        let v = flow.abs() - limit;
        if v > constants::EPS_FLOW * limit {
            let c = Violation { line: l, flow, amount: v };
            match best { Some(b) if c.amount <= b.amount => {}, _ => best = Some(c) }
        }
    }
    best
}

fn is_flow_feasible(challenge: &Challenge, state: &State, action: &[f64]) -> bool {
    most_violated_line(challenge, &compute_flows(challenge, state, action)).is_none()
}

fn soften(challenge: &Challenge, v: Violation, action: &mut [f64]) -> bool {
    let dir = v.flow.signum();
    if dir.abs() <= EPS { return false; }
    let mut idx = Vec::new();
    let mut s = 0.0;
    for (i, b) in challenge.batteries.iter().enumerate() {
        let c = dir * challenge.network.ptdf[v.line][b.node] * action[i];
        if c > EPS { s += c; idx.push(i); }
    }
    if idx.is_empty() || s <= EPS { return false; }
    let keep = (1.0 - v.amount / s).clamp(0.0, 1.0);
    if (1.0 - keep).abs() <= EPS { return false; }
    for i in idx { action[i] *= keep; }
    true
}

fn enforce_flow(challenge: &Challenge, state: &State, mut action: Vec<f64>) -> Result<Vec<f64>> {
    for _ in 0..MAX_FLOW_ADJUST_ITERS {
        let flows = compute_flows(challenge, state, &action);
        let Some(v) = most_violated_line(challenge, &flows) else { return Ok(action); };
        if !soften(challenge, v, &mut action) { break; }
    }
    if is_flow_feasible(challenge, state, &action) { return Ok(action); }
    let zero = vec![0.0; action.len()];
    if !is_flow_feasible(challenge, state, &zero) {
        return Err(anyhow!("Grid infeasible with zero actions"));
    }
    let base = action;
    let (mut lo, mut hi) = (0.0, 1.0);
    for _ in 0..GLOBAL_SCALE_BSEARCH_ITERS {
        let mid = 0.5 * (lo + hi);
        let scaled: Vec<f64> = base.iter().map(|u| mid * u).collect();
        if is_flow_feasible(challenge, state, &scaled) { lo = mid; } else { hi = mid; }
    }
    Ok(base.into_iter().map(|u| lo * u).collect())
}

// ─── Unified Policy ─────────────────────────────────────────────────────────

fn unified_policy(
    challenge: &Challenge,
    state: &State,
    expected_rt: &[Vec<f64>],
    cong_prob: &[Vec<f64>],
) -> Result<Vec<f64>> {
    let t = state.time_step;
    let ns = challenge.num_steps;
    let da = &challenge.market.day_ahead_prices;
    let remaining = ns - t;
    let sigma = challenge.market.params.volatility;
    let jump_prob = challenge.market.params.jump_probability;
    let alpha = challenge.market.params.tail_index;
    let eta_rt = constants::ETA_CHARGE * constants::ETA_DISCHARGE;

    let mut action = vec![0.0; challenge.num_batteries];

    for (i, bat) in challenge.batteries.iter().enumerate() {
        let node = bat.node;
        let (lb, ub) = state.action_bounds[i];
        let soc = state.socs[i];
        let rt = state.rt_prices[node];
        let cda = da[t][node];
        let sr = bat.soc_max_mwh - bat.soc_min_mwh;
        let sf = if sr > EPS { (soc - bat.soc_min_mwh) / sr } else { 0.5 };

        // === LAYER 1: Sigma-aware spike detection ===
        // SOC-dependent: more willing to sell with high SOC (lower k)
        let spike_k = 2.0 + 2.0 * (1.0 - sf);
        let spike_threshold = if cda > EPS {
            (cda * (1.0 + spike_k * sigma)).max(cda + 20.0)
        } else {
            100.0
        };
        if rt > spike_threshold && sf > 0.05 {
            action[i] = ub;
            continue;
        }

        // === LAYER 2: Negative/very low price ===
        if rt < -5.0 && sf < 0.95 {
            action[i] = lb;
            continue;
        }

        // === LAYER 3: End-of-horizon discharge ===
        if remaining <= 10 && sf > 0.08 {
            let urgency = (1.0 - remaining as f64 / 10.0).max(0.0);
            let min_price = constants::KAPPA_TX + (1.0 - urgency) * 5.0;
            if rt > min_price || (remaining <= 3 && sf > 0.12) {
                action[i] = ub;
                continue;
            }
        }

        // === LAYER 4: Congestion premium capture (lagged indicator) ===
        if t > 0 {
            let cong_now = cong_prob[t - 1][node];
            if cong_now > 0.3 && rt > cda + 5.0 && sf > 0.15 {
                action[i] = ub;
                continue;
            }
        }

        // === LAYER 5: Efficiency-adjusted order-statistic thresholds ===
        let discharge_energy_per_step = ub * constants::DELTA_T / bat.efficiency_discharge;
        let discharge_steps = if discharge_energy_per_step > EPS {
            ((soc - bat.soc_min_mwh) / discharge_energy_per_step).floor() as usize
        } else { 0 };

        let charge_energy_per_step = (-lb) * bat.efficiency_charge * constants::DELTA_T;
        let charge_steps = if charge_energy_per_step > EPS {
            ((bat.soc_max_mwh - soc) / charge_energy_per_step).floor() as usize
        } else { 0 };

        if remaining == 0 { continue; }

        // Sort remaining expected RT prices for this node
        let mut future_prices: Vec<f64> = (t..ns).map(|s| expected_rt[s][node]).collect();
        future_prices.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // Discharge if current expected RT is among top discharge_steps prices
        let discharge_rank = remaining.saturating_sub(discharge_steps).min(remaining - 1);
        let charge_rank = charge_steps.min(remaining - 1);
        let discharge_threshold = future_prices[discharge_rank];
        let charge_threshold = future_prices[charge_rank];

        // Efficiency-adjusted charge threshold:
        // Only charge if selling later at the discharge threshold is profitable
        let effective_charge_threshold = if discharge_threshold > 0.0 {
            let max_profitable_charge = (discharge_threshold * bat.efficiency_discharge
                - 2.0 * constants::KAPPA_TX)
                * bat.efficiency_charge;
            charge_threshold.min(max_profitable_charge)
        } else {
            charge_threshold
        };

        // Spike-size-weighted reserve: scale with expected spike magnitude
        let expected_spikes = remaining as f64 * jump_prob;
        let expected_spike_size = if alpha > 1.0 { alpha / (alpha - 1.0) } else { 5.0 };
        let reserve_frac = if remaining < 8 {
            0.0
        } else {
            (0.03 * (expected_spikes * expected_spike_size).sqrt()).min(0.20)
        };

        let current_ert = expected_rt[t][node];

        if current_ert <= effective_charge_threshold && sf < 0.92 {
            action[i] = lb; // Full power charge
        } else if current_ert >= discharge_threshold && sf > reserve_frac + 0.05 {
            action[i] = ub; // Full power discharge
        } else {
            // Hold zone: efficiency-aware spread trading
            let spread = rt - cda;
            let min_profitable_spread =
                2.0 * constants::KAPPA_TX / eta_rt + cda * (1.0 - eta_rt);
            let spread_threshold = min_profitable_spread.max(3.0);
            if spread > spread_threshold && sf > reserve_frac + 0.08 {
                action[i] = ub * (spread / (spread_threshold * 5.0)).min(1.0);
            } else if spread < -spread_threshold && sf < 0.88 {
                action[i] = lb * (-spread / (spread_threshold * 5.0)).min(1.0);
            }
        }

        action[i] = action[i].clamp(lb, ub);
    }

    enforce_flow(challenge, state, action)
}
