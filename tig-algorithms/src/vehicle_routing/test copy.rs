/*!
Copyright 2024 Uncharted Trading Limited

Licensed under the TIG Benchmarker Outbound Game License v1.0 (the "License"); you 
may not use this file except in compliance with the License. You may obtain a copy 
of the License at

https://github.com/tig-foundation/tig-monorepo/tree/main/docs/licenses

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.
*/

use tig_challenges::vehicle_routing::*;

const MAX_ITERATIONS: usize = 1000;

pub fn solve_challenge(challenge: &Challenge) -> anyhow::Result<Option<Solution>> {
    let distance_matrix = &challenge.distance_matrix;
    let capacity = challenge.max_capacity;
    let demands = &challenge.demands;
    let num_nodes = challenge.difficulty.num_nodes;
    let max_total_distance = challenge.max_total_distance as f64;

    // Construct initial solution
    let mut best_solution = construct_initial_solution(num_nodes, capacity, demands, distance_matrix);
    let mut best_fitness = calculate_total_distance(&best_solution, distance_matrix);

    // Improve the solution using local search
    two_opt_optimization(&mut best_solution, distance_matrix);
    best_fitness = calculate_total_distance(&best_solution, distance_matrix);

    // Iterative refinement
    for _ in 0..MAX_ITERATIONS {
        let mut new_solution = best_solution.clone();
        apply_insertion_move(&mut new_solution, demands, capacity, distance_matrix);
        apply_swap_move(&mut new_solution, demands, capacity, distance_matrix);
        two_opt_optimization(&mut new_solution, distance_matrix);

        let new_fitness = calculate_total_distance(&new_solution, distance_matrix);
        if new_fitness < best_fitness {
            best_solution = new_solution;
            best_fitness = new_fitness;
        }

        if best_fitness <= max_total_distance {
            return Ok(Some(Solution { routes: best_solution }));
        }
    }

    Ok(None)
}

fn construct_initial_solution(
    num_nodes: usize,
    capacity: i32,
    demands: &[i32],
    distance_matrix: &[Vec<i32>],
) -> Vec<Vec<usize>> {
    let mut solution = vec![vec![0]];
    let mut visited = vec![false; num_nodes];
    visited[0] = true;
    let mut current_load = 0;

    while visited.iter().any(|&v| !v) {
        let last_node = *solution.last().unwrap().last().unwrap();
        let mut nearest_node = None;
        let mut nearest_distance = i32::MAX;

        for j in 1..num_nodes {
            if !visited[j] && current_load + demands[j] <= capacity {
                let distance = distance_matrix[last_node][j];
                if distance < nearest_distance {
                    nearest_distance = distance;
                    nearest_node = Some(j);
                }
            }
        }

        if let Some(next_node) = nearest_node {
            solution.last_mut().unwrap().push(next_node);
            visited[next_node] = true;
            current_load += demands[next_node];
        } else {
            solution.last_mut().unwrap().push(0);
            solution.push(vec![0]);
            current_load = 0;
        }
    }

    for route in &mut solution {
        if *route.last().unwrap() != 0 {
            route.push(0);
        }
    }

    solution
}

fn calculate_total_distance(solution: &[Vec<usize>], distance_matrix: &[Vec<i32>]) -> f64 {
    solution.iter().map(|route| {
        route.windows(2).map(|pair| distance_matrix[pair[0]][pair[1]]).sum::<i32>()
    }).sum::<i32>() as f64
}

fn two_opt_optimization(solution: &mut Vec<Vec<usize>>, distance_matrix: &[Vec<i32>]) {
    let mut improved = true;

    while improved {
        improved = false;

        for route in solution.iter_mut() {
            let route_len = route.len();
            for i in 1..route_len - 2 {
                for j in i + 1..route_len - 1 {
                    if j - i == 1 {
                        continue;
                    }
                    let delta = distance_matrix[route[i - 1]][route[j]]
                        + distance_matrix[route[i]][route[j + 1]]
                        - distance_matrix[route[i - 1]][route[i]]
                        - distance_matrix[route[j]][route[j + 1]];
                    if delta < 0 {
                        route[i..=j].reverse();
                        improved = true;
                    }
                }
            }
        }
    }
}

fn apply_insertion_move(
    solution: &mut Vec<Vec<usize>>,
    demands: &[i32],
    capacity: i32,
    distance_matrix: &[Vec<i32>],
) {
    let mut best_delta = 0;
    let mut best_move = None;

    for route_idx in 0..solution.len() {
        let route = &solution[route_idx];
        for i in 1..route.len() - 1 {
            let node = route[i];
            for new_route_idx in 0..solution.len() {
                if new_route_idx == route_idx {
                    continue;
                }
                let new_route = &solution[new_route_idx];
                let new_load: i32 = new_route.iter().map(|&n| demands[n]).sum();
                if new_load + demands[node] > capacity {
                    continue;
                }
                for j in 1..new_route.len() {
                    let delta = distance_matrix[route[i - 1]][route[i + 1]] - distance_matrix[route[i - 1]][node] - distance_matrix[node][route[i + 1]]
                        + distance_matrix[new_route[j - 1]][node] + distance_matrix[node][new_route[j]] - distance_matrix[new_route[j - 1]][new_route[j]];

                    if delta < best_delta {
                        best_delta = delta;
                        best_move = Some((route_idx, i, new_route_idx, j));
                    }
                }
            }
        }
    }

    if let Some((from_route_idx, from_idx, to_route_idx, to_idx)) = best_move {
        let node = solution[from_route_idx].remove(from_idx);
        solution[to_route_idx].insert(to_idx, node);
    }
}

fn apply_swap_move(
    solution: &mut Vec<Vec<usize>>,
    demands: &[i32],
    capacity: i32,
    distance_matrix: &[Vec<i32>],
) {
    let mut best_delta = 0;
    let mut best_move = None;

    for route_idx1 in 0..solution.len() {
        for i in 1..solution[route_idx1].len() - 1 {
            let node1 = solution[route_idx1][i];
            for route_idx2 in route_idx1..solution.len() {
                for j in if route_idx1 == route_idx2 { i + 1 } else { 1 }..solution[route_idx2].len() - 1 {
                    let node2 = solution[route_idx2][j];

                    if route_idx1 != route_idx2 {
                        let load1: i32 = solution[route_idx1].iter().map(|&n| demands[n]).sum();
                        let load2: i32 = solution[route_idx2].iter().map(|&n| demands[n]).sum();
                        if load1 - demands[node1] + demands[node2] > capacity || load2 - demands[node2] + demands[node1] > capacity {
                            continue;
                        }
                    }

                    let delta = distance_matrix[solution[route_idx1][i - 1]][node2] + distance_matrix[node2][solution[route_idx1][i + 1]] - distance_matrix[solution[route_idx1][i - 1]][node1] - distance_matrix[node1][solution[route_idx1][i + 1]]
                        + distance_matrix[solution[route_idx2][j - 1]][node1] + distance_matrix[node1][solution[route_idx2][j + 1]] - distance_matrix[solution[route_idx2][j - 1]][node2] - distance_matrix[node2][solution[route_idx2][j + 1]];

                    if delta < best_delta {
                        best_delta = delta;
                        best_move = Some((route_idx1, i, route_idx2, j));
                    }
                }
            }
        }
    }

    if let Some((route_idx1, i, route_idx2, j)) = best_move {
        solution[route_idx1][i] = solution[route_idx2][j];
        solution[route_idx2][j] = solution[route_idx1][i];
    }
}

fn main() {
    let challenge = Challenge {
        seed: 0,
        distance_matrix: vec![vec![0, 10, 20, 30, 40], vec![10, 0, 15, 25, 35], vec![20, 15, 0, 20, 30], vec![30, 25, 20, 0, 10], vec![40, 35, 30, 10, 0]],
        demands: vec![0, 25, 30, 40, 50],
        max_capacity: 100,
        max_total_distance: 140,
        difficulty: Difficulty { num_nodes: 5, better_than_baseline: 8 },
    };

    let result = solve_challenge(&challenge);
    match result {
        Ok(Some(solution)) => println!("Solution found: {:?}", solution.routes),
        Ok(None) => println!("No valid solution found"),
        Err(e) => println!("Error: {:?}", e),
    }
}
