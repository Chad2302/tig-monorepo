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
use rand::seq::SliceRandom;
use rand::thread_rng;
use rand::Rng;

const INITIAL_TEMPERATURE: f64 = 1000.0;
const COOLING_RATE: f64 = 0.995;
const MIN_TEMPERATURE: f64 = 1e-3;
const ITERATIONS_PER_TEMP: usize = 100;

pub fn solve_challenge(challenge: &Challenge) -> anyhow::Result<Option<Solution>> {
    let distance_matrix = &challenge.distance_matrix;
    let capacity = challenge.max_capacity;
    let demands = &challenge.demands;
    let n = challenge.difficulty.num_nodes;

    let mut current_solution = initialize_solution(n, capacity, demands);
    let mut current_fitness = calculate_fitness(&current_solution, distance_matrix);

    let mut best_solution = current_solution.clone();
    let mut best_fitness = current_fitness;

    let mut temperature = INITIAL_TEMPERATURE;
    let mut rng = thread_rng();

    while temperature > MIN_TEMPERATURE {
        for _ in 0..ITERATIONS_PER_TEMP {
            let neighbor_solution = generate_neighbor(&current_solution, capacity, demands);
            let neighbor_fitness = calculate_fitness(&neighbor_solution, distance_matrix);

            let acceptance_probability = if neighbor_fitness < current_fitness {
                1.0
            } else {
                ((current_fitness - neighbor_fitness) / temperature).exp()
            };

            if rng.gen::<f64>() < acceptance_probability {
                current_solution = neighbor_solution;
                current_fitness = neighbor_fitness;

                if current_fitness < best_fitness {
                    best_solution = current_solution.clone();
                    best_fitness = current_fitness;
                }
            }
        }

        temperature *= COOLING_RATE;
    }

    Ok(Some(Solution {
        routes: best_solution,
    }))
}

fn initialize_solution(n: usize, capacity: i32, demands: &[i32]) -> Vec<Vec<usize>> {
    let mut nodes: Vec<usize> = (1..n).collect();
    let mut rng = thread_rng();
    nodes.shuffle(&mut rng);

    let mut routes = vec![vec![0]];
    let mut current_load = 0;

    for &node in &nodes {
        if current_load + demands[node] > capacity {
            routes.push(vec![0]);
            current_load = 0;
        }
        routes.last_mut().unwrap().push(node);
        current_load += demands[node];
    }
    for route in &mut routes {
        route.push(0);
    }

    routes
}

fn calculate_fitness(routes: &[Vec<usize>], distance_matrix: &[Vec<i32>]) -> f64 {
    routes.iter().map(|route| {
        route.windows(2).map(|pair| distance_matrix[pair[0]][pair[1]]).sum::<i32>()
    }).sum::<i32>() as f64
}

fn generate_neighbor(solution: &Vec<Vec<usize>>, capacity: i32, demands: &[i32]) -> Vec<Vec<usize>> {
    let mut rng = thread_rng();
    let mut new_solution = solution.clone();

    let route_idx = rng.gen_range(0..new_solution.len());
    let route_len = new_solution[route_idx].len();

    if route_len > 3 {
        let node_idx = rng.gen_range(1..(route_len - 1));
        let new_idx = rng.gen_range(1..(route_len - 1));

        new_solution[route_idx].swap(node_idx, new_idx);
    }

    let mut valid_solution = vec![vec![0]];
    let mut current_load = 0;

    for &node in &new_solution[route_idx] {
        if current_load + demands[node] > capacity {
            valid_solution.push(vec![0]);
            current_load = 0;
        }
        valid_solution.last_mut().unwrap().push(node);
        current_load += demands[node];
    }
    for route in &mut valid_solution {
        route.push(0);
    }

    valid_solution
}
