pub mod clarke_wright_merge_vrp;

#[cfg(test)]
mod tests {
    use super::*;
    use tig_challenges::{vehicle_routing::*, *};

    #[test]
    fn test_clarke_wright_merge_vrp() {
        let difficulty = Difficulty {
            // Uncomment the relevant fields.
            // Modify the values for different difficulties

            // -- satisfiability --
            // num_variables: 50,
            // clauses_to_variables_percent: 300,

            // -- vehicle_routing --
            num_nodes: 40,
            better_than_baseline: 250,

            // -- knapsack --
            // num_items: 50,
            // better_than_baseline: 10,
            
            // -- vector_search --
            // num_queries: 10,
            // better_than_baseline: 350,
        };
        let seeds = [0; 8]; // change this to generate different instances
        let challenge = Challenge::generate_instance(seeds, &difficulty).unwrap();
        match clarke_wright_merge_vrp::solve_challenge(&challenge) {
            Ok(Some(solution)) => match challenge.verify_solution(&solution) {
                Ok(_) => println!("Valid solution"),
                Err(e) => println!("Invalid solution: {}", e),
            },
            Ok(None) => println!("No solution"),
            Err(e) => println!("Algorithm error: {}", e),
        };
    }
}