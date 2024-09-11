pub mod test;

#[cfg(test)]
mod tests {
    use super::*;
    use tig_challenges::{knapsack::*, *};

    #[test]
    fn test_test() {
        let difficulty = Difficulty {
            // Uncomment the relevant fields.
            // Modify the values for different difficulties

            // -- satisfiability --
            // num_variables: 50,
            // clauses_to_variables_percent: 300,

            // -- vehicle_routing --
            // num_nodes: 40,
            // better_than_baseline: 250,
            // -- knapsack --
            num_items: 50,
            better_than_baseline: 10,
        };
        let seed = 0; // change this to generate different instances
        let challenge = Challenge::generate_instance(seed, &difficulty).unwrap();
        match test::solve_challenge(&challenge) {
            Ok(Some(solution)) => match challenge.verify_solution(&solution) {
                Ok(_) => println!("Valid solution"),
                Err(e) => println!("Invalid solution: {}", e),
            },
            Ok(None) => println!("No solution"),
            Err(e) => println!("Algorithm error: {}", e),
        };
    }
}
