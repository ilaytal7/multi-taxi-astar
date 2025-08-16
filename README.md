# Multi-Taxi Planning with A*

The **main goal** of this project is to coordinate multiple taxis so that **all passengers reach their destinations with the minimum number of steps**.  
We use the **A\*** search algorithm to guarantee an **optimal solution**.

## Problem Statement
- Grid cells can be: free, **I** (wall), or **G** (gas station).
- Each taxi has a location `(x, y)`, `fuel`, and `capacity`.
- Each passenger has an origin and a destination.
- An action step is either a single taxi action or a **tuple** of actions (one per taxi), executed simultaneously (no collisions).
- **Goal:** deliver all passengers to their destinations optimally.

---
## Demo 
## Example Input
Below is a grid-world configuration you can run with `TaxiProblem`:

    {
        'map': [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'G', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P']],
        'taxis': {'taxi 1': {'location': (2, 0), 'fuel': 5, 'capacity': 2},
                  'taxi 2': {'location': (0, 1), 'fuel': 6, 'capacity': 2}},
        'passengers': {'Iris': {'location': (0, 0), 'destination': (1, 4)},
                       'Daniel': {'location': (3, 1), 'destination': (2, 1)},
                       'Freyja': {'location': (2, 3), 'destination': (2, 4)},
                       'Tamar': {'location': (3, 0), 'destination': (3, 2)}},
    }

## Example Output
A*: (number of moves, runtime , [(taxi_1 action, location), (taxi_2 action, location) , ....]           ->
A*:  (13, 0.031313180923461914, [(('move', 'taxi 1', (3, 0)), ('move', 'taxi 2', (0, 0))), (('pick up', 'taxi 1', 'Tamar'), ('pick up', 'taxi 2', 'Iris')), (('move', 'taxi 1', (3, 1)), ('move', 'taxi 2', (0, 1))), (('move', 'taxi 1', (3, 2)), ('move', 'taxi 2', (0, 2))), (('drop off', 'taxi 1', 'Tamar'), ('move', 'taxi 2', (0, 3))), (('move', 'taxi 1', (3, 1)), ('move', 'taxi 2', (1, 3))), (('pick up', 'taxi 1', 'Daniel'), ('refuel', 'taxi 2')), (('move', 'taxi 1', (2, 1)), ('move', 'taxi 2', (2, 3))), (('drop off', 'taxi 1', 'Daniel'), ('pick up', 'taxi 2', 'Freyja')), (('wait', 'taxi 1'), ('move', 'taxi 2', (2, 4))), (('wait', 'taxi 1'), ('drop off', 'taxi 2', 'Freyja')), (('wait', 'taxi 1'), ('move', 'taxi 2', (1, 4))), (('wait', 'taxi 1'), ('drop off', 'taxi 2', 'Iris'))])

---

## Algorithms
- **A\*** Search over the state space using AIMA helpers (`search.py`, `utils.py`).

### Heuristics
The implementation includes three heuristics:

1. **h1 (Baseline)**  
   ```
   h1 = (2 * num_unpicked + num_picked_not_delivered) / num_taxis
   ```
   - Simple lower bound that counts the minimum number of required actions.  
   - Admissible because it never overestimates.

2. **h2 (Manhattan-based)**  
   ```
   h2 = (sum of Manhattan distances from passenger origins to destinations
         + sum of distances from current passenger locations to destinations)
         / num_taxis
   ```
   - Estimates effort based on Manhattan distances.  
   - Admissible because Manhattan distance ≤ true path cost in a grid.

3. **h (Custom heuristic)**  
   Designed to be a **tighter admissible heuristic**, combining multiple signals:  
   - **Average passenger-to-destination distance**: average remaining distance for undelivered passengers.  
   - **Pickup effort**: average distance from unpicked passengers to their closest taxi.  
   - **Capacity shortfall penalty**: if total taxi capacity < number of unpicked passengers, add a penalty.  
   - **Counts of remaining tasks**: adds the number of unpicked + undelivered passengers as stabilizers.

   Formula (simplified):  
   ```
   h = avg_distance_to_dest
     + avg_distance_to_closest_taxi
     + capacity_shortfall
     + num_undelivered
     + num_unpicked
   ```

   ✅ **Admissibility:**  
   - Average distance never overestimates (because at least one taxi must travel the maximum distance, which is always greater than or equal to the average distance).   -      - Closest-taxi distances are the minimum possible pickup costs.  
   - Capacity penalty reflects unavoidable extra trips.  
   - Task counts are minimal required steps.  
   Therefore, `h` is **admissible** and guarantees that A* will find the optimal solution.

---

## Repository Layout
- `ex1.py` — Core problem definition + heuristics.
- `check.py` — Runner with example scenarios for a quick demo.
- `search.py` / `utils.py` — AIMA search helpers.


```
