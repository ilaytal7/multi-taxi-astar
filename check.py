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

## Example Input
Below is a grid-world configuration you can run with `TaxiProblem`:

     'map': [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'G', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P']],
        'taxis': {'taxi 1': {'location': (2, 0), 'fuel': 5, 'capacity': 2},
                  'taxi 2': {'location': (0, 1), 'fuel': 6, 'capacity': 2}},
        'passengers': {'Iris': {'location': (0, 0), 'destination': (1, 4)},
                                'Tomer': {'location': (3, 1), 'destination': (2, 1)},
                                'Sahar': {'location': (2, 3), 'destination': (2, 4)},
                                'Yarin': {'location': (3, 0), 'destination': (3, 2)}}
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
   - Manhattan distances never overestimate.  
   - Closest-taxi distances are the minimum possible pickup costs.  
   - Capacity penalty reflects unavoidable extra trips.  
   - Task counts are minimal required steps.  
   Therefore, `h` is **admissible** and guarantees that A* will find the optimal solution.

---

## Repository Layout
- `ex1.py` — Core problem definition + heuristics.
- `check.py` — Runner with example scenarios for a quick demo.
- `search.py` / `utils.py` — AIMA search helpers.

## Demo
Run locally:
```bash
python check.py
```

Sample output:
```
Solution: ['move east', 'pick up Alice', 'move south', 'drop off Alice']
Path length: 4
```
