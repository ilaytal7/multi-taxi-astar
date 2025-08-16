# Multi-Taxi Planning with A* 

This project implements a heuristic search solution (**A*** algorithm) for a deterministic **multi-taxi transportation problem**.
The objective is to coordinate taxis on a grid to **pick up** and **drop off** passengers while respecting **fuel** and **capacity** constraints.

## Problem Statement
- Grid cells can be: free, **I** (wall), or **G** (gas station).
- Each taxi has a location `(x, y)`, `fuel`, and `capacity`.
- Each passenger has an origin and destination.
- A step is either a single taxi action or a **tuple** of actions (one per taxi), executed simultaneously (no collisions).
- **Goal:** deliver all passengers to their destinations.

## Algorithms
- **A\*** Search over the state space using AIMA helpers (`search.py`, `utils.py`).

### Heuristics
The implementation includes three heuristics:

1. **`h_1` (Baseline)**  
   \[
   h_1 = \frac{2 \cdot |U'| + |P_{undelivered}|}{|T|}
   \]  
   Where \(U'\) = number of unpicked passengers,  
   \(P_{undelivered}\) = passengers picked up but not delivered,  
   \(T\) = number of taxis.  

   A simple admissible lower bound, counting actions needed at minimum.

2. **`h_2` (Manhattan-based)**  
   \[
   h_2 = \frac{\sum_{p \in P} \text{dist}(origin_p, dest_p) + \text{dist}(current_p, dest_p)}{|T|}
   \]  
   Uses Manhattan distances from passenger locations to destinations.

3. **`h` (Custom heuristic)**  
   This heuristic was designed to give a tighter lower bound by combining multiple signals:

   - **Passenger-to-destination distance (average):**  
     \[
     \frac{\sum_{p \in U} \text{dist}(loc_p, dest_p)}{|U|}
     \]
     where \(U\) = set of undelivered passengers.

   - **Pickup effort (average distance from unpicked passengers to closest taxi):**  
     \[
     \frac{\sum_{p \in U'} \min_{t \in T} \text{dist}(loc_p, loc_t)}{|U'|}
     \]
     where \(U'\) = set of unpicked passengers, \(T\) = set of taxis.

   - **Capacity shortfall penalty:**  
     \[
     \Delta_{capacity} = \max \Big(0, |U'| - \sum_{t \in T} capacity_t \Big)
     \]  
     Ensures that if there are more unpicked passengers than available capacity, the heuristic reflects the need for additional trips.

   - **Counts of remaining tasks:**  
     \[
     |U| + |U'|
     \]
     Stabilizes the estimate by directly counting outstanding passengers.

   Putting it together:  
   \[
   h = \frac{\sum_{p \in U} \text{dist}(loc_p, dest_p)}{|U|}
     + \frac{\sum_{p \in U'} \min_{t \in T} \text{dist}(loc_p, loc_t)}{|U'|}
     + \Delta_{capacity}
     + |U| + |U'|
   \]

   ✅ **Admissibility:**  
   - Manhattan distances are always lower bounds in a grid.  
   - Closest-taxi distance is the minimum possible pickup effort.  
   - The capacity penalty only acknowledges unavoidable extra trips.  
   - Passenger counts cannot overestimate cost.  
   Therefore, `h` never exceeds the true minimal cost to reach the goal, making it **admissible**.

---

## Repository Layout
- `ex1.py` — Core problem definition + heuristics.
- `check.py` — Runner with example scenarios for a quick demo.
- `search.py` / `utils.py` — AIMA search helpers.

## Demo
```bash
python check.py
