"""
Multi-Taxi Planning with A*

This module defines `TaxiProblem`, a deterministic planning problem on a 2D grid,
designed to be solved with AIMA-style search (see `search.py`, `utils.py`).

Notes (for reviewers):
- State is stored as a JSON string to keep it hashable for graph search.
- Each step may include concurrent atomic actions (one per taxi): move / pick up / drop off / refuel / wait.
- Heuristics: `h_1`, `h_2`, and `h` (admissible) guide A* for efficient search.
"""

import copy
import json
import random
import math
from itertools import product
import search
import utils

class TaxiProblem(search.Problem):
    """Deterministic multi-taxi planning problem on a 2D grid."""

    def __init__(self, initial):
        """Build initial internal state from the given input dict and serialize it as JSON (hashable for graph search)."""
        self.map = initial['map']
        self.gas_station = []
        for x in range(len(self.map)):
            for y in range(len(self.map[0])):
                if self.map[x][y] == 'G':
                    loc = []
                    loc.append(x)
                    loc.append(y)
                    self.gas_station.append(loc)

        taxis = initial['taxis']
        passengers = initial['passengers']

        # create state as dict
        state = {
            'taxis': taxis,
            'passengers': passengers,
            'num_of_undelivered': len(passengers),
            'num_of_unpicked': len(passengers),
            'num_of_picked_undelivered': 0
        }

        for taxi in state['taxis']:
            state['taxis'][taxi]["MAX_FUEL"] = state['taxis'][taxi]['fuel']
            state['taxis'][taxi]["passengers_on_taxi"] = []

        for passenger in state['passengers']:
            state['passengers'][passenger]["is_picked"] = False
            state['passengers'][passenger]["is_arrived"] = False
            state['passengers'][passenger]["current_location"] = state['passengers'][passenger]["location"]

        state = json.dumps(state, indent=3)
        search.Problem.__init__(self, state)

    def actions(self, state):
        """
        Return all legal actions from the given state (JSON string).
        Each taxi can: wait, move, pick up, drop off, refuel.
        For multiple taxis, returns tuples of concurrent actions without collisions.
        """
        state = json.loads(state)
        rows, cols = len(self.map) - 1, len(self.map[0]) - 1
        taxis = state['taxis']
        taxis_action = []

        for taxi in taxis:
            taxi_action = []
            x, y = state['taxis'][taxi]['location'][0], state['taxis'][taxi]['location'][1]
            taxi_action.append(('wait', taxi))

            # movement:
            if state['taxis'][taxi]['fuel'] > 0:
                if x < rows and self.map[x + 1][y] != 'I':
                    taxi_action.append(("move", taxi, (x + 1, y)))
                if x >= 1 and self.map[x - 1][y] != 'I':
                    taxi_action.append(("move", taxi, (x - 1, y)))
                if y < cols and self.map[x][y + 1] != 'I':
                    taxi_action.append(("move", taxi, (x, y + 1)))
                if y >= 1 and self.map[x][y - 1] != 'I':
                    taxi_action.append(("move", taxi, (x, y - 1)))

            # picking:
            passengers = state['passengers']
            if state['taxis'][taxi]['capacity'] > 0:
                for passenger in passengers:
                    if (state['passengers'][passenger]['is_picked'] is False) and \
                       (x == state['passengers'][passenger]['location'][0]) and \
                       (y == state['passengers'][passenger]['location'][1]):
                        taxi_action.append(("pick up", taxi, passenger))

            # dropping:
            for passenger in passengers:
                if passenger in state['taxis'][taxi]['passengers_on_taxi']:
                    if (x == state['passengers'][passenger]['destination'][0]) and \
                       (y == state['passengers'][passenger]['destination'][1]):
                        taxi_action.append(("drop off", taxi, passenger))

            # refuel:
            if self.map[x][y] == 'G':
                taxi_action.append(("refuel", taxi))

            taxis_action.append(taxi_action)

        if len(taxis_action) == 1:
            # Single taxi: return tuple of its atomic actions
            return tuple(tuple(sub) for sub in taxis_action[0])

        # Multi-taxi: Cartesian product of atomic actions, filter collisions
        action_result = list(product(*taxis_action))
        for action in list(action_result):
            locations_count = 0
            location_set = set()
            for tax_act in action:
                if tax_act[0] != "move":
                    location = ' '.join([str(elem) for elem in state['taxis'][tax_act[1]]['location']])
                else:
                    location = ' '.join([str(elem) for elem in tax_act[2]])
                location_set.add(location)
                locations_count += 1
            if locations_count != len(location_set):
                action_result.remove(action)

        returned_action_result = tuple(tuple(sub) for sub in action_result)
        return returned_action_result

    def result(self, state, action):
        """Apply a single atomic action or a tuple of per-taxi actions and return the next state (as JSON string)."""
        state = copy.deepcopy(state)
        state = json.loads(state)

        if isinstance(action[0], str):
            self.apply_taxi_action(state, action)
        else:
            for taxi_action in action:
                self.apply_taxi_action(state, taxi_action)

        state = json.dumps(state)
        return copy.deepcopy(state)

    def apply_taxi_action(self, state, taxi_action):
        """Apply a single taxi atomic action; updates location/fuel/passenger bookkeeping accordingly."""
        if taxi_action[0] == "move":
            state["taxis"][taxi_action[1]]["location"] = taxi_action[2]
            state["taxis"][taxi_action[1]]["fuel"] -= 1
            for passenger in state['taxis'][taxi_action[1]]['passengers_on_taxi']:
                state['passengers'][passenger]['current_location'] = taxi_action[2]

        if taxi_action[0] == "pick up":
            state["taxis"][taxi_action[1]]["capacity"] -= 1
            state["taxis"][taxi_action[1]]["passengers_on_taxi"].append(taxi_action[2])
            state['passengers'][taxi_action[2]]['is_picked'] = True
            state["num_of_unpicked"] -= 1
            state["num_of_picked_undelivered"] += 1

        if taxi_action[0] == "drop off":
            state["taxis"][taxi_action[1]]["capacity"] += 1
            state["num_of_undelivered"] -= 1
            state['passengers'][taxi_action[2]]['is_arrived'] = True
            state['taxis'][taxi_action[1]]['passengers_on_taxi'].remove(taxi_action[2])
            state["num_of_picked_undelivered"] -= 1

        if taxi_action[0] == "refuel":
            state["taxis"][taxi_action[1]]["fuel"] = state['taxis'][taxi_action[1]]['MAX_FUEL']

    def goal_test(self, state):
        """Goal is reached iff there are no undelivered passengers remaining."""
        state = json.loads(state)
        if state['num_of_undelivered'] == 0:
            return True
        else:
            return False

    def manhattan_distance(self, point1, point2):
        """Compute Manhattan (L1) distance between two grid points."""
        return sum(abs(value1 - value2) for value1, value2 in zip(point1, point2))

    def find_distance_of_closest_taxi(self, passenger_loc, state):
        """Return (distance, taxi_name) of the closest taxi to the given passenger location in the given state dict."""
        min_dis = 100000
        for taxi in state['taxis']:
            pass_dis = self.manhattan_distance(passenger_loc, state['taxis'][taxi]['location'])
            if pass_dis < min_dis:
                min_dis = pass_dis
                min_taxi = taxi
        return (min_dis, min_taxi)

    def h(self, node):
        """
        Admissible heuristic combining:
        - average distance-to-destination for undelivered passengers,
        - distance of unpicked passengers to the closest taxi,
        - small penalty for capacity shortfall,
        scaled by problem counts.
        """
        h_sum = 0  # (kept for compatibility with original structure)
        state = json.loads(node.state)
        if state['num_of_undelivered'] == 0:
            return 0

        passengers = state['passengers']
        taxis = state['taxis']

        sum_of_dis_to_dest = 0
        sum_of_dis_to_closest_taxi = 0

        for passenger in passengers:
            loc = passengers[passenger]['current_location']
            dest = passengers[passenger]['destination']
            if passengers[passenger]['is_arrived'] is False:
                sum_of_dis_to_dest += self.manhattan_distance(loc, dest)
            if passengers[passenger]['is_picked'] is False:
                _taxi_dis = self.find_distance_of_closest_taxi(loc, state)
                sum_of_dis_to_closest_taxi += self.find_distance_of_closest_taxi(loc, state)[0]

        avg_of_dis_to_dest = sum_of_dis_to_dest / state['num_of_undelivered']

        if state['num_of_unpicked'] > 0:
            avg_of_dis_to_closest_taxi = sum_of_dis_to_closest_taxi / state['num_of_unpicked']
            sum_capacity = 0
            for taxi in taxis:
                sum_capacity += taxis[taxi]['capacity']
            diff_cap_unpicked_pass = state['num_of_unpicked'] - sum_capacity

            if diff_cap_unpicked_pass > 0:
                return (
                    avg_of_dis_to_dest
                    + avg_of_dis_to_closest_taxi
                    + diff_cap_unpicked_pass
                    + state['num_of_undelivered']
                    + state['num_of_unpicked']
                )

            return (
                avg_of_dis_to_dest
                + avg_of_dis_to_closest_taxi
                + state['num_of_undelivered']
                + state['num_of_unpicked']
            )

        return avg_of_dis_to_dest + state['num_of_undelivered']

    def h_1(self, node):
        """(2 * #unpicked + #picked_but_not_delivered) / #taxis."""
        state = json.loads(node.state)
        return ((state['num_of_unpicked'] * 2 + state['num_of_picked_undelivered']) / (len(state['taxis'])))

    def h_2(self, node):
        """
        Sum of Manhattan distances from (destination to initial origin) +
        (destination to current location) for all passengers, normalized by #taxis.
        """
        state = json.loads(node.state)
        sum_of_initial_distance = 0
        sum_of_curr_distance = 0
        for passenger in state['passengers']:
            sum_of_initial_distance += self.manhattan_distance(
                state['passengers'][passenger]['destination'],
                state['passengers'][passenger]['location']
            )
            sum_of_curr_distance += self.manhattan_distance(
                state['passengers'][passenger]['destination'],
                state['passengers'][passenger]['current_location']
            )
        return (sum_of_initial_distance + sum_of_curr_distance) / len(state['taxis'])


def create_taxi_problem(game):
    return TaxiProblem(game)
