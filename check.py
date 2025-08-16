import multi_taxi_planning
import search
import time
from dataclasses import dataclass, field


def timeout_exec(func, args=(), kwargs={}, timeout_duration=10, default=None):
    """This function will spawn a thread and run the given function
    using the args, kwargs and return the given default value if the
    timeout_duration is exceeded.
    """
    import threading
    class InterruptableThread(threading.Thread):
        def __init__(self):
            threading.Thread.__init__(self)
            self.result = default

        def run(self):
            # try:
            self.result = func(*args, **kwargs)
            # except Exception as e:
            #    self.result = (-3, -3, e)

    it = InterruptableThread()
    it.start()
    it.join(timeout_duration)
    if it.is_alive():
        return default
    else:
        return it.result


def check_problem(p, search_method, timeout):

    """ (-2, -2, None) means there was a timeout
    (-3, -3, ERR) means there was some error ERR during search """

    t1 = time.time()
    s = timeout_exec(search_method, args=[p], timeout_duration=timeout)
    t2 = time.time()

    if isinstance(s, search.Node):
        solve = s
        solution = list(map(lambda n: n.action, solve.path()))[1:]
        return (len(solution), t2 - t1, solution)
    elif s is None:
        return (-2, -2, None)
    else:
        return s


def solve_problems(problems):
    solved = 0
    for problem in problems:
        try:
            p = multi_taxi_planning.create_taxi_problem(problem)
        except Exception as e:
            print("Error creating problem: ", e)
            return None
        timeout = 60
        result = check_problem(p, (lambda p: search.astar_search(p, p.h)), timeout)
        print("A*: ", result)
        if result[2] != None:
            if result[0] != -3:
                solved = solved + 1


def main():
    """Here goes the input you want to check"""
    problems = [
    {
        'map': [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'G', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P']],
        'taxis': {'taxi 1': {'location': (1, 3), 'fuel': 10, 'capacity': 3}},
        'passengers': {'Michael': {'location': (3, 4), 'destination': (2, 1)},
                       'Freyja': {'location': (0, 0), 'destination': (2, 1)}},
    },


    {
        'map': [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'G', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P']],
        'taxis': {'taxi 1': {'location': (0, 2), 'fuel': 8, 'capacity': 2}},
        'passengers': {'Eitan': {'location': (1, 0), 'destination': (0, 2)},
                       'Omer': {'location': (3, 4), 'destination': (0, 2)}},
    },

    {
        'map': [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'],
                ['P', 'P', 'P', 'G', 'P']],
        'taxis': {'taxi 1': {'location': (1, 2), 'fuel': 18, 'capacity': 1}},
        'passengers': {'Freyja': {'location': (2, 0), 'destination': (4, 2)},
                       'Wolfgang': {'location': (2, 1), 'destination': (1, 4)},
                       'Jacob': {'location': (3, 4), 'destination': (3, 2)}},
    },

    {
        'map': [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'],
                ['P', 'P', 'P', 'G', 'P']],
        'taxis': {'taxi 1': {'location': (0, 2), 'fuel': 15, 'capacity': 2}},
        'passengers': {'Omar': {'location': (0, 3), 'destination': (3, 2)},
                       'John': {'location': (1, 0), 'destination': (4, 0)}},
    },

    {
        'map': [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'],
                ['P', 'P', 'P', 'G', 'P']],
        'taxis': {'taxi 1': {'location': (1, 0), 'fuel': 15, 'capacity': 3}},
        'passengers': {'Moshe': {'location': (4, 2), 'destination': (0, 1)},
                       'Freyja': {'location': (2, 3), 'destination': (0, 3)}},
    },



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


]
    solve_problems(problems)


if __name__ == '__main__':
    main()
