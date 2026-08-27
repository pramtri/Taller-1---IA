from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    posicion, tieneKit, sistemasPendientes = state

    if not tieneKit:
        objetivo = problem.kitPosition
    elif sistemasPendientes:
        # Se toma el T pendiente mas cercano en distancia Manhattan
        objetivo = min(
            sistemasPendientes,
            key=lambda t: abs(posicion[0] - t[0]) + abs(posicion[1] - t[1]),
        )
    else:
        objetivo = problem.controlPosition

    return abs(posicion[0] - objetivo[0]) + abs(posicion[1] - objetivo[1])


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    # TODO: Add your code here
    utils.raiseNotDefined()


def systemRepairHeuristic(
    state: Tuple[Tuple, bool], problem: SystemRepairProblem
):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    #state = (position, hasKit, pendingSystems)
    position, hasKit, pendingSystems = state
    if hasKit:
        points = [position] + list(pendingSystems) + [problem.controlPosition]
        return MST(points)
    else:
        distancia_kit = distancia(position, problem.kitPosition)
        points = [problem.kitPosition] + list(pendingSystems) + [problem.controlPosition]
        return distancia_kit + MST(points)
        
            

def distancia(p1,p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

def MST(points):
    """
    implementado con el algoritmo de prim para encontrar el arbol de expansión mínima
    entre puntos de interés
    """

    #evitar puntos duplicados
    points = list(set(points)) 

    if len(points) <= 1:
        return 0

    visited = {points[0]}
    total_cost = 0

    while len(visited) < len(points):
        # Punto más cercano a los puntos visitados
        best = None
        min_distance = float('inf')
        for p1 in visited:
            for p2 in points:
                if p2 not in visited:
                    dist = distancia(p1, p2)
                    if dist < min_distance:
                        min_distance = dist
                        best = p2
        total_cost += min_distance
        visited.add(best)
    return total_cost