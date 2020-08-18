"""Functions to generate various graph Laplacians.

Written by: The Robotarium Team
Modified by: Zahi Kakish (zmk5)

"""
import numpy as np


def cycle_GL(N: int) -> np.ndarray:
    """Generate a graph Laplacian for a cycle graph.

    Parameters
    ----------
    N : int
        The number of agents.

    Returns
    -------
    An `NxN` numpy array (representing the graph Laplacian).
    """
    # Check user input types
    assert isinstance(N, int), f'In the cycle_GL function, the number of nodes (N) must be an integer. Recieved type {type(N).__name__}.'
    # Check user input ranges/sizes
    assert N > 0, f'In the cycle_GL function, number of nodes (N) must be positive. Recieved {N}.'

    ones = np.ones(N - 1)
    L = 2 * np.identity(N) - np.diag(ones, 1) - np.diag(ones, -1)
    L[N-1, 0] = -1
    L[0, N-1] = -1

    return L


def lineGL(N: int) -> np.ndarray:
    """Generate a graph Laplacian for a line graph.

    Parameters
    ----------
    N : int
        The number of agents.

    Returns
    -------
    An `NxN` numpy array (representing the graph Laplacian).
    """
    # Check user input types
    assert isinstance(N, int), f'In the lineGL function, the number of nodes (N) must be an integer. Recieved type {type(N).__name__}.'
    # Check user input ranges/sizes
    assert N > 0, f'In the lineGL function, number of nodes (N) must be positive. Recieved {N}.'

    ones = np.ones(N - 1)
    L = 2 * np.identity(N) - np.diag(ones, 1) - np.diag(ones, -1)
    L[0, 0] = 1
    L[N-1, N-1] = 1

    return L


def completeGL(N: int) -> np.ndarray:
    """Generate a graph Laplacian for a complete graph.

    Parameters
    ----------
    N : int
        The number of agents.

    Returns
    -------
    An `NxN` numpy array (representing the graph Laplacian).
    """
    # Check user input types
    assert isinstance(N, int), f'In the completeGL function, the number of nodes (N) must be an integer. Recieved type {type(N).__name__}.'
    # Check user input ranges/sizes
    assert N > 0, f'In the completeGL function, number of nodes (N) must be positive. Recieved {N}.'

    return N * np.identity(N) - np.ones((N, N))


def random_connectedGL(v: int, e: int) -> np.ndarray:
    """Generate a Laplacian for a random, connected graph with v verticies
    and (v-1) + e edges.

    Parameters
    ----------
    v : int
        The number of nodes.
    e : int
        Number of additional edges

    Returns
    -------
    A `vxv` numpy array (representing the graph Laplacian).
    """
    # Check user input types
    assert isinstance(v, int), f'In the random_connectedGL function, the number of verticies (v) must be an integer. Recieved type {type(v).__name__}.'
    assert isinstance(e, int), f'In the random_connectedGL function, the number of additional edges (e) must be an integer. Recieved type {type(e).__name__}.'
    # Check user input ranges/sizes
    assert v > 0, f'In the random_connectedGL function, number of verticies (v) must be positive. Recieved {v}.'
    assert e >= 0, f'In the random_connectedGL function, number of additional edges (e) must be greater than or equal to zero. Recieved {e}.'


    L = np.zeros((v, v))

    for i in range(1, v):
        edge = np.random.randint(i,size=(1,1))
        # Update adjancency relations
        L[i, edge] = -1
        L[edge, i] = -1

        # Update node degrees
        L[i, i] += 1
        L[edge, edge] = L[edge, edge]+1

    # Because all nodes have at least 1 degree, chosse from only upper diagonal portion
    iut = np.triu_indices(v)
    iul = np.tril_indices(v)
    Ltemp = np.copy(L)
    Ltemp[iul] = 1
    potEdges = np.where(np.logical_xor(Ltemp, 1) is True)
    numEdges = min(e, len(potEdges[0]))

    if numEdges <= 0:
        return L

    # Indicies of randomly chosen extra edges
    edgeIndicies = np.random.permutation(len(potEdges[0]))[:numEdges]
    sz = L.shape

    for index in edgeIndicies:
        # Update adjacency relation
        L[potEdges[0][index], potEdges[1][index]] = -1
        L[potEdges[1][index], potEdges[0][index]] = -1

        # Update Degree Relation
        L[potEdges[0][index], potEdges[0][index]] += 1
        L[potEdges[1][index], potEdges[1][index]] += 1

    return L


def randomGL(v: int, e: int) -> np.ndarray:
    """Generates a Laplacian for a random graph with v verticies
    and e edges.

    Parameters
    ----------
    v : int
        The number of nodes.
    e : int
        Number of additional edges.

    Returns
    -------
    A `vxv` numpy array (representing the graph Laplacian)
    """
    L = np.tril(np.ones((v, v)))

    # This works because you can't select diagonals
    potEdges = np.where(L == 0)

    L = L-L

    numEdges = min(e, len(potEdges[0]))
    # Indicies of randomly chosen extra edges
    edgeIndicies = np.random.permutation(len(potEdges[0]))[:numEdges]

    for index in edgeIndicies:
        # Update adjacency relation
        L[potEdges[0][index], potEdges[1][index]] = -1
        L[potEdges[1][index], potEdges[0][index]] = -1

        # Update Degree Relation
        L[potEdges[0][index], potEdges[0][index]] += 1
        L[potEdges[1][index], potEdges[1][index]] += 1

    return L


def topological_neighbors(L: np.ndarray, agent: int) -> np.ndarray:
    """Return the neighbors of a particular agent using the graph Laplacian.

    Parameters
    ----------
    L : np.ndarray
        An NxN numpy array (representing the graph Laplacian).
    agent : int
        The agent ID (0 - N-1).

    Returns
    -------
    A `1xM` numpy array (with M neighbors).
    """
    # Check user input types
    assert isinstance(L, np.ndarray), f'In the topological_neighbors function, the graph Laplacian (L) must be a numpy ndarray. Recieved type {type(L).__name__}.'
    assert isinstance(agent, int), f'In the topological_neighbors function, the agent number (agent) must be an integer. Recieved type { type(agent).__name__}.'

    # Check user input ranges/sizes
    assert agent >= 0, f'In the topological_neighbors function, the agent number (agent) must be greater than or equal to zero. Recieved {agent}.'
    assert agent <= L.shape[0], f'In the topological_neighbors function, the agent number (agent) must be within the dimension of the provided Laplacian (L). Recieved agent number {agent} and Laplactian size {L.shape[0]} by {L.shape[1]}.'

    row = L[agent, :]
    row[agent] = 0
    # Since L = D - A
    return np.where(row != 0)[0]


def delta_disk_neighbors(
        poses: np.ndarray,
        agent: int,
        delta: float) -> np.ndarray:
    """Return the agents within the 2-norm of the supplied agent.

    NOTE: The does not including the agent itself!

    Parameters
    ----------
    poses : np.ndarray
        A `3xN` numpy array representing the unicycle statese of the robots.
    agent : int
        The agent whose neighbors within a radius will be returned.
    delta : float
        The radius of delta disk considered.

    Returns
    -------
    A `1xM` numpy array (with M neighbors).
    """
    # Check user input types
    assert isinstance(poses, np.ndarray), f'In the delta_disk_neighbors function, the robot poses (poses) must be a numpy ndarray. Recieved type {type(poses).__name__}.'
    assert isinstance(agent, int), f'In the delta_disk_neighbors function, the agent number (agent) must be an integer. Recieved type {type(agent).__name__}.'
    assert isinstance(delta, (int, float)), f'In the delta_disk_neighbors function, the agent number (agent) must be an integer. Recieved type  {type(agent).__name__}.'

    # Check user input ranges/sizes
    assert agent >= 0, f'In the delta_disk_neighbors function, the agent number (agent) must be greater than or equal to zero. Recieved {agent}.'
    assert delta >= 0, f'In the delta_disk_neighbors function, the sensing/communication radius (delta) must be greater than or equal to zero. Recieved {delta}.'
    assert agent <= poses.shape[1], f'In the delta_disk_neighbors function, the agent number (agent) must be within the dimension of the provided poses. Recieved agent number {agent} and poses for {poses.shape[1]} agents.'

    N = poses.shape[1]
    agents = np.arange(N)

    within_distance = [
        np.linalg.norm(poses[:2, x] - poses[:2, agent]) <= delta for x in agents
    ]

    within_distance[agent] = False
    return agents[within_distance]
