from swarm_mapf.allocators import nearest

ALLOCATORS = {
    'nearest': nearest.allocate,
}


def get(name):
    """
    Search an allocator by name

    Args:
        name: value of the 'allocator' parameter of the node

    Returns:
        The allocate() function, or None if the name is unknown
    """
    return ALLOCATORS.get(name)


def names():
    """Return the names of the available allocators, for error messages"""
    return sorted(ALLOCATORS)
