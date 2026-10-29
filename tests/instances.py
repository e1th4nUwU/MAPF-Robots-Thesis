"""
Instances drawn by hand, each one testing a single thing

They are small on purpose: the right answer is known by looking at the
drawing, before running anything, so a wrong answer is visible instead
of plausible. That is what the real scenarios could not give, because
there cooperation costs +0 steps and nobody ever waits

Format, readable on paper and pasted straight into the thesis:

    #  obstacle          A  where robot A starts
    .  free              a  where robot A is going

When a goal lands on another robot's start the drawing cannot hold both
characters, so those instances give the goal in `goals` instead

`priority` is the order the robots are planned in, highest first. It is
the only knob WHCA* has: the first robot plans as if it were alone and
every later one plans around what the earlier ones wrote
"""

# Steps of cooperation. Large on purpose: WHCA* only reserves `window`
# steps ahead, and these instances are meant to show what the algorithm
# does when it is actually cooperating, not what the windowing
# approximation costs. The window is studied separately, in the maps
WINDOW = 32

INSTANCES = [
    {
        'name': 'corredor-sin-salida',
        'asks': 'WHCA* es incompleto: aqui debe fallar',
        'why': (
            'Dos robots que se quieren intercambiar en un pasillo de un '
            'carril, sin ningun lugar donde hacerse a un lado. No hay '
            'solucion que exista, asi que el planificador tiene que '
            'devolver None. Fallar aqui es el comportamiento correcto y '
            'es la propiedad que CBS deberia mejorar'),
        'drawing': """
#######
#A...B#
#######
""",
        'goals': {'A': (5, 1), 'B': (1, 1)},
        'expect': {'solved': False},
    },
    {
        'name': 'nicho-lejos',
        'asks': 'hay solucion y WHCA* no la encuentra',
        'why': (
            'El mismo pasillo con un nicho, pero puesto lejos de B. La '
            'solucion existe: A se queda quieto cuatro pasos, B alcanza '
            'el nicho, A pasa, B sale. WHCA* no la halla porque planifica '
            'por prioridad y el primero planifica como si estuviera solo, '
            'asi que nunca espera por nadie.\n\n'
            'Esta instancia es la mas valiosa del banco: no muestra un '
            'error, muestra el limite del metodo. Es incompletitud con '
            'solucion disponible, y es exactamente donde CBS deberia '
            'ganar, porque CBS si puede obligar a esperar al de arriba'),
        'drawing': """
#######
#A...B#
###.###
#######
""",
        'goals': {'A': (5, 2), 'B': (1, 2)},
        'expect': {'solved': False},
    },
    {
        'name': 'nicho-cerca',
        'asks': 'con el nicho al alcance, el de abajo cede y si resuelve',
        'why': (
            'El mismo pasillo y el mismo nicho, una celda mas cerca de B. '
            'Ahora B alcanza a meterse antes de que A llegue, lo deja '
            'pasar y sale. Lo unico que cambia entre esta y la anterior '
            'es la posicion del nicho: una celda decide si el metodo '
            'resuelve o se rinde'),
        'drawing': """
#######
#A...B#
####.##
#######
""",
        'goals': {'A': (5, 2), 'B': (1, 2)},
        'expect': {'solved': True, 'uses_cell': (4, 1)},
    },
    {
        'name': 'cuello-simultaneo',
        'asks': 'la espera: dos robots equidistantes de un hueco de una celda',
        'why': (
            'Dos corredores separados por un muro con un solo hueco. Los '
            'arranques estan a la misma distancia del hueco, asi que los '
            'dos lo necesitan en el mismo paso y no hay desvio posible: '
            'el de menor prioridad tiene que esperar. Es el caso que los '
            'escenarios reales nunca producen, porque ahi la geometria '
            'escalona a los robots sola'),
        'drawing': """
#########
#A..#..a#
#.......#
#B..#..b#
#########
""",
        'expect': {'solved': True, 'waits_at_least': 1},
    },
    {
        'name': 'prioridad-invertida',
        'asks': 'el orden de prioridad cambia la solucion, no solo quien gana',
        'why': (
            'La misma instancia con el orden al reves. El que espera es '
            'el otro, y las rutas no son las mismas: la prioridad es el '
            'unico parametro de WHCA* y aqui se ve que no es cosmetico'),
        'drawing': """
#########
#A..#..a#
#.......#
#B..#..b#
#########
""",
        'priority': ['B', 'A'],
        'expect': {'solved': True, 'waits_at_least': 1},
    },
    {
        'name': 'meta-en-el-cuello',
        'asks': 'hold_until: un robot estacionado sigue ocupando su celda',
        'why': (
            'La meta de A es el hueco mismo. A llega, se queda ahi, y B '
            'se queda sin ruta: no hay otro paso. Si la tabla soltara la '
            'celda al terminar la ruta de A, B planificaria encima de un '
            'robot estacionado y el choque apareceria en Gazebo y no '
            'aqui. Que falle es la prueba de que hold_until se respeta'),
        'drawing': """
#########
#A..#...#
#.......#
#B..#..b#
#########
""",
        'goals': {'A': (4, 2)},
        'expect': {'solved': False},
        'known_issue': (
            'Hoy no falla: B espera window - 3 pasos y luego cruza por '
            'encima de A. La causa es que whca.plan reserva la meta con '
            'hold_until=window, asi que un robot estacionado se evapora '
            'pasado el horizonte. Operando no se nota, porque cada ronda '
            'reconstruye la ventana; pero el plan que devuelve es, '
            'estrictamente, invalido. hold_until deberia atarse al limite '
            'de la busqueda y no a la ventana'),
    },
    {
        'name': 'cruce-en-cuarto-abierto',
        'asks': 'con espacio, cooperar es desviarse y no esperar',
        'why': (
            'El mismo intercambio de frente, pero en un cuarto de dos '
            'filas. Hay solucion sin que nadie espere: el de menor '
            'prioridad rodea. Explica por que los escenarios reales dan '
            'cero esperas sin que eso signifique que no haya cooperacion'),
        'drawing': """
########
#A....B#
#......#
########
""",
        'goals': {'A': (6, 2), 'B': (1, 2)},
        'expect': {'solved': True, 'waits_at_most': 0},
    },
]


def by_name(name):
    """
    Look up one instance

    Args:
        name: the instance name

    Returns:
        The instance, or None when there is no such name
    """
    for instance in INSTANCES:
        if instance['name'] == name:
            return instance
    return None
