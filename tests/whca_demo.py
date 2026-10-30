"""
Demostracion de WHCA* fuera de ROS: escenarios chicos, animados

    python3 whca_demo.py

Se abre una ventana: a la izquierda el escenario animandose, a la
derecha que esta probando y por que. Con los botones se cambia de
escenario.

No necesita Gazebo ni colcon ni source. Toda la capa de planeacion solo
importa math, heapq e itertools, y lo unico con forma de ROS que toca es
un OccupancyGrid, del que lee seis atributos: por eso la clase Rejilla
de aqui abajo basta para correr el planificador completo.
"""

import os
import sys
import textwrap

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from matplotlib.widgets import Button

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'ros2_ws', 'src', 'navigation', 'swarm_mapf'))

from swarm_mapf.planners import whca          # noqa: E402

CELDA = 1.0          # un caracter del dibujo es una celda del enrejado
VENTANA = 64         # pasos de cooperacion; grande, para ver el algoritmo
COLORES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']

MURO = '#b8b6b0'
PISO = '#ffffff'
LINEA = '#e4e2dc'
FONDO = '#fcfcfb'
TINTA = '#0b0b0b'
TINTA_2 = '#52514e'


# ─── Los escenarios ──────────────────────────────────────────────────
#
#   #  muro          A  donde empieza el robot A
#   .  libre         a  a donde va el robot A
#
# Cuando la meta cae sobre el arranque de otro robot el dibujo no puede
# llevar los dos caracteres, y entonces la meta va en 'metas'.

ESCENARIOS = [
    {
        'titulo': 'Pasillo de una via',
        'prueba': 'WHCA* es incompleto: aqui tiene que fallar',
        'porque':
            'Dos robots quieren intercambiar sus lugares en un pasillo '
            'donde no cabe mas que uno, y no hay ningun rincon donde '
            'hacerse a un lado.\n\n'
            'No existe solucion, asi que lo correcto es que el '
            'planificador conteste que no hay ruta. Fallar aqui no es un '
            'error: es la propiedad que CBS deberia mejorar.',
        'mapa': """
#########
#A...B.a#
#########
""",
        'metas': {'B': (1, 1)},
    },
    {
        'titulo': 'El rincon, lejos',
        'prueba': 'hay solucion y WHCA* no la encuentra',
        'porque':
            'El mismo pasillo, ahora con un rincon donde caber. Pero esta '
            'del lado de A, y B no alcanza a llegar antes de que A le '
            'cierre el paso.\n\n'
            'La solucion existe: si A se quedara quieto unos pasos, B '
            'llegaria al rincon, A pasaria y B saldria. WHCA* no la '
            'encuentra porque planifica por prioridad, y el primero '
            'planifica como si estuviera solo: nunca espera por nadie.\n\n'
            'Este es el limite del metodo, no una falla. Es justo donde '
            'CBS gana, porque CBS si puede obligar a esperar al de '
            'arriba.',
        'mapa': """
#########
#A...B.a#
##.######
#########
""",
        'metas': {'B': (1, 2)},
    },
    {
        'titulo': 'El rincon, cerca',
        'prueba': 'una sola celda decide si resuelve o se rinde',
        'porque':
            'El mismo pasillo, con el rincon justo al lado de B.\n\n'
            'Ahora B si alcanza a meterse, se queda ahi mientras A pasa '
            'de largo, y sale detras de el. Lo unico que cambio respecto '
            'al escenario anterior es donde esta el rincon.\n\n'
            'Fijate en el momento en que B se detiene: eso es ceder el '
            'paso, y es posible porque esperar es un movimiento mas de la '
            'busqueda, igual que avanzar.\n\n'
            'B da un paso de mas antes de meterse al rincon. No es un '
            'error: su llegada depende de cuando A libere el pasillo, '
            'asi que varias rutas cuestan exactamente lo mismo y la '
            'busqueda toma una cualquiera de ellas.',
        'mapa': """
#########
#A...B.a#
#####.###
#########
""",
        'metas': {'B': (1, 2)},
    },
    {
        'titulo': 'Cuello de botella',
        'prueba': 'la espera: tres robots y un solo paso',
        'porque':
            'Dos salas unidas por un hueco de una sola celda. Los tres '
            'robots tienen que cruzar por ahi.\n\n'
            'A y C estan a la misma distancia del hueco, asi que lo '
            'necesitan en el mismo paso y no hay desvio posible: uno de '
            'los dos tiene que esperar su turno. Se forma una fila.\n\n'
            'Este es el caso que los escenarios reales nunca producen, '
            'porque alla la geometria escalona a los robots sola.',
        'mapa': """
#############
#A....#....a#
#B.........b#
#C....#....c#
#############
""",
    },
    {
        'titulo': 'Cuello de botella, prioridad al reves',
        'prueba': 'el orden no es cosmetico: cambia la solucion',
        'porque':
            'El mismo cuello de botella, planificando en orden inverso.\n\n'
            'Ahora espera otro robot y las rutas no son las mismas. La '
            'prioridad es la unica perilla que tiene WHCA*, y aqui se ve '
            'que decide quien gana y cuanto le cuesta a cada quien.',
        'mapa': """
#############
#A....#....a#
#B.........b#
#C....#....c#
#############
""",
        'prioridad': ['C', 'B', 'A'],
    },
    {
        'titulo': 'Cruce',
        'prueba': 'dos trayectorias perpendiculares que coinciden',
        'porque':
            'A cruza de izquierda a derecha y B de arriba abajo. Los dos '
            'pasan por el centro, y estan a la misma distancia de el.\n\n'
            'No es un pasillo: hay espacio alrededor. Aun asi, la celda '
            'del centro solo admite a uno a la vez, y el de menor '
            'prioridad tiene que dejarla pasar antes de entrar.',
        'mapa': """
#########
####B####
####.####
##A....a#
####.####
####b####
#########
""",
    },
    {
        'titulo': 'Una meta que tapa el paso',
        'prueba': 'un robot estacionado sigue ocupando su celda',
        'porque':
            'La meta de A es el hueco mismo. A llega, se queda ahi para '
            'siempre, y B se queda sin ruta porque no hay otro paso.\n\n'
            'Que falle es la prueba de que la tabla sostiene la celda de '
            'un robot que ya llego. Si la soltara, B planificaria encima '
            'de un robot estacionado y el choque apareceria hasta '
            'Gazebo.',
        'mapa': """
#########
#A..#...#
#.......#
#B..#..b#
#########
""",
        'metas': {'A': (4, 2)},
    },
]


# ─── Lo minimo que el planificador necesita de un mapa ───────────────

class Rejilla:
    """Lo que el planificador ve de un nav_msgs/OccupancyGrid."""

    class _Info:
        class _Origen:
            class _Punto:
                x = 0.0
                y = 0.0
            position = _Punto()
        resolution = CELDA

    def __init__(self, ancho, alto, datos):
        self.info = Rejilla._Info()
        self.info.width = ancho
        self.info.height = alto
        self.info.origin = Rejilla._Info._Origen()
        self.data = datos


def leer(mapa):
    """Convierte el dibujo en rejilla, arranques y metas."""
    filas = [f for f in mapa.strip('\n').split('\n') if f.strip()]
    alto, ancho = len(filas), max(len(f) for f in filas)
    datos = [0] * (ancho * alto)
    arranques, metas = {}, {}

    for numero, fila in enumerate(filas):
        j = alto - 1 - numero          # el dibujo baja, la rejilla sube
        for i, c in enumerate(fila):
            if c == '#':
                datos[j * ancho + i] = 100
            elif c.isupper():
                arranques[c] = (i, j)
            elif c.islower():
                metas[c.upper()] = (i, j)

    return Rejilla(ancho, alto, datos), arranques, metas


def planificar(escenario):
    """Arma el escenario y lo resuelve. Devuelve None si no hay ruta."""
    rejilla, arranques, metas = leer(escenario['mapa'])
    metas.update(escenario.get('metas', {}))

    # El orden de este diccionario es la prioridad: WHCA* la lee de ahi
    orden = escenario.get('prioridad') or sorted(arranques)
    arranques = {nombre: arranques[nombre] for nombre in orden}

    rutas = whca.plan(arranques, metas, rejilla, CELDA, window=VENTANA)
    return rejilla, arranques, metas, rutas


# ─── La ventana ──────────────────────────────────────────────────────

def _envolver(texto, ancho=62):
    """Corta el texto a un ancho fijo, respetando los parrafos."""
    return '\n\n'.join(textwrap.fill(parrafo, ancho)
                       for parrafo in texto.split('\n\n'))


class Demo:
    """Dibuja un escenario a la vez y lo anima en bucle."""

    PAUSA = 8          # cuadros quieto al final, antes de repetir

    def __init__(self):
        self.fig = plt.figure(figsize=(15, 7), facecolor=FONDO)
        self.fig.canvas.manager.set_window_title('WHCA* paso a paso')

        self.mapa_ax = self.fig.add_axes([0.03, 0.12, 0.56, 0.80])
        self.texto_ax = self.fig.add_axes([0.62, 0.12, 0.36, 0.80])
        self.texto_ax.axis('off')

        anterior = self.fig.add_axes([0.03, 0.03, 0.10, 0.055])
        siguiente = self.fig.add_axes([0.14, 0.03, 0.10, 0.055])
        self.b_anterior = Button(anterior, 'anterior', color=FONDO)
        self.b_siguiente = Button(siguiente, 'siguiente', color=FONDO)
        self.b_anterior.on_clicked(lambda _: self.mover(-1))
        self.b_siguiente.on_clicked(lambda _: self.mover(+1))

        self.indice = 0
        self.cargar()
        self.animacion = FuncAnimation(
            self.fig, self.cuadro, interval=700,
            cache_frame_data=False, blit=False)

    # ── escenario actual ──

    def mover(self, paso):
        self.indice = (self.indice + paso) % len(ESCENARIOS)
        self.cargar()

    def cargar(self):
        self.escenario = ESCENARIOS[self.indice]
        (self.rejilla, self.arranques,
         self.metas, self.rutas) = planificar(self.escenario)

        self.color = {n: COLORES[i % len(COLORES)]
                      for i, n in enumerate(sorted(self.arranques))}
        self.largo = (0 if self.rutas is None
                      else max(len(r) for r in self.rutas.values()) - 1)
        self.paso = 0
        self.explicar()

    def donde(self, nombre, paso):
        """Celda que ocupa un robot en un paso, ya estacionado o no."""
        ruta = (self.rutas or {}).get(nombre)
        if not ruta:
            return self.arranques[nombre]
        return ruta[min(paso, len(ruta) - 1)]

    def espera(self, nombre, paso):
        ruta = (self.rutas or {}).get(nombre)
        return bool(ruta) and 0 < paso < len(ruta) and ruta[paso] == ruta[paso - 1]

    # ── dibujo ──

    def cuadro(self, _):
        self.dibujar()
        self.paso += 1
        if self.paso > self.largo + self.PAUSA:
            self.paso = 0

    def dibujar(self):
        paso = min(self.paso, self.largo)
        ax = self.mapa_ax
        ax.clear()
        ax.set_facecolor(FONDO)

        info = self.rejilla.info
        for i in range(info.width):
            for j in range(info.height):
                muro = self.rejilla.data[j * info.width + i] > 50
                ax.add_patch(Rectangle((i, j), 1, 1,
                                       facecolor=MURO if muro else PISO,
                                       edgecolor=LINEA, linewidth=0.8))

        for nombre in sorted(self.arranques):
            color = self.color[nombre]

            mi, mj = self.metas[nombre]
            ax.add_patch(Circle((mi + 0.5, mj + 0.5), 0.40, fill=False,
                                edgecolor=color, linewidth=2.5,
                                linestyle=(0, (3, 2)), zorder=3))

            ruta = (self.rutas or {}).get(nombre)
            if ruta:
                hasta = ruta[:paso + 1]
                ax.plot([c[0] + 0.5 for c in hasta],
                        [c[1] + 0.5 for c in hasta],
                        color=color, linewidth=2.0, alpha=0.35, zorder=4)

            i, j = self.donde(nombre, paso)
            ax.add_patch(Circle((i + 0.5, j + 0.5), 0.34, facecolor=color,
                                edgecolor=FONDO, linewidth=2.0, zorder=5))
            ax.text(i + 0.5, j + 0.5, nombre, ha='center', va='center',
                    color='white', fontsize=12, fontweight='bold', zorder=6)
            if self.espera(nombre, paso):
                ax.text(i + 0.5, j + 1.02, 'espera', ha='center', va='bottom',
                        color=color, fontsize=9, fontweight='bold', zorder=6)

        if self.rutas is None:
            titulo = '%s  —  sin solucion' % self.escenario['titulo']
        else:
            titulo = '%s  —  paso %d de %d' % (self.escenario['titulo'],
                                               paso, self.largo)
        ax.set_title(titulo, color=TINTA, fontsize=13, loc='left', pad=12)

        ax.set_xlim(0, info.width)
        ax.set_ylim(0, info.height)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        for lado in ax.spines.values():
            lado.set_visible(False)

    def explicar(self):
        ax = self.texto_ax
        ax.clear()
        ax.axis('off')

        ax.text(0, 1.0, 'Escenario %d de %d' % (self.indice + 1,
                                                len(ESCENARIOS)),
                va='top', color=TINTA_2, fontsize=9)
        ax.text(0, 0.95, self.escenario['titulo'], va='top', color=TINTA,
                fontsize=15, fontweight='bold')
        ax.text(0, 0.87, _envolver('Que prueba:  ' + self.escenario['prueba']),
                va='top', color=TINTA, fontsize=10)
        ax.text(0, 0.78, _envolver(self.escenario['porque']), va='top',
                color=TINTA_2, fontsize=10, linespacing=1.6)

        if self.rutas is None:
            resultado = 'Resultado:  sin solucion, como debe ser'
        else:
            lineas = []
            for nombre, ruta in self.rutas.items():
                esperas = sum(1 for a, b in zip(ruta, ruta[1:]) if a == b)
                lineas.append('%s:  %d pasos, %d esperas'
                              % (nombre, len(ruta) - 1, esperas))
            resultado = 'Resultado\n' + '\n'.join(lineas)
        ax.text(0, 0.26, resultado, va='top', color=TINTA, fontsize=10,
                linespacing=1.8, fontweight='bold')

        orden = ', '.join(self.arranques)
        ax.text(0, 0.03, 'Prioridad: %s  (el primero nunca cede)' % orden,
                va='top', color=TINTA_2, fontsize=9)


if __name__ == '__main__':
    Demo()
    plt.show()
