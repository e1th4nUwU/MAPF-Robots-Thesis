"""
Demostracion de WHCA* fuera de ROS sobre instancias pequenas.

    python3 whca_demo.py

Abre una ventana con el escenario animado a la izquierda y su
descripcion, parametros y resultado a la derecha. Los botones cambian de
escenario.

No requiere Gazebo, colcon ni source. La capa de planeacion solo importa
math, heapq e itertools, y del OccupancyGrid lee seis atributos, de modo
que la clase Rejilla de este archivo basta para ejecutar el planificador
completo.
"""

import filecmp
import glob
import os
import sys
import textwrap

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from matplotlib.widgets import Button

PAQUETE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'ros2_ws', 'src', 'navigation', 'swarm_mapf')
sys.path.insert(0, PAQUETE)

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


def revisar_instalacion():
    """
    Avisa si la copia que ejecuta ROS difiere de la fuente que usa esto.

    colcon build copia los archivos a install/, asi que editar la fuente
    sin reconstruir deja dos versiones distintas: este demo prueba una y
    el robot ejecuta la otra.
    """
    destino = glob.glob(os.path.join(
        PAQUETE, '..', '..', '..', 'install', 'swarm_mapf', 'lib',
        'python3*', 'site-packages', 'swarm_mapf'))
    if not destino:
        return

    fuente = os.path.join(PAQUETE, 'swarm_mapf')
    distintos = []
    for carpeta, _, archivos in os.walk(fuente):
        for archivo in archivos:
            if not archivo.endswith('.py'):
                continue
            ruta = os.path.join(carpeta, archivo)
            relativa = os.path.relpath(ruta, fuente)
            copia = os.path.join(destino[0], relativa)
            if os.path.exists(copia) and not filecmp.cmp(ruta, copia,
                                                         shallow=False):
                distintos.append(relativa)

    if distintos:
        print('Aviso: la copia instalada no coincide con la fuente.')
        print('Este demo usa la fuente; el robot usaria la copia.')
        for relativa in sorted(distintos):
            print('   ', relativa)
        print('Para igualarlas: colcon build --packages-select swarm_mapf')


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
        'prueba': 'instancia sin solucion',
        'porque':
            'Dos robots intercambian posiciones en un pasillo de una '
            'celda de ancho. No hay celdas laterales.\n\n'
            'Ninguna secuencia de movimientos resuelve la instancia, por '
            'lo que el planificador debe reportar que no existe ruta. El '
            'resultado esperado es que falle.',
        'mapa': """
#########
#A...B.a#
#########
""",
        'metas': {'B': (1, 1)},
    },
    {
        'titulo': 'El rincon, lejos',
        'prueba': 'incompletitud con solucion existente',
        'porque':
            'El pasillo tiene una celda lateral, ubicada del lado de A. B '
            'necesita cuatro pasos para alcanzarla y A la rebasa en el '
            'primero.\n\n'
            'Existe una solucion: si A permaneciera detenido mientras B '
            'entra a la celda lateral, A podria recorrer el pasillo y B '
            'salir despues. WHCA* no la encuentra porque planifica en '
            'orden de prioridad y el primer robot planifica sin '
            'restricciones, de modo que nunca espera.\n\n'
            'La limitacion es propia de la planificacion por prioridades. '
            'CBS no la tiene, porque puede imponer restricciones tambien '
            'al robot de mayor prioridad.',
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
        'prueba': 'efecto de la posicion de la celda lateral',
        'porque':
            'La misma instancia con la celda lateral junto a B.\n\n'
            'B entra en ella, permanece mientras A recorre el pasillo y '
            'sale cuando A libero su posicion. La unica diferencia '
            'respecto a la instancia anterior es la ubicacion de esa '
            'celda.\n\n'
            'La espera es una accion de la busqueda igual que avanzar: el '
            'estado incluye el paso de tiempo, por lo que permanecer en '
            'una celda es una transicion valida.\n\n'
            'B realiza un movimiento adicional antes de entrar. Su tiempo '
            'de llegada depende de cuando A libere el pasillo, por lo que '
            'varias rutas tienen el mismo costo y la busqueda devuelve '
            'una cualquiera de ellas.',
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
        'prueba': 'esperas por contencion en un paso unico',
        'porque':
            'Dos salas comunicadas por una sola celda. Los tres robots '
            'deben cruzarla.\n\n'
            'A y C estan a la misma distancia de esa celda y la requieren '
            'en el mismo paso. No hay ruta alternativa, de modo que el de '
            'menor prioridad espera su turno.\n\n'
            'En los escenarios de simulacion esta situacion no se '
            'presenta: las distancias a los cuellos difieren y los robots '
            'quedan escalonados sin necesidad de esperar.',
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
        'prueba': 'efecto del orden de prioridad',
        'porque':
            'La misma instancia planificada en orden inverso.\n\n'
            'El robot que espera es otro y las rutas cambian. El orden de '
            'prioridad es el unico parametro de WHCA* y determina como se '
            'reparte el costo entre los robots.',
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
        'prueba': 'contencion fuera de un pasillo',
        'porque':
            'A se desplaza en horizontal y B en vertical. Ambos pasan por '
            'la celda central y estan a la misma distancia de ella.\n\n'
            'No se trata de un pasillo: hay espacio libre alrededor. Aun '
            'asi la celda central admite un robot por paso, por lo que el '
            'de menor prioridad espera a que se libere.',
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
        'prueba': 'reserva de la celda meta',
        'porque':
            'La meta de A es la celda del paso unico. A llega y permanece '
            'ahi.\n\n'
            'B no tiene otra ruta, de modo que la instancia no tiene '
            'solucion. El resultado verifica que la tabla sostiene la '
            'celda de un robot que ya llego hasta el limite de la '
            'busqueda. Si la liberara, B planificaria sobre un robot '
            'detenido.',
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

        info = self.rejilla.info
        ax.text(0, 1.0, 'Escenario %d de %d' % (self.indice + 1,
                                                len(ESCENARIOS)),
                va='top', color=TINTA_2, fontsize=9)
        ax.text(0, 0.95, self.escenario['titulo'], va='top', color=TINTA,
                fontsize=15, fontweight='bold')
        ax.text(0, 0.88, _envolver('Que prueba:  ' + self.escenario['prueba']),
                va='top', color=TINTA, fontsize=10)
        ax.text(0, 0.80, _envolver(self.escenario['porque']), va='top',
                color=TINTA_2, fontsize=10, linespacing=1.6)

        parametros = [
            ('planificador', 'WHCA*'),
            ('ventana w', '%d pasos' % VENTANA),
            ('tamano de celda', '%.1f m' % CELDA),
            ('rejilla', '%d x %d celdas' % (info.width, info.height)),
            ('robots', '%d' % len(self.arranques)),
            ('prioridad', ', '.join(self.arranques)),
        ]
        ax.text(0, 0.30, 'Parametros', va='top', color=TINTA, fontsize=10,
                fontweight='bold')
        ax.text(0, 0.25, '\n'.join('%-15s %s' % par for par in parametros),
                va='top', color=TINTA_2, fontsize=9, linespacing=1.7,
                family='monospace')

        if self.rutas is None:
            resultado = 'sin solucion'
        else:
            filas = []
            for nombre, ruta in self.rutas.items():
                esperas = sum(1 for a, b in zip(ruta, ruta[1:]) if a == b)
                filas.append('%-6s %2d pasos, %d esperas'
                             % (nombre, len(ruta) - 1, esperas))
            total = sum(len(r) - 1 for r in self.rutas.values())
            filas.append('%-6s %2d pasos' % ('suma', total))
            resultado = '\n'.join(filas)

        ax.text(0.55, 0.30, 'Resultado', va='top', color=TINTA, fontsize=10,
                fontweight='bold')
        ax.text(0.55, 0.25, resultado, va='top', color=TINTA_2, fontsize=9,
                linespacing=1.7, family='monospace')


if __name__ == '__main__':
    revisar_instalacion()
    Demo()
    plt.show()
