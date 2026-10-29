# Banco de pruebas del planificador, fuera de ROS

Instancias pequeñas, dibujadas a mano, donde la respuesta correcta se
conoce **antes** de correr nada. Sin Gazebo, sin `colcon`, sin `source`.

```bash
python3 run.py              # todas, con figuras
python3 run.py --no-figs    # sólo la tabla
python3 run.py nicho-cerca  # una sola
```

Las figuras quedan en `figuras/`.

## Por qué existe

Los tres escenarios reales resultaron **demasiado fáciles**: cooperar
cuesta +0 y +1 paso, y ningún robot espera nunca. Un escenario que no
reta al algoritmo no puede demostrar que funciona.

Aquí la instancia se diseña para exigir exactamente una cosa, y el
resultado esperado está escrito en el propio archivo.

## Que no necesite ROS no es casualidad

Toda la capa de planeación importa `math`, `heapq`, `itertools` y a sí
misma. Nada más. El único objeto con forma de ROS es el `OccupancyGrid`,
y lo toca por seis atributos, así que `harness.Grid` —veinte líneas— lo
sustituye entero.

Eso **demuestra** la separación que afirma la tesis, en lugar de sólo
declararla.

## El formato

Legible en papel y pegable en el capítulo:

```
#######
#A...B#      #  obstáculo        A  arranque del robot A
###.###      .  libre            a  meta del robot A
#######
```

Cuando una meta cae sobre el arranque de otro robot, el dibujo no puede
llevar los dos caracteres; esas instancias dan la meta en `goals`.

`priority` es el orden de planificación, mayor primero. Es la única
perilla de WHCA\*.

## Las figuras

Un plan espacio-temporal no cabe en un dibujo plano: la misma celda
significa cosas distintas en pasos distintos. La salida es poner el
tiempo en el color — **un tono por robot, y la luminosidad sube con el
paso** — con el número del paso escrito en la celda.

Así se ven las dos cosas que importan sin animación:

- una celda que dice `B 2-3` es un robot que **esperó**, o sea que cedió;
- celdas vecinas con el mismo paso en tonos distintos son un **cruce**.

Una celda que usan dos robots en pasos distintos se parte en bandas, una
por robot. Las celdas compartidas son justo las interesantes.

## Qué prueba cada instancia

| instancia | qué exige |
|---|---|
| `corredor-sin-salida` | WHCA\* es incompleto: aquí **debe** fallar |
| `nicho-lejos` | hay solución y **no la encuentra** — el límite del método |
| `nicho-cerca` | con el nicho al alcance, el de abajo cede y sí resuelve |
| `cuello-simultaneo` | **la espera**: dos equidistantes de un hueco de una celda |
| `prioridad-invertida` | el orden cambia la solución, no sólo quién gana |
| `meta-en-el-cuello` | `hold_until`: un robot estacionado sigue ocupando |
| `cruce-en-cuarto-abierto` | con espacio, cooperar es desviarse, no esperar |

`nicho-lejos` y `nicho-cerca` son el mismo pasillo con el nicho movido
**una celda**. En uno el método se rinde y en el otro resuelve. Esa
pareja sola vale una figura del capítulo.

## Estados

- **ok** — hace lo que dice.
- **FALLA** — regresión: algo se rompió.
- **PENDIENTE** — defecto conocido, documentado en la instancia. No
  cuenta como regresión, pero sale en la tabla para que no se olvide.

Hoy hay uno: `meta-en-el-cuello`. Ver la nota ahí.

## Qué **no** prueba

El planificador, no la integración. Nada de lo que mordió en ROS
—`cost_map` pintando robots, `pure_pursuit` estacionándose— aparece
aquí. Es complemento, no reemplazo.
