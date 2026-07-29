# Notas de sesión — Versión Greedy (GNFC + WHCA*)
> Fecha: mayo 2026 · Branch: main · Robots: alvin, teodoro, simon (justina_base)

---

## Arquitectura general del sistema

```
GNFC Goal Allocator  ──── goal_pose ────►  Pure Pursuit (por robot)
                                               │
priority_manager ─── /robot_priorities ─►  WHCA* coordinator ─── /whca_replan ─► Pure Pursuit
                                               │
cost_map_node ──── /inflated_map ─────────►  WHCA* planner (por robot)
              ──── /cost_map    ─────────►        │
                                               GetPlan service
health_monitor ─── /{robot}/health ──────►  cost_map_node (pinta obstáculo)
                                         └──► Pure Pursuit (kill ▸ detiene)
```

**Flujo completo de una ejecución:**
1. `gnfc_goal_allocator` genera una cuadrícula de puntos en la zona meta y asigna el más cercano a cada robot (GNFC greedy).
2. Cada robot recibe su `goal_pose` por `/{robot}/goal_pose` (QoS TRANSIENT_LOCAL).
3. `pure_pursuit` entra a `SM_PLAN_PATH` y llama al servicio `path_planning/plan_path` de su instancia de `whca_star`.
4. `whca_star` corre A* espacio-temporal con ventana, respetando reservaciones de robots de mayor prioridad.
5. `pure_pursuit` sigue la ruta con lookahead puro y republica `goal_reached` al terminar.
6. `whca_coordinator` dispara `/replan_needed` periódicamente para que los robots replanifiquen dentro de la ventana de coordinación.

---

## Bug 1 — Teodoro se desvía en Gazebo aunque RViz parece correcto

### Síntoma
Teodoro seguía la ruta perfectamente en RViz pero en Gazebo chocaba con las paredes del escenario de torres. El robot hacía paradas bruscas seguidas de giros en su lugar, acumulando impulso y pasando de largo.

### Causa raíz (dos problemas independientes)

**1a. `LOOKAHEAD_DIST` demasiado pequeño (0.25 m)**

El pure pursuit buscaba el punto en la ruta que estuviera al menos 0.25 m de distancia. A esa escala, cuando el robot se acercaba a un waypoint antes de la curva, el punto de lookahead ya estaba "al otro lado" de la esquina: el robot giraba bruscamente, perdía inercia, se detenía casi en seco y la inercia del modelo físico lo llevaba más lejos de lo que RViz indicaba (RViz no simula inercia).

**1b. Wait-actions de WHCA* generaban waypoints duplicados**

WHCA* codifica "espera en celda" como pasos consecutivos en `(r, c, t)` con la misma posición `(r, c)`. `_cells_to_path` los convertía todos a la misma coordenada `(x, y)`, por lo que `pure_pursuit` los ignoraba (el lookahead los sobrepasaba al instante). Así los "tiempos de espera" no tenían efecto alguno: el robot avanzaba sin ceder el paso, y terminaba chocando contra otro robot en la intersección.

### Correcciones

**`pure_pursuit.py`** — Aumentar lookahead:
```python
LOOKAHEAD_DIST = 0.4  # era 0.25 m
```

**`whca_star.py`** — Deduplicar waypoints de espera en `_cells_to_path`:
```python
prev_rc = None
for (r, c, _) in path_cells:
    if (r, c) == prev_rc:
        continue  # descarta duplicados de wait-actions
    prev_rc = (r, c)
    ...
```

> **Lección:** Un simulador con física realista amplifica cualquier discontinuidad en la señal de control. El pure pursuit necesita un lookahead que amortigüe curvas antes de llegar a ellas, no después.

---

## Bug 2 — Reasignación infinita de metas (los robots no se detienen)

### Síntoma
Al llegar a la zona meta, `gnfc_goal_allocator` reasignaba inmediatamente el mismo punto al mismo robot. Los logs mostraban el ciclo:
```
Assigned alvin → (8.30, 0.80)
alvin reached goal zone
Assigned alvin → (8.30, 0.80)
...
```

### Causa raíz
El método `_allocate_and_publish` corría en timer de 1 Hz. Al recibir `goal_reached`, el robot era quitado de `needs_goal`, pero en la siguiente iteración del timer `needs_goal` volvía a incluirlo porque nunca había un mecanismo de "estacionamiento" permanente.

### Corrección — `gnfc_goal_allocator.py`

```python
self.parked = set()  # robots que ya llegaron a la zona meta

def _goal_reached_cb(self, msg, robot_name: str):
    if msg.data and robot_name not in self.parked:
        self.parked.add(robot_name)
        self.needs_goal.discard(robot_name)

def _allocate_and_publish(self):
    pending = self.needs_goal - self.parked  # ignorar robots estacionados
    if not pending:
        return
```

> **Lección:** Un mecanismo de asignación de metas necesita estado persistente de "tarea completada", no sólo estado de "necesita meta ahora".

---

## Bug 3 — Sólo 1-2 de 3 robots reciben su meta inicial

### Síntoma
Tras el arranque, sólo simon (o a veces alvin) recibía su `goal_pose`. Los demás robots esperaban indefinidamente en `SM_WAIT_FOR_NEW_GOAL`. Los logs de GNFC mostraban que los 3 goals sí se publicaban, pero los nodos de `pure_pursuit` de los otros robots no los recibían.

### Causa raíz — Race condition de descubrimiento DDS

ROS 2 Jazzy usa DDS para matching publisher-subscriber. El matching tarda entre 100–500 ms. GNFC publicaba los goals ~200 ms después del arranque, antes de que el handshake de DDS estuviera completo para todos los robots. Los mensajes se enviaban al "vacío" y se perdían silenciosamente — sin error, sin warning.

### Corrección — QoS `TRANSIENT_LOCAL` (latched) en publisher y subscriber

```python
# gnfc_goal_allocator.py
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
self.goal_publishers[name] = self.create_publisher(
    PoseStamped, f'/{name}/goal_pose', _LATCHED_QOS)

# pure_pursuit.py
_latched = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
self.sub_goal_pose = self.create_subscription(
    PoseStamped, 'goal_pose', self.callback_goal_pose, _latched)
```

Con `TRANSIENT_LOCAL`, el publisher guarda el último mensaje en caché. Cuando un subscriber se conecta tarde (incluso segundos después), recibe el mensaje cacheado automáticamente.

> **Lección:** Cualquier topic de configuración o meta que se publica una sola vez al arranque DEBE usar `TRANSIENT_LOCAL` en ROS 2. El QoS por defecto (`VOLATILE`) descarta mensajes si el subscriber aún no existe.

---

## Bug 4 — Obstáculo de robot muerto bloquea corredor entero (dos iteraciones)

### Síntoma (iteración 1)
Cuando simon moría, el obstáculo pintado en el mapa bloqueaba un corredor completo entre dos torres. teodoro ya no podía pasar y se quedaba sin ruta.

**Iteración 1 de fix:** Se separó `DEAD_ROBOT_CLEARANCE_M` del `inflation_radius` general (0.5 m → 0.20 m clearance):
```python
DEAD_ROBOT_RADIUS_M    = 0.30  # radio físico circunscrito de justina
DEAD_ROBOT_CLEARANCE_M = 0.20  # radio total = 0.50 m
```

### Síntoma (iteración 2)
Usuario: "aún sigue siendo demasiado grande". Con 0.50 m de radio total, en corredores de ~1.0 m de ancho, el obstáculo dejaba sólo 0.50 m libres a cada lado — insuficiente para que un robot de 0.39 m de ancho pasara con el margen de inflación estándar.

**Iteración 2 de fix:** Reducir clearance a 0.10 m (radio total 0.40 m):
```python
DEAD_ROBOT_CLEARANCE_M = 0.10  # radio total = 0.40 m
```

### Geometría del robot justina

| Dimensión | Valor |
|---|---|
| Ancho del cuerpo | 0.39 m (0.35 m + 0.04 m ruedas) |
| Radio circunscrito | √(0.225² + 0.195²) ≈ 0.30 m |
| Radio inflado (mapa estático) | 0.50 m |
| Radio muerto (cuerpo) | 0.30 m |
| Radio muerto + clearance | 0.40 m |

> **Lección:** El radio de obstáculo para robot muerto debe ser independiente del inflation_radius del mapa estático. Los obstáculos estáticos pueden ser conservadores; un obstáculo de robot muerto en un corredor activo necesita ser más ajustado.

---

## Bug 5 — Teodoro declarado muerto mientras gira en su lugar (falso positivo)

### Síntoma
Tras detectarse la muerte de simon, teodoro recibía `/replan_needed`, paraba, esperaba 1.5 s, y luego empezaba a girar hacia el nuevo plan. Al cabo de 3–4 s sin desplazamiento xy, el `health_monitor` lo declaraba muerto. Los logs mostraban:
```
[HEALTH] teodoro DIED at (x, y) — no movement for 3.0s
```
…a pesar de que teodoro estaba activamente girando y comandando velocidades angulares.

### Causa raíz (dos factores combinados)

1. **`DEATH_TIMEOUT = 3.0 s`** era menor que el tiempo de una vuelta de 90° en seco: π/(2 · 0.5 rad/s) ≈ 3.14 s, más 1.5 s de espera de replanificación = 4.64 s sin movimiento xy.

2. **Detección basada únicamente en posición xy (odometría)**. Un robot girando en su eje no mueve su centroide, por lo que el health_monitor no lo detectaba como "activo".

### Corrección — `swarm_health_monitor.py`

**a) Aumentar timeout a 6.0 s:**
```python
# Debe superar: replan-sleep (1.5 s) + giro 90° a w_max=0.5 rad/s (3.1 s) = 4.6 s
DEATH_TIMEOUT = 6.0
```

**b) Reiniciar el timer de muerte cuando llega cmd_vel activo:**
```python
def _cmd_vel_callback(self, msg: Twist, robot_name: str):
    if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
        state = self.states[robot_name]
        if not state.is_killed:
            state.last_moved_time = self.get_clock().now()
```

> **Lección:** La detección de "robot muerto" no debe basarse sólo en posición. Un robot que gira en su lugar está vivo y activo. `cmd_vel` es el indicador correcto de que el controlador está funcionando.

---

## Bug 6 — Robot muerto intenta planear A* desde su propia celda bloqueada

### Síntoma
Secuencia observada en logs:
```
[COST MAP] simon dead — rebuilding maps and requesting replan
[COST MAP] teodoro dead — rebuilding maps and requesting replan  ← falso positivo (bug 5)
teodoro: replan_callback → needs_replan = True
teodoro: sleep 1.5 s (esperando mapa reconstruido)
...1.5 s después...
teodoro: SM_PLAN_PATH — planeando desde (x,y) a meta
teodoro: kill ← llega DESPUÉS del sleep
A*: no path found!  ← teodoro ya está en celda bloqueada
```

### Causa raíz
Dos señales llegaban al mismo robot en orden invertido:
1. `/replan_needed` disparaba el `_replan_callback` → `needs_replan = True`
2. La corrutina `SM_PLAN_PATH` entraba al sleep de 1.5 s (bloqueante en `get_clock().sleep_for`)
3. El topic `/{robot}/kill` llegaba **durante** ese sleep → `_kill_callback` no podía ejecutarse (el nodo no hacía spin durante el sleep)
4. Al terminar el sleep, `SM_PLAN_PATH` continuaba ignorando que el robot ya estaba muerto

### Corrección — `pure_pursuit.py`

Dos guards de `robot_alive` en `SM_PLAN_PATH`:

```python
elif state == SM_PLAN_PATH:
    # 1) Verificar ANTES de entrar a planning
    if not self.robot_alive:
        state = SM_WAIT_FOR_NEW_GOAL
        continue
    self.needs_replan = False
    self.map_changed  = False
    ...
    future = self.clt_plan_path.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    # 2) Verificar DESPUÉS de spin_until_future_complete
    #    (la señal kill puede haber llegado durante el spin)
    if not self.robot_alive:
        state = SM_WAIT_FOR_NEW_GOAL
        continue
    path = future.result().plan
```

> **Lección:** `get_clock().sleep_for()` y `spin_until_future_complete()` son puntos de bloqueo donde callbacks de otros topics no se procesan. Siempre verificar flags críticos antes y después de cualquier operación bloqueante.

---

## Decisiones de arquitectura WHCA*

### Por qué g_score en 3D (r, c, t) dentro de la ventana

Un g_score 2D `(r, c)` asignaría al nodo `start` un costo de 0. Cualquier "espera" en start tendría costo 1 > 0, así que el algoritmo nunca elegiría esperar ahí. Con g_score 3D `(r, c, t)`, start es `(r, c, 0)` y una espera es `(r, c, 1)` — son nodos diferentes, ambos con costo finito y explorable.

```python
key = (nr, nc, nt) if nt < self._window else (nr, nc)
```

Fuera de la ventana se vuelve 2D para evitar explosión combinatoria: sin ese cambio, el grafo tiene `H × W × T` nodos donde T puede ser arbitrariamente grande.

### Por qué TEMPORAL_BUFFER = 20

Pure pursuit puede tardar ~3 s en completar un giro de 90° a `w_max = 0.5 rad/s`. Con pasos de planificación de ~0.05 m / ciclo, eso equivale a ~60 pasos en los que el robot no avanza. Reservar sólo el paso exacto `t` en que el plan predice ocupar una celda no es suficiente — la ejecución real puede retrasarse hasta 20 pasos de más. `TEMPORAL_BUFFER = 20` extiende cada reservación 20 pasos hacia el futuro, dando margen a los robots de menor prioridad para planear alrededor.

### Por qué SAFETY_CELLS = 7

- Radio circunscrito del robot: 0.30 m
- Resolución del mapa: 0.05 m/celda
- Celdas mínimas para cubrir el radio: ceil(0.30/0.05) = 6
- Buffer de seguimiento: +1 celda
- Total: 7 celdas × 0.05 m = 0.35 m de margen por robot → 0.70 m de separación centro a centro

Valores de 8+ generaban caminos infeasibles en los corredores del escenario de torres (1.0 m de ancho = 20 celdas; con 8 celdas por robot se necesitan 32 celdas de separación para que dos robots pasen, imposible en un corredor de 20).

### Coordinación por stagger (WHCA* coordinator)

El coordinador espacia los replans entre robots en `stagger_sec = 0.8 s` por orden de prioridad. Esto permite que el robot de mayor prioridad publique sus reservaciones en `/space_time_reservations` antes de que los de menor prioridad planifiquen, evitando conflictos de primer ciclo.

---

## Parámetros ajustados y su justificación

| Parámetro | Archivo | Valor anterior | Valor actual | Razón |
|---|---|---|---|---|
| `LOOKAHEAD_DIST` | pure_pursuit.py | 0.25 m | 0.40 m | Anticipar curvas; reducir brakings bruscos |
| `DEATH_TIMEOUT` | health_monitor.py | 3.0 s | 6.0 s | Cubrir giro 90° (3.1 s) + sleep replan (1.5 s) |
| `DEAD_ROBOT_RADIUS_M` | cost_map.py | — | 0.30 m | Radio físico real (circunscrito) de justina |
| `DEAD_ROBOT_CLEARANCE_M` | cost_map.py | 0.50 m | 0.10 m | Mínimo para no bloquear corredores activos |
| `DEFAULT_GOAL_GRID_SPACING` | gnfc_goal_allocator.py | 0.3 m | 0.8 m | ≥ diámetro del robot (0.60 m) + 0.20 m clearance |
| `SAFETY_CELLS` | whca_star.py | — | 7 | Cubre radio robot + 1 celda buffer; máximo viable en torres |
| `TEMPORAL_BUFFER` | whca_star.py | — | 20 | Cubre latencia de giro en ejecución real |
| `RESERVATION_TTL_SEC` | whca_star.py | — | 12.0 s | Reservaciones de robot muerto expiran automáticamente |

---

## Factores que incrementan las colisiones en modo GNFC

- **Goal grid spacing pequeño** (0.3 m): los puntos de la cuadrícula de destino están dentro del radio del robot; dos robots que buscan metas adyacentes convergen hacia el mismo punto físico.
- **Wait-actions no aplicadas** (bug 1b): WHCA* planifica con tiempos de espera pero pure pursuit los saltaba → los robots no cedían paso en intersecciones.
- **Falso positivo de muerte** (bug 5): Un robot vivo declarado muerto activa reconstrucción del mapa y señal de replan → cascada de replans simultáneos → todos los robots se detienen y giran al mismo tiempo → intersecciones sin cobertura temporal.

---

## Bug 7 — Colisión persistente en chokepoints: el problema fundamental WHCA* + pure_pursuit

### Síntoma
Con todos los bugs 1–6 corregidos (rutas frescas, reservaciones bidireccionales, robots vivos detectados correctamente), los robots **siguen colisionando** en chokepoints estrechos antes/dentro de la zona meta. Las rutas en RViz convergen visualmente al mismo punto y los robots se golpean en Gazebo.

### Causa raíz — Disonancia tiempo-espacio

WHCA* es un planificador **espacio-temporal**: cada celda en el plan tiene un timestep `t` asociado. Una ruta válida significa "robot A está en celda X en t=10, robot B está en X en t=30 — sin colisión porque están separados en tiempo".

Pero `pure_pursuit` es un controlador **puramente espacial**: recibe una secuencia de puntos `(x, y)` y los persigue a la velocidad máxima posible. **Ignora completamente el campo de tiempo del plan**.

Resultado: WHCA* genera planes donde "B espera 20 pasos antes de cruzar", pero pure_pursuit nunca ejecuta esa espera. Los wait-actions (celdas duplicadas en el plan) se filtran en `_cells_to_path` porque pure_pursuit no los puede honrar (su lookahead las salta inmediatamente). Sin esperas reales, ambos robots llegan al chokepoint al mismo tiempo y colisionan.

### Por qué los fixes "obvios" no resuelven el problema

#### Fix intentado 1: bloqueo espacial puro (todos los timesteps)

Cambio en `_build_blocked_set`: en vez de bloquear celdas en `t..t+TEMPORAL_BUFFER`, bloquearlas en **todos** los timesteps del window.

```python
spatial_cells = set()
for x, y, _ in zip(res.x_cells, res.y_cells, res.time_steps):
    r, c = self._world_to_cell_xy(x, y)
    for dr in range(-SAFETY_CELLS, SAFETY_CELLS+1):
        for dc in range(-SAFETY_CELLS, SAFETY_CELLS+1):
            spatial_cells.add((r+dr, c+dc))
for (r, c) in spatial_cells:
    for t_block in range(self._window+1):
        blocked.add((r, c, t_block))
```

**Por qué falla:** En el escenario de torres, el corredor de aproximación al goal zone es estrecho (~1 m). Con `SAFETY_CELLS=7` (radio de 0.35 m), bloquear el camino de robot A inhabilita el corredor entero. Robot B no encuentra ruta → se activa el fallback `_whca(start, goal, set())` que ignora reservaciones y devuelve la ruta óptima — la **misma** ruta que A.

Resultado: paths idénticos en RViz, colisión en Gazebo.

#### Fix intentado 2: sincronización al inicio (`/start_navigation` global)

Hipótesis: si todos planean antes de que cualquiera se mueva, sus reservaciones reflejan posiciones consistentes y no hay datos stale.

Cambio: nuevo state `SM_WAIT_GO` en pure_pursuit que espera `/start_navigation` antes de seguir el path. Coordinador publica `/start_navigation` cuando todos los robots han planeado.

**Por qué falla:** Aunque las reservaciones iniciales son consistentes, los robots **arrancan simultáneamente** y convergen al chokepoint al mismo tiempo. El sync barrier no ayuda — empeora las cosas comparado con stagger natural, porque elimina el head start del robot de mayor prioridad.

#### Fix intentado 3: departure stagger por-robot

Cambio: en vez de `/start_navigation` global, cada robot tiene su propio `/{robot}/start_navigation` con delay incremental:
- Alvin sale en T=4s
- Teodoro sale 2s después de Alvin
- Simon sale 2s después de Teodoro

**Por qué falla parcialmente:** El plan de Simon se calculó en T=2.5s, cuando Alvin estaba al inicio. Para cuando Simon sale en T=8s, Alvin ya recorrió 3 m. Simon sigue su plan stale, que pasa por donde Alvin se encontraba — colisión.

#### Fix intentado 4: replan fresco antes de salir

Cambio: cuando llega `/{robot}/start_navigation`, en vez de ir directo a `SM_FOLLOWING_PATH`, regresar a `SM_PLAN_PATH` para replanificar con estado actual.

**Mejora pero no resuelve:** Aunque el plan se refresca, sigue siendo un plan **espacial**. Si Alvin está en el chokepoint en T=8s y Simon planea en T=8s, el plan de Simon evita la posición actual de Alvin pero atraviesa el chokepoint. Para cuando Simon llega al chokepoint (1 segundo después), Alvin se ha movido — y la posición exacta depende de las velocidades relativas.

#### Fix intentado 5: Departure stagger grande (6+ segundos)

Funciona pero **rompe el propósito de WHCA***. Si los robots salen serialmente con head start grande, no hay coordinación multi-agente real — es navegación secuencial. La tesis es sobre MAPF, no sobre robots tomando turnos.

### Lección fundamental

**Un planificador espacio-temporal requiere un controlador time-aware para funcionar correctamente.** Las opciones son:

1. **Pure pursuit time-aware**: el path lleva timestamps por waypoint, pure_pursuit ajusta velocidad para cumplir el horario. Si va adelantado, espera; si atrasado, va a v_max.
2. **Reactive collision avoidance**: capa adicional sobre pure_pursuit que detecta robots cercanos y cede el paso por prioridad. WHCA* da rutas mostly-correctas, la capa reactiva resuelve los casos límite.
3. **CBS con FollowTimedPath**: arquitectura distinta donde un planificador centralizado (CBS) emite trayectorias completas con timing, y un controlador time-aware las ejecuta. (Ya hay stubs en `cbs_coordinator.py`.)

Para esta tesis: la opción 2 (reactive yield por prioridad) es la más práctica — agregar al pure_pursuit existente.

---

## Cambios desplegados para mitigar colisiones

| Cambio | Archivo | Efecto |
|---|---|---|
| `replan_period_sec` 15.0 → 3.0 s | scenario.launch.py | Reservaciones siempre frescas durante navegación |
| `stagger_sec` 1.0 → 2.0 → 1.0 s | scenario.launch.py | Tiempo entre planificaciones de robots; ajuste con replan-fresh |
| Heartbeat de reservaciones cada 4 s | whca_star.py | Robot parqueado mantiene su posición bloqueada para otros |
| Heartbeat resetea time_steps a 0 | whca_star.py | Bloqueo correcto independiente del paso temporal |
| Bloqueo espacial puro en `_build_blocked_set` | whca_star.py | Fuerza separación espacial cuando es factible (con fallback) |
| Estado `SM_WAIT_GO` en pure_pursuit | pure_pursuit.py | Espera señal de departure antes de moverse |
| Replan fresco al salir de `SM_WAIT_GO` | pure_pursuit.py | Plan refleja posiciones actuales de los demás |
| `/{robot}/start_navigation` por-robot | whca_coordinator.py | Permite stagger de salida por prioridad |
| `departure_stagger_sec` 6.0 → 2.0 s | whca_coordinator.py | Compromiso: head start mínimo combinado con replan-fresh |

### Parámetros propuestos para experimentación en tesis

Estos valores son los más sensibles para reproducir/comparar resultados:

| Parámetro | Valor actual | Rango a explorar | Métrica que afecta |
|---|---|---|---|
| `LOOKAHEAD_DIST` (pure_pursuit) | 0.40 m | 0.20–0.60 m | Smoothness del path tracking |
| `SAFETY_CELLS` (whca_star) | 7 | 5–10 | Trade-off feasibility vs. clearance |
| `TEMPORAL_BUFFER` (whca_star) | 20 | 10–40 | Margen de timing en wait-actions |
| `window_size` (whca_star) | 80 | 40–120 | Horizonte de coordinación |
| `replan_period_sec` (coord.) | 3.0 s | 1.0–10.0 s | Frescura de reservaciones |
| `stagger_sec` (coord.) | 1.0 s | 0.5–3.0 s | Orden de planificación |
| `departure_stagger_sec` (coord.) | 2.0 s | 0.0–8.0 s | Separación temporal en salida |
| `goal_grid_spacing` (gnfc) | 0.8 m | 0.6–1.5 m | Densidad de metas en zona |

### Métricas a reportar en tesis

- **Tasa de éxito** (% de runs sin colisión) por configuración
- **Tiempo total de misión** (T desde primera salida hasta último goal_reached)
- **Distancia total recorrida** por robot (eficiencia vs. ruta óptima individual)
- **Número de replans** por robot durante una misión
- **Tiempo de espera** acumulado por robot (cuando cede paso o espera departure)

---

## Solución final: Reactive yield-by-priority sobre WHCA*

Tras descartar departure stagger como cop-out, la arquitectura final combina:

1. **WHCA* con bloqueo espacial** — produce planes mostly-disjoint en RViz
2. **Replanificación cada 3 s** — reservaciones siempre frescas
3. **Heartbeat de reservaciones** — robots parqueados siguen bloqueando su celda
4. **Capa reactiva en pure_pursuit** — cuando dos planes convergen físicamente, el robot de menor prioridad se detiene

### Implementación de la capa reactiva (`pure_pursuit.py`)

⚠️ **Bug crítico encontrado en primera versión:** suscribirse a `/{r}/odom` para obtener posiciones de otros robots da resultados **inválidos** porque cada robot publica odometría en SU PROPIO frame `{robot}/odom` (origen local en su posición de spawn). Comparar la posición de simon (frame `simon/odom`) con la de alvin (frame `alvin/odom`) no es una distancia geométrica válida — siempre da números que no corresponden a la realidad. **Por eso el yield no se disparaba.**

**Fix:** usar TF lookup en frame `map` para todos los robots:

```python
YIELD_DISTANCE_M = 0.80  # 2 radios (0.30 m c/u) + 0.20 m buffer

def _should_yield(self, robot_p):
    """True cuando un robot de mayor prioridad está demasiado cerca."""
    if self._robot_name not in self._priorities:
        return False
    my_idx = self._priorities.index(self._robot_name)
    for r in ROBOT_NAMES:
        if r == self._robot_name or r not in self._priorities:
            continue
        if self._priorities.index(r) >= my_idx:
            continue
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{r}/base_link', rclpy.time.Time())
            pos = numpy.asarray([t.transform.translation.x,
                                  t.transform.translation.y])
        except TransformException:
            continue
        if numpy.linalg.norm(pos - robot_p) < YIELD_DISTANCE_M:
            return True
    return False

# En el loop de pure_pursuit (antes de calcular control):
if self._should_yield(robot_p):
    self.publish_and_save_data(robot_x, robot_y, robot_a,
                               robot_x, robot_y, 0.0, 0.0)
    continue  # frenar y reintentar el ciclo
```

### Eliminación del barrier `SM_WAIT_GO`

El estado `SM_WAIT_GO` y los signals `/{robot}/start_navigation` se eliminaron por completo. Razón: agregaban 4 s de espera al inicio sin resolver la colisión real (que era el bug del frame de odometría).

**Flujo simplificado:**
1. Cada robot recibe `whca_replan` con stagger de 1 s (T=0.5, 1.5, 2.5)
2. Cada robot planea inmediatamente con las reservaciones de los anteriores
3. Cada robot empieza a moverse en cuanto termina su plan (T≈0.7, 1.7, 2.7)
4. La capa reactiva (`_should_yield`) maneja convergencia en chokepoints en tiempo real

Total wait: ~0.7 s para el primer robot, ~2.7 s para el último. Mucho mejor que los 4+ s del barrier.

### Por qué esto sí funciona

- **WHCA* sigue siendo el cerebro**: produce los planes coordinados que minimizan conflictos
- **La capa reactiva sólo actúa en casos límite**: chokepoints estrechos donde dos planes deben cruzarse
- **Prioridad asimétrica = no deadlock**: sólo el robot de menor prioridad cede; el de mayor prioridad sigue → siempre hay progreso
- **Sin departure stagger**: los robots salen casi simultáneamente (`departure_stagger_sec=0`)
- **No defeats el propósito de MAPF**: WHCA* es responsable de la coordinación global; el yield es defense-in-depth

### Topología del flujo final

```
GNFC asigna goals (latched QoS)
   │
   ▼
pure_pursuit recibe goal → espera whca_replan del coordinador
   │
   ├── T=0.5: alvin recibe whca_replan
   ├── T=1.5: teodoro recibe whca_replan (ve reservaciones de alvin)
   └── T=2.5: simon recibe whca_replan (ve reservaciones de alvin y teodoro)
   │
   ▼
SM_PLAN_PATH: whca_star planea con bloqueo espacial puro
   │
   ├──► publica reservaciones en /space_time_reservations
   │       (heartbeat cada 4 s con time_steps=0)
   │
   ▼
SM_SMOOTH_PATH → SM_FOLLOWING_PATH (sin barrier, sale inmediatamente)
   │
   ▼
bucle pure_pursuit (200 Hz):
   robot_p = TF lookup map → base_link
   if _should_yield(robot_p):              ← capa reactiva
       Twist(0, 0)  (yield a mayor prioridad)
       continue
   v, w = lookahead control
   │
   ▼
goal_reached → parked (no más asignaciones, heartbeat sigue bloqueando posición)
```

### Parámetros finales recomendados (modo whca)

```python
# scenario.launch.py:
'replan_period_sec':       3.0,   # frescura de coordinación
'stagger_sec':             1.0,   # planning stagger (orden de prioridad)
# pure_pursuit:
'LOOKAHEAD_DIST':          0.40,  # m
'YIELD_DISTANCE_M':        0.80,  # m (= 2 × radio robot + buffer)
# whca_star:
'window_size':             80,
'SAFETY_CELLS':            7,
'TEMPORAL_BUFFER':         20,
```

## Resumen de cambios por archivo

| Archivo | Cambios |
|---|---|
| `pure_pursuit.py` | `LOOKAHEAD_DIST` 0.25→0.40; latched QoS en `sub_goal_pose`; guards `robot_alive` en SM_PLAN_PATH (×2); estado nuevo `SM_WAIT_GO`; subscripción a `start_navigation`; replan fresco al recibir start signal |
| `whca_star.py` | Deduplicación de waypoints en `_cells_to_path`; bloqueo espacial puro (todos los timesteps del window); heartbeat de reservaciones cada 4 s con time_steps reseteados a 0 |
| `whca_coordinator.py` | Publishers `/{robot}/start_navigation` por-robot; departure stagger configurable; barrier de "todos planearon antes de cualquier salida" |
| `gnfc_goal_allocator.py` | Set `parked`; latched QoS en publishers; `_allocate_and_publish` excluye robots estacionados |
| `cost_map.py` | `DEAD_ROBOT_RADIUS_M=0.30`, `DEAD_ROBOT_CLEARANCE_M=0.10`; gradiente de costo usa clearance cells |
| `swarm_health_monitor.py` | `DEATH_TIMEOUT` 3.0→6.0 s; subscripción a `cmd_vel`; `_cmd_vel_callback` reinicia timer durante rotación |
| `scenario.launch.py` | `replan_period_sec` 15.0→3.0; `stagger_sec` 2.0→1.0; agregado `departure_stagger_sec: 2.0` |

---

# Bug 8 — GNFC no determinista por marco de referencia local del `/odom`

## Síntoma

> "GNFC asigna las posiciones de forma un poquito random, por lo que a veces
> cruzan y a veces no y eso no me deja ver si va a cumplir o no el objetivo
> de dejar pasar."

Ejecutar dos veces seguidas el mismo escenario daba asignaciones de meta
distintas a cada robot — incluso con la misma `goal_zone` y las mismas
posiciones de spawn.  Esto impedía repetibilidad para validar WHCA*.

## Causa

`gnfc_goal_allocator.py` inicializaba `self.robot_positions[name] = (0,0)`
y se actualizaba vía `/{name}/odom`.  Pero **cada robot publica `/odom` en
su propio marco local** (origen en su punto de spawn), no en `map`.

Resultado:
1. En el primer tick del timer, los 3 robots aparecen en (0, 0) en el
   diccionario.  El ordenamiento `sorted(... key=distancia)` desempata por
   orden alfabético / inserción → pseudo-aleatorio.
2. Para cuando llega el primer `/odom`, la asignación ya está hecha y se
   queda *latched* en TRANSIENT_LOCAL.

Es exactamente el mismo bug de marco que detectamos en `pure_pursuit`
(Bug 7 / sección "Frame de odometría"), pero esta vez afecta la asignación
en lugar del yielding.

## Fix

`gnfc_goal_allocator.py` ahora acepta parámetros `initial_x_<robot>` /
`initial_y_<robot>` por cada robot, los guarda en `self.spawn_positions`,
y:

- Inicializa `self.robot_positions[name] = self.spawn_positions[name].copy()`
  (no `(0,0)`), así el primer tick ya tiene posiciones correctas.
- Convierte odom a marco mapa sumando el offset: `map = spawn + odom_local`.

`scenario.launch.py` rellena estos parámetros automáticamente desde
`SCENARIO_ROBOTS[scenario]` cuando `planner ∈ {whca, cbs}`, así que el
usuario no tiene que pasarlos manualmente.

**Resultado**: cada lanzamiento asigna exactamente las mismas metas a
los mismos robots — tests reproducibles.

# Rediseño del escenario `maze` para forzar coordinación

## Motivación

> "Maze necesitará un re-work porque es demasiado sencillo; no nos va a
> dejar ver bien la conducta del algoritmo."

El maze original era de **dos corredores** (norte y sur), separados por una
pared central sólida.  Cada corredor tenía **una** chicane.  Las metas se
ponían arbitrariamente en cada corredor.  Problemas:

- Cada robot vivía dentro de su corredor — no había forma de que **dos
  robots se cruzaran** salvo en una sola chicane angosta.
- La pared central impedía cualquier conflicto N↔S.
- Teodoro estaba completamente aislado.

## Nuevo diseño (3 corredores + 2 gaps centrales)

```
y=+6 ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
     [alvin →]                          (y=+4)
                          ║N            bar_n2 (x=0, y=4.8..6)
     ║S                          ║S    bar_n1/n3 (x=±5, y=2..3.2)
y=+2 ═══════════ GAP ═══════════       gap x=-1..1 (2 m)
                ║N                       bar_m1 (x=-3, y=0.8..2)
     [teodoro →]                        (y=0)
                          ║S            bar_m2 (x=+3, y=-2..-0.8)
y=-2 ═══════════ GAP ═══════════       gap x=-1..1 (2 m)
     ║N                          ║N    bar_s1/s3 (x=±5, y=-3.2..-2)
                          ║S            bar_s2 (x=0, y=-6..-4.8)
     [simon →]                          (y=-4)
y=-6 ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
     x=-10                  x=+10
```

- 3 corredores (norte y∈[2,6], medio y∈[-2,2], sur y∈[-6,-2]).
- Cada corredor tiene **3 chicanes alternadas** (S-N-S en norte y sur,
  N-S en medio) → slalom obligatorio.
- Las paredes divisorias **dejan un gap único de 2 m centrado en x=0**.
- **Una sola zona de meta** está en el centro-este del corredor medio
  (x∈[7.5, 9.0], y∈[-1, 1]).

## Por qué fuerza coordinación

- alvin (norte) y simon (sur) **no pueden ir directo** al goal — deben
  bajar / subir a través del gap central.
- Los gaps (2 m de ancho geométrico) tienen sólo **1 m de espacio libre
  para el centro del robot** después de la inflación 0.5 m del cost_map
  → un solo robot puede pasar a la vez.
- Por lo tanto si dos robots llegan al gap a la vez, **WHCA\* debe
  secuenciar** (uno cede, el otro pasa).
- En el corredor medio (alvin descendiendo + teodoro siguiente + simon
  subiendo) los tres terminan compartiendo la última recta E-W antes
  del goal.

## Dimensionado de chicanes (importante para el planner)

`motion_planning.launch.py` usa `inflation_radius = 0.5 m`.  Una barra
vertical de altura *B* en un corredor bloquea un *ancho efectivo* de
`B + 1.0 m` (la barra más 0.5 m de inflación a cada lado).

Con corredores de 4 m de alto, capamos las barras a **1.2 m de altura**
para garantizar 1.8 m libres del lado opuesto — espacio holgado para el
robot de 0.6 m.  Las barras se "pegan" alternadamente a un divisor o al
otro, así el slalom es real pero el path siempre existe.

| Componente | Altura | Inflación efectiva | Espacio libre robot |
|---|---|---|---|
| Chicane corredor | 1.2 m | 2.2 m | 1.8 m |
| Gap central (divisor) | 2.0 m | — | 1.0 m (centro robot) |
| Ancho corredor | 4.0 m | — | 3.0 m (centro robot) |

## Posiciones de spawn nuevas

| Robot | x | y | Corredor |
|---|---|---|---|
| alvin   | -8.0 | +4.0 | norte |
| teodoro | -8.0 |  0.0 | medio |
| simon   | -8.0 | -4.0 | sur |

# Zonas de meta fijas por escenario

Tabla constante en `scenario.launch.py` (`SCENARIO_GOAL_ZONES`):

| Escenario | (x_min, x_max, y_min, y_max) | Spacing |
|---|---|---|---|
| towers | (7.5, 9.0, -1.5, 1.5) | 0.8 |
| rocks  | (7.5, 9.0, -1.5, 1.5) | 0.8 |
| maze   | (7.5, 9.0, -1.0, 1.0) | 0.8 |

Cuando `planner ∈ {whca, cbs}`, el lanzamiento añade automáticamente:
- `gnfc_goal_allocator` con la zona del escenario **y** las posiciones de
  spawn de los 3 robots (para asignación determinista — ver Bug 8).
- `goal_zone_visualizer` con la misma zona (rectángulo verde en RViz).

El render del goal en Gazebo (`show_gazebo_goals:=true`) también usa la
misma zona vía el helper nuevo `_goal_marker_for_zone`.

## Comandos de prueba canónicos

```bash
# Rocks con WHCA* — el escenario "evidente" para ver path interleaving
ros2 launch swarm_bringup scenario.launch.py \
  scenario:=rocks planner:=whca show_gazebo_goals:=true

# Maze con WHCA* — stress test de coordinación en los gaps centrales
ros2 launch swarm_bringup scenario.launch.py \
  scenario:=maze planner:=whca show_gazebo_goals:=true

# Baselines independientes (sin coordinación) para comparar
ros2 launch swarm_bringup scenario.launch.py scenario:=rocks planner:=independent
ros2 launch swarm_bringup scenario.launch.py scenario:=maze  planner:=independent
```

---

# Bug 9 — Lookahead blocking (WHCA*) y la trampa del bloqueo full-path

## Síntoma observado

En maze, al primer ciclo de replan Teodoro registraba `blocked=7080`
celdas y no encontraba ruta.  Pocos segundos después incluso Alvin
(supuestamente la prioridad más alta) aparecía con `blocked=7029`.
Sólo un robot avanzaba a la vez, los demás se quedaban con paths
vacíos (`0 points`).

## Causa raíz

`_build_blocked_set` originalmente expandía **toda** la ruta reservada
de cada robot de mayor prioridad con un kernel `SAFETY_CELLS=7`.  Una
ruta típica en maze tiene ~325 celdas; el kernel es `(2·7+1)² = 225`.
La cota superior por ruta: 325 × 225 ≈ **73 125 celdas** marcadas.

En maze los corredores navegables son ~16 celdas de ancho — un único
bloqueo full-path tapa el corredor entero.  Geométricamente imposible
que un robot subordinado encuentre paso.

## Fix — bloqueo *lookahead* (sólo 2 m hacia adelante)

`whca_star.py` ahora:
1. Hace `tf_buffer.lookup_transform('map', f'{robot}/base_link', …)`
   por cada robot de mayor prioridad para obtener su posición real.
2. Encuentra el `start_idx` más cercano dentro de su ruta reservada
   (proyección por distancia euclidiana).
3. Sólo expande `LOOKAHEAD_CELLS=40` celdas hacia adelante (2 m).

Cota nueva: 40 × 225 = **9 000 celdas por robot** (~8× menos).

```python
LOOKAHEAD_CELLS = 40   # 40 × 0.05 m = 2 m

# por cada robot de mayor prioridad:
end_idx = min(start_idx + LOOKAHEAD_CELLS, len(res.x_cells))
for x, y in zip(res.x_cells[start_idx:end_idx],
                res.y_cells[start_idx:end_idx]):
    r, c = self._world_to_cell_xy(x, y)
    for dr in range(-SAFETY_CELLS, SAFETY_CELLS + 1):
        for dc in range(-SAFETY_CELLS, SAFETY_CELLS + 1):
            blocked.add((r + dr, c + dc))
```

Las celdas más allá del lookahead quedan libres: el robot subordinado
puede planear *atrás* del líder con ≥2 m de gap.  La capa física
(`EMERGENCY_STOP_M`, `YIELD_DISTANCE_M`) sigue protegiendo el corto
alcance — la planeación es para coordinación, no para safety.

Dependencia nueva añadida en `mapf_coordinator/package.xml`:
```xml
<depend>tf2_ros</depend>
```

## Limitación remanente — Higher-priority no ve a lower-priority

`_build_blocked_set` *salta* las reservaciones de igual o menor
prioridad.  Si dos robots entran a un corredor en sentido contrario,
el superior planea atravesar la posición actual del inferior — el
inferior cede (yield) pero como está parado en el corredor sin
espacio lateral, el superior acaba parando por `EMERGENCY_STOP_M` →
deadlock.

Es la misma limitación fundamental documentada en Bug 7.  Para esta
tesis el modo WHCA* asume corredores con espacio lateral; en single-
file la solución correcta es CBS (timed paths que sí coordinan
swaps).

---

# Bug 10 — Prioridad por distancia (regla invertida)

## Síntoma

En rocks: Simon iba acercándose a su meta y Teodoro estaba más lejos.
Por la regla anterior **"más lejos = mayor prioridad"**, Teodoro tenía
prioridad ⇒ Simon (cerca, baja prioridad) le cedía el paso a Teodoro
(lejos, alta prioridad).  Pero Simon ya estaba prácticamente sobre el
chokepoint y al frenar quedaba atravesado en él; Teodoro al entrar le
chocaba por detrás.  Ambos morían en cascada.

En maze: Teodoro (lejos del goal pero en el corredor medio) era casi
siempre mayor prioridad que Alvin/Simon en sus corredores N/S.
Como su ruta atravesaba el gap central que necesitaban los otros dos,
Teodoro **bloqueaba el gap** mientras los otros le cedían — quedaban
atascados detrás indefinidamente.

## Causa raíz

`priority_manager.py` calculaba `distances[r] = ‖goal − pos‖` y
ordenaba **descendente**:

```python
ordered = sorted(ROBOT_NAMES, key=lambda r: -distances[r])
```

Filosofía original: "darle camino al que tiene más por recorrer".  En
la práctica produce dos patologías:

1. El robot que va adelante físicamente tiene **menor** prioridad y no
   se quita ⇒ el de atrás (alta prioridad) lo embiste.
2. Robots cerca del goal nunca terminan de cruzar el último chokepoint
   porque siempre hay alguien más lejos pidiendo paso primero.

## Fix — invertir la regla: cerca del goal = alta prioridad

```python
def _key(r):
    d = distances[r]
    return (d == 0.0, d)  # parked/unassigned last, then ASCENDING dist
ordered = sorted(ROBOT_NAMES, key=_key)
```

Justificación:
- El robot más cerca del goal **termina antes** y se parquea
  (`gnfc.parked`), liberando el corredor para los demás.
- Soluciona la inversión "líder con baja prioridad" — el líder ahora
  manda y los seguidores se mantienen detrás.
- Robots sin meta asignada (`distance == 0.0`) van al final para no
  contaminar el orden.

---

# Bug 11 — Yield distance demasiado corto + sin filtro direccional

## Síntoma

Aún con prioridades correctas, en corredores de rocks el robot
subordinado no frenaba a tiempo: el de mayor prioridad ya estaba a
≤0.65 m antes de que el de menor prioridad iniciara su frenado.
Resultado: parón en seco → impacto por inercia → ambos en falla.

Adicionalmente: un robot de mayor prioridad **detrás** activaba el
yield del de adelante, dejándolo parado indefinidamente aunque el
trasero nunca se le acercara (iba en otra dirección).

## Fix — yield con cono frontal + más margen

`pure_pursuit.py`:

```python
YIELD_DISTANCE_M = 1.20   # antes 0.80 m
                          # = 2·r (0.60) + ~1.2 s de frenado a v_max=0.5 m/s
FRONT_CONE_RAD   = 1.57   # ±90° respecto a heading propio
```

`_should_yield(robot_p, robot_a)`:

```python
delta = pos_other - robot_p
bearing = atan2(delta.y, delta.x)
rel_angle = bearing - robot_a  # normalize to (-π, π]
if r_idx < my_idx and dist < YIELD_DISTANCE_M and abs(rel_angle) < FRONT_CONE_RAD:
    return True  # mayor prioridad delante de nosotros — ceder
```

- `EMERGENCY_STOP_M = 0.65` se mantiene como red de seguridad
  (omnidireccional, ignora prioridad y cono).
- `YIELD_DISTANCE_M = 1.20` da ~0.6 s extra de margen comparado con
  0.80 m.
- Cono frontal evita el caso "alguien detrás me paraliza"; si el de
  mayor prioridad va detrás, su propia lógica de prioridad (cerca del
  goal) y su replan le permitirán rebasar/redirigir.

## Parámetros nuevos / actualizados

| Parámetro | Archivo | Valor anterior | Valor actual | Razón |
|---|---|---|---|---|
| `LOOKAHEAD_CELLS` | whca_star.py | — (full path) | 40 | Bloqueo sólo 2 m delante → no tapa corredor entero |
| Regla prioridad | priority_manager.py | `-dist` (desc) | `(parked, dist)` (asc) | Cerca de goal = alta prioridad → líder manda |
| `YIELD_DISTANCE_M` | pure_pursuit.py | 0.80 m | 1.20 m | Frenado anticipado en corredores |
| `FRONT_CONE_RAD` | pure_pursuit.py | — | 1.57 rad | Sólo ceder a robots delante |

## Comportamiento esperado tras estos cambios

- **Towers**: igual que antes (sigue funcionando perfecto, corredores
  anchos).
- **Rocks**: el robot más cerca del goal pasa primero, el de atrás se
  queda 1.2 m atrás hasta que el líder despeje.  Sin rear-end.
- **Maze**: Alvin/Simon (en sus corredores N/S, lejos del goal central)
  tienen menor prioridad inicial.  Teodoro (corredor medio, también
  lejos pero más cerca por geometría) lidera y libera el gap primero.
  Conforme cualquiera se acerca al goal sube su prioridad y se le da
  paso.

## Caso aún no resuelto — corredor single-file con tráfico oncoming

Si dos robots planean cruzarse en un corredor sin espacio lateral, el
de menor prioridad se detiene; el de mayor prioridad acaba parando
también por `EMERGENCY_STOP_M`.  Es la limitación inherente de
WHCA* + pure_pursuit señalada en Bug 7.  El plan a futuro:

1. **CBS con FollowTimedPath** (ya hay stubs) — coordinación temporal
   real, permite "esperar a que pase y luego cruzar".
2. **Reroute reactivo del higher-priority** cuando hay deadlock
   detectado (lower-priority parado >5 s en su corredor).
3. **Symmetric blocking opcional** — que el higher-priority también
   evite la posición actual del lower-priority en el plan
   (re-route por la otra esquina del corredor si la geometría lo
   permite).

---

# Bug 12 — Path arrows desfasados respecto al robot en RViz

## Síntoma

> "La posición de las flechas de dónde se ubican los robots en RViz
> sigue sin ser coherente; aunque tenemos el inflado de mapas no parece
> ser coherente la disposición con lo que tenemos en Gazebo."

En maze (con paths largos + `replan_period_sec=3.0`), las flechas
del path en RViz empezaban hasta ~1.5 m detrás del robot real.  El
usuario interpretaba esto como un problema de frame o de offset entre
el mapa de RViz y el mundo de Gazebo, e incluso como inflación
"flotando" donde no debería.  La consecuencia fue mala interpretación
visual: lo que parecía inflación era de hecho la pared misma, y el
robot acababa estampándose contra ella sin que el usuario lo
anticipara desde RViz.

## Causa raíz — Path stale durante la ventana de replan

`whca_star` publicaba el path al planear, y un timer cada 0.5 s
re-publicaba el **mismo** `Path` mensaje completo para que RViz no lo
descartara.  El path contiene todas las waypoints desde la posición
del robot al momento de planear, hasta la meta.

Como `replan_period_sec=3.0` y `v_max=0.5 m/s`, el robot recorre hasta
1.5 m del path entre replans.  Las flechas en RViz sin embargo seguían
mostrándose desde la posición original del plan — no desde la
posición actual del robot.  Sumado al `stagger_sec=1.0 s` entre
planeaciones, el desfase aparente entre flechas y robot es muy
visible y rompe la confianza en lo que se ve.

## No es un bug de frames ni de map server

- TF chain `map → {name}/odom → {name}/base_link` está bien
  configurada (offset estático por robot en `scenario.launch.py`).
- `OccupancyGrid.info.origin = (-10, -6)` es consistente con el yaml.
- `_world_to_grid` pinta el PGM con row 0 en `y=+ARENA_Y`;
  `nav2_map_server` lo voltea verticalmente al cargar, así que la
  `OccupancyGrid` resultante tiene row 0 en `y=-ARENA_Y` (correcto).
- Geometría idéntica entre `_build_maze` (Gazebo) y
  `_build_scenario_map_image` (PGM) — los bars usan los mismos
  `(cx, cy, lx, ly)` en ambos.

Lo único desfasado eran las flechas porque venían de un mensaje
viejo.

## Fix — Recortar el path traversed antes de re-publicar

`whca_star._republish_trimmed_path` reemplaza al timer anterior:

```python
def _republish_trimmed_path(self):
    if not self._last_path.poses:
        self._path_pub.publish(self._last_path)
        return
    try:
        t = self.tf_buffer.lookup_transform(
            'map', f'{self._robot_name}/base_link', rclpy.time.Time())
        rx, ry = t.transform.translation.x, t.transform.translation.y
    except TransformException:
        self._path_pub.publish(self._last_path)
        return

    # Closest waypoint = inicio del segmento aún no recorrido
    best_i, best_d2 = 0, float('inf')
    for i, ps in enumerate(self._last_path.poses):
        dx = ps.pose.position.x - rx
        dy = ps.pose.position.y - ry
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2, best_i = d2, i

    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.poses = self._last_path.poses[best_i:]
    self._path_pub.publish(msg)
```

Las flechas ahora empiezan exactamente donde está el robot (≤ 1
celda = 0.05 m de error) y se ven coherentes en RViz frente a la
posición en Gazebo.

## Nota — Distinguir "pared" vs "inflación" visualmente

Con `inflation_radius=0.6 m` y `cost_radius=1.0 m` el `inflated_map`
expande cada obstáculo +12 celdas; el `cost_map` añade un gradiente
de 20 celdas más.  En RViz **ambos** se ven como halo gris alrededor
del muro negro.  Sin un display dedicado del mapa estático (sólo
`/map`) es muy fácil confundirlos.

Para depuración: añadir un display `Map` extra suscrito a `/map` (no
`/inflated_map` ni `/cost_map`) para ver únicamente las paredes
reales.  El `motion_planning.rviz` ya tiene "Static Map" suscrito a
`/map` — basta activarlo y desactivar los otros para verificar
choques reales.

## Confirmación empírica — POSE_DEBUG en maze

Tras añadir un log `POSE_DEBUG` temporal en `whca_star._plan_cb`,
los tres robots reportaron en su primera planificación:

```
alvin   req=(-8.00, 5.00) tf=(-8.00, 5.00) delta=(0.00,0.00)
teodoro req=(-8.00,-0.00) tf=(-8.00,-0.00) delta=(0.00,0.00)
simon   req=(-8.00,-5.00) tf=(-8.00,-5.00) delta=(0.00,0.00)
map_origin=(-10.00,-6.00)  res=0.05  size=400×240
```

`delta = (0, 0)` para los tres ⇒ pure_pursuit y whca_star ven la
**misma** posición TF, idéntica al spawn de Gazebo.  No hay bug de
frames, ni de `map_origin`, ni de dimensiones de PGM.

## Causa real del "offset" percibido — falta del modelo del robot en RViz

`motion_planning.rviz` tenía **un solo** `RobotModel` display
suscrito a `/robot_description`.  Pero cada robot publica a
`/{name}/robot_description` (vía `PushRosNamespace`), así que el
display estaba vacío y **no se veía ningún cuerpo de robot en RViz**.

El usuario percibía "offset" al comparar el inicio del path con su
intuición de dónde estaba el robot — pero sin un display que mostrara
los cuerpos, esa intuición era guesswork basado en geometría de
corredores e inflación gris.  En towers (corredores anchos) el error
de percepción era pequeño; en maze (corredores estrechos + inflación
densa) era obvio.

## Fix — Tres RobotModel displays, uno por robot

`motion_planning.rviz` ahora declara tres `RobotModel`:

| Display name  | Description Topic              |
|---|---|
| Alvin Model   | `/alvin/robot_description`     |
| Teodoro Model | `/teodoro/robot_description`   |
| Simon Model   | `/simon/robot_description`     |

Cada uno usa la TF del frame_prefix `{name}/` automáticamente y
aparece donde el TF lo coloque en `map` — exactamente donde está
en Gazebo.

Con esto las flechas y los robots están visiblemente alineados,
desaparece la percepción de offset, y se distinguen choques reales
contra paredes de inflación que el robot apenas roza.

## Bug 12b — El offset real venía de `swarm_viz_bridge.py`

Tras agregar los `RobotModel` displays se hizo evidente que el offset
no era percepción — había clusters densos de flechas (100 mensajes
históricos por robot, `Keep: 100` en el `Odometry` display) que sí
estaban desplazados respecto al cuerpo del robot.

Esos clusters venían de `/{name}/odom_map`, publicado por
`swarm_viz_bridge.py`.  El bridge convierte `/odom` (frame local,
empieza en 0,0 al spawn) a `map` sumando la posición de spawn:

```python
out.pose.pose.position.x += ox
out.pose.pose.position.y += oy
```

Pero los valores hardcoded en `SCENARIO_STARTS` **no coincidían** con
`_ROBOTS_MAZE` en `scenario.launch.py`:

| Robot   | Bridge (mal) | Real (`_ROBOTS_MAZE`) |
|---------|--------------|----------------------|
| alvin   | (-8.0,  2.5) | (-8.0,  5.0)         |
| simon   | (-8.0,  1.0) | (-8.0, -5.0)         |
| teodoro | (-8.0, -2.5) | (-8.0,  0.0)         |

Towers/rocks coincidían (`_ROBOTS_OPEN` con y=1.5/0/-1.5), por eso el
bug era exclusivo de maze.

### Fix

Corregir `SCENARIO_STARTS['maze']` para que coincida con
`_ROBOTS_MAZE`:

```python
'maze': {
    'alvin':   (-8.0,  5.0),
    'teodoro': (-8.0,  0.0),
    'simon':   (-8.0, -5.0),
},
```

A futuro: pasar estas posiciones como parámetros desde el launch (ya
hay infraestructura para esto en `gnfc_goal_allocator` — `initial_x_*`
/ `initial_y_*` params) en vez de hardcoded.  Una sola fuente de
verdad evita estas inconsistencias.

### Por qué el log POSE_DEBUG mostraba `delta=(0,0)`

`whca_star` y `pure_pursuit` usan TF directamente (`map →
{name}/base_link`), y ese chain está correcto: el
`static_transform_publisher` toma su offset de `robot['x']`,
`robot['y']` de `_ROBOTS_MAZE`.  El bug no afectaba TF, sólo el
mensaje `Odometry` reconstruido por el bridge para visualización.
Por eso planeación y navegación funcionaban (las trayectorias eran
correctas), pero las flechas históricas en RViz aparecían movidas.
