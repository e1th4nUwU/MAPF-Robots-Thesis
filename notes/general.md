## Path smoothing
- We had to adjust alpha and beta to get good results. We found that alpha = x and beta = y5 worked well for our dataset because otherwise intertia on the path would cause it to deviate too much from the original path, and we wanted to maintain a balance between smoothness and fidelity to the original path. We ended up using alpha = 

---

## WHCA* coordination bugs y fixes (2026-05-11)

### Bugs observados
1. **Cascade de 0-puntos tras muerte de robot** — el obstáculo del robot muerto (radio 0.40 m = 6 celdas cuerpo + 2 celdas clearance) cubría la posición de robots vecinos. A* iniciando dentro de un obstáculo no puede expandir ningún vecino → 0 puntos → los tres robots se bloqueaban en cadena.
2. **Colisión física por fallback descoordinado** — cuando WHCA* no encontraba ruta coordinada en el corredor estrecho del maze, reintentaba con el conjunto de celdas bloqueadas vacío → el robot tomaba exactamente la misma ruta que el robot de mayor prioridad → colisión garantizada.
3. **Alvin no cedía el paso** — el mecanismo `_should_yield` solo frenaba ante robots de *mayor* prioridad. Alvin (prioridad más alta) nunca frenaba ante nadie y chocaba contra Teodoro detenido.
4. **Muerte prematura por yielding legítimo** — `DEATH_TIMEOUT = 6 s` era demasiado corto: un robot parado esperando que el corredor se despejara (≤ 3 s por ciclo de replan) podía acumular 6 s sin movimiento y ser declarado muerto falsamente.

### Fixes aplicados

| Archivo | Cambio | Razón |
|---|---|---|
| `whca_star.py` | `_nearest_free()` + start-escape en `_plan_cb` | Si la celda de inicio está dentro de un obstáculo, BFS al vecino libre más cercano antes de planear |
| `whca_star.py` | Eliminado fallback descoordinado | En lugar de reintentar con blocked=∅ (colisión), retorna path vacío → `pure_pursuit` espera en `SM_WAIT_FOR_NEW_GOAL` hasta el siguiente ciclo de replan |
| `pure_pursuit.py` | `EMERGENCY_STOP_M = 0.65 m` en `_should_yield` | Frena ante *cualquier* robot a ≤ 0.65 m sin importar prioridad (capa de prevención física); el yield proactivo a robots de mayor prioridad sigue a 0.80 m |
| `swarm_health_monitor.py` | `DEATH_TIMEOUT`: 6 s → 10 s | Permite hasta 3 ciclos de replan (3 s c/u) de yielding legítimo antes de declarar muerto |

### Cómo funciona el paso en fila india por corredores estrechos (maze)
- El corredor libre tras inflación 0.6 m es ~16 celdas (0.80 m). Con `SAFETY_CELLS = 7` dos rutas necesitan ≥ 14 celdas de separación → tres robots en paralelo es físicamente imposible.
- Flujo correcto: robot de mayor prioridad planea y avanza → robot de menor prioridad no encuentra ruta coordinada → retorna path vacío → se detiene → el coordinador (periodo 3 s) reintenta → para entonces el corredor está libre → encuentra ruta y avanza.
- La capa de emergency stop previene que el robot de mayor prioridad empuje a los detenidos mientras esperan.

### Parámetros relevantes actuales
- `SAFETY_CELLS = 7` (0.35 m por robot, 0.70 m mínimo entre centros de rutas)
- `EMERGENCY_STOP_M = 0.65 m` (frena ante cualquier robot)
- `YIELD_DISTANCE_M = 0.80 m` (frena ante robots de mayor prioridad)
- `replan_period_sec = 3.0 s` (whca_coordinator, configurado en scenario.launch.py)
- `DEATH_TIMEOUT = 10.0 s`
- `DEAD_ROBOT_RADIUS_M = 0.30 m`, `DEAD_ROBOT_CLEARANCE_M = 0.10 m` → radio total 0.40 m
