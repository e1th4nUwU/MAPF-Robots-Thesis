"""
Run every instance, check it against what it is supposed to do, and draw

    python3 run.py              run all, write the figures
    python3 run.py --no-figs    just the table
    python3 run.py <nombre>     one instance

Exit status is non zero when an instance does not do what it claims, so
this can be wired into CI later without changing anything
"""

import os
import sys

import harness
import instances

from swarm_mapf.planners import whca

FIGURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figuras')


def check(instance, paths):
    """
    Compare a result against what the instance says should happen

    Args:
        instance: the entry of instances.INSTANCES
        paths: the plan, or None

    Returns:
        A list of failures, empty when the instance behaved
    """
    expect = instance['expect']
    failures = []

    if expect.get('solved') is not None:
        if expect['solved'] and paths is None:
            failures.append('se esperaba solucion y no la hubo')
        if not expect['solved'] and paths is not None:
            failures.append('se esperaba que fallara y resolvio')

    if paths is None:
        return failures

    total_waits = sum(harness.waits(path) for path in paths.values())
    if 'waits_at_least' in expect and total_waits < expect['waits_at_least']:
        failures.append('se esperaban al menos %d esperas y hubo %d'
                        % (expect['waits_at_least'], total_waits))
    if 'waits_at_most' in expect and total_waits > expect['waits_at_most']:
        failures.append('se esperaban a lo mas %d esperas y hubo %d'
                        % (expect['waits_at_most'], total_waits))
    if 'uses_cell' in expect:
        cell = tuple(expect['uses_cell'])
        if not any(cell in path for path in paths.values()):
            failures.append('nadie uso la celda %s' % (cell,))

    return failures


def run(instance, with_figure=True):
    """
    Plan one instance, check it, and draw it

    Args:
        instance: the entry of instances.INSTANCES
        with_figure: whether to write the png

    Returns:
        The list of failures
    """
    grid, starts, goals = harness.load(instance)
    paths = whca.plan(starts, goals, grid, harness.CELL_SIZE,
                      window=instances.WINDOW)
    failures = check(instance, paths)

    if paths is None:
        summary = 'sin solucion'
    else:
        summary = '  '.join(
            '%s %d/%d' % (name, len(path) - 1, harness.waits(path))
            for name, path in paths.items())

    known = instance.get('known_issue')
    if failures and known:
        state = 'PENDIENTE'
    elif failures:
        state = 'FALLA'
    else:
        state = 'ok'

    print('%-26s %-10s %s' % (instance['name'], state, summary))
    for failure in failures:
        print('%-26s           %s' % ('', failure))
    if failures and known:
        for line in known.split('. '):
            print('%-26s           %s' % ('', line.strip()))

    if with_figure:
        import draw
        os.makedirs(FIGURES, exist_ok=True)
        draw.draw(instance, grid, starts, goals, paths,
                  os.path.join(FIGURES, '%s.png' % instance['name']))

    return failures


def main():
    arguments = sys.argv[1:]
    with_figure = '--no-figs' not in arguments
    names = [a for a in arguments if not a.startswith('--')]

    chosen = ([instances.by_name(n) for n in names] if names
              else instances.INSTANCES)
    if any(instance is None for instance in chosen):
        print('No conozco esa instancia. Hay:')
        for instance in instances.INSTANCES:
            print('   ', instance['name'])
        return 2

    print('%-26s %-10s %s' % ('instancia', 'estado', 'robot pasos/esperas'))
    print('-' * 72)
    failed = 0
    for instance in chosen:
        if run(instance, with_figure) and not instance.get('known_issue'):
            failed += 1

    print('-' * 72)
    pending = sum(1 for i in chosen if i.get('known_issue'))
    print('%d de %d instancias hacen lo que dicen%s'
          % (len(chosen) - failed - pending, len(chosen),
             ', %d con defecto conocido' % pending if pending else ''))
    if with_figure:
        print('figuras en %s' % FIGURES)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
