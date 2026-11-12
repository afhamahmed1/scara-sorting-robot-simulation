"""Sort rules: which bin each kind of item goes into.

Plain Python, no ROS: tested in test/test_rules.py.

A rules file lists rules in order; the first one that matches an item wins, and '*' matches
anything:

    rules:
      - {colour: red,   shape: '*', bin: 1}
      - {colour: '*',   shape: '*', bin: reject}

An item no rule matches goes to the reject bin.
"""

from scara_sort.cell import BINS

ANY = '*'


def check_rules(rules):
    """Every problem with a list of rules, as messages (empty = all good)."""
    problems = []
    for n, rule in enumerate(rules, start=1):
        missing = {'colour', 'shape', 'bin'} - set(rule)
        if missing:
            problems.append(f"rule {n}: missing {', '.join(sorted(missing))}")
        elif str(rule['bin']) not in BINS:
            problems.append(f"rule {n}: no bin {rule['bin']!r} (bins: {', '.join(BINS)})")
    return problems


def bin_for(rules, colour, shape):
    """Name of the bin an item of this colour and shape goes into: '1', '2', '3' or 'reject'."""
    for rule in rules:
        if rule['colour'] in (ANY, colour) and rule['shape'] in (ANY, shape):
            return str(rule['bin'])
    return 'reject'
