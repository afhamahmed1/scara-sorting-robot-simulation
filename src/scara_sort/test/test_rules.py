"""Unit tests for rules.py and the rules files."""

import os

import pytest
import yaml

from scara_sort.rules import bin_for, check_rules

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


def load(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)['rules']


@pytest.mark.parametrize('colour, shape, expected', [
    ('red', 'cube', '1'), ('red', 'prism', '1'), ('green', 'cylinder', '2'),
    ('blue', 'prism', '3'), ('yellow', 'cube', 'reject'), ('unknown', 'unknown', 'reject'),
])
def test_sort_by_colour(colour, shape, expected):
    assert bin_for(load('sort_by_colour.yaml'), colour, shape) == expected


@pytest.mark.parametrize('colour, shape, expected', [
    ('red', 'cube', '1'), ('yellow', 'cube', '1'), ('blue', 'cylinder', '2'),
    ('green', 'prism', '3'), ('red', 'unknown', 'reject'),
])
def test_sort_by_shape(colour, shape, expected):
    assert bin_for(load('sort_by_shape.yaml'), colour, shape) == expected


def test_first_matching_rule_wins():
    rules = [{'colour': 'red', 'shape': 'cube', 'bin': 1},
             {'colour': 'red', 'shape': '*', 'bin': 2}]
    assert bin_for(rules, 'red', 'cube') == '1'
    assert bin_for(rules, 'red', 'prism') == '2'


def test_no_matching_rule_means_reject():
    assert bin_for([{'colour': 'red', 'shape': '*', 'bin': 1}], 'blue', 'cube') == 'reject'


def test_the_rules_files_are_valid():
    for name in ('sort_by_colour.yaml', 'sort_by_shape.yaml'):
        assert check_rules(load(name)) == []


def test_bad_rules_are_reported():
    problems = check_rules([{'colour': 'red', 'bin': 1}, {'colour': 'red', 'shape': '*', 'bin': 7}])
    assert problems == ['rule 1: missing shape', "rule 2: no bin 7 (bins: 1, 2, 3, reject)"]
