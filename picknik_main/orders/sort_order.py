from __future__ import division, print_function, absolute_import

import json
from collections import Counter
from itertools import chain
import warnings

import pandas as pd


class ContestInterface(object):
    data = pd.read_csv('items_data.csv', index_col=0)
    # These are the probabilites to correctly perform a grasp for each
    # object. Probabilities are not random, but they should be tuned
    # as testing progresses so that the robot starts picking the
    # objects with maximum expected score.
    _p_grasping_correctly = data.to_dict()['p_grasping_correctly']
    # Hopefull we won't pick up the wrong object, or move other
    # objects too often... If this is low, the sorting order will
    # tipically have the easiest items from the most crowded bins
    # first. If it gets higher, single item bins will be preferred.
    _p_mistakes = 0.2
    _possible_items = list(_p_grasping_correctly)
    del data

    # Things to consider:
    # 'first_years_take_and_toss_straw_cup',  # berkeley data had cups, amz cup
    # 'oreo_mega_stuf',  # berkeley data had double ff, amz example one
    # 'dr_browns_bottle_brush',  # (no longer) missing from berkeley
    # 'laugh_out_loud_joke_book'  # (no longer) missing from berkeley

    def __init__(self, bin_contents, work_order):
        self.bin_contents = bin_contents
        self.work_order = work_order
        self.valid = self.validate_order()

    @classmethod
    def from_json(cls, json_fn):
        """Initialize from json file name."""
        with open(json_fn) as fh:
            data = json.load(fh)
        bin_contents, work_order = data["bin_contents"], data["work_order"]
        return cls(bin_contents, work_order)

    def to_json(self, json_fn, sort=True):
        """Save (sorted) order (aka ContestInterface) to json file."""
        data = {"bin_contents": self.bin_contents,
                "work_order": self.sorted_order() if sort else self.work_order}

        with open(json_fn, "w") as fh:
            json.dump(data, fh, indent=4, separators=(',', ': '), sort_keys=True)

    def validate_order(self):
        """Check if order makes any sense.

        Returns
        =======
        bool
            Order fullfills amazon rules? In particular:
            - All item names are in the set of possible item names.
            - At least two bins will only contain one item.
            - At least two bins will contain only two items.
            - At least two bins will contain three or more items.
            - Each bin only has a single item that's a picking target. I
              assume that means there are as many order items as bins.
        """
        # Check at least two bins have 1, 2 or >=3 items
        n_items = Counter(min(len(i), 3) for i in self.bin_contents.values())
        ok = all(ni >= 2 for ni in n_items.values())

        # Check item names
        names = list(chain.from_iterable(self.bin_contents.values())) + [i["item"] for i in self.work_order]
        ok = ok and set(names).issubset(self._possible_items)

        # check whether there are as many picking targets as bins
        ok = ok and len(self.work_order) == len(self.bin_contents)
        if not ok:
            warnings.warn("Order does NOT fulfill rules, sorting, etc will"
                          " be disabled and things MAY FAIL randomly.", RuntimeWarning)
        return ok


    def sorted_order(self):
        """Sort from hightest to lowest expected score.

        The expected score for each object is the product of the
        probability of grasping it correctly (tweak them in
        `items_data.csv`) times the score for doing it right (depends
        on the number of objects in the bin). Then, if there're
        multiple objects in the bin, we remove the product of the
        number of objects in the bin times the probablity of removing
        an object we shouldn't have touched and the score we'd lose.
        """
        if not self.valid:
            return self.work_order

        expected_scores = []
        for wi in self.work_order:
            bin, item = wi['bin'], wi['item']
            n_items = len(self.bin_contents[bin])
            score = self._p_grasping_correctly[item] * [10, 15, 20][min(3, n_items) - 1]
            if n_items > 1:
                # If there are more than one item in the bin we may
                # lose points for moving a non-target item out of a
                # shelf bin (and not replacing it)
                score -= self._p_mistakes * 12 * n_items  # Also increases with n_items
            expected_scores.append(score)
        # DSU pattern to sort by expected scores
        return [y for (x, y) in sorted(zip(expected_scores, self.work_order), reverse=True)]


if __name__ == '__main__':
    import argparse


    parser = argparse.ArgumentParser()
    parser.add_argument("input_filename", type=str,
                        help="filename for the json order")
    parser.add_argument("output_filename", type=str,
                        help="filename for the sorted order")

    args = parser.parse_args()

    contest = ContestInterface.from_json(args.input_filename)
    contest.to_json(args.output_filename, sort=True)
