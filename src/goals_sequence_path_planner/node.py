#!/usr/bin/env python

class Node(object):
    def __init__(self, point, parent, cost=0.0, id=0):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
        self.cost = cost
        self.id = id
