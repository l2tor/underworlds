#!/usr/bin/env python
#-*- coding: UTF-8 -*-

import logging; logger = logging.getLogger("underworlds.filter.flying")

import sys, copy
import underworlds
from underworlds.types import *
from underworlds.helpers.geometry import transformed_aabb

def setup_physics(in_scene):

    to_update = []
    for n in in_scene.nodes:
        if n.type == MESH:
            if "Monkey" in n.name:
                n.properties["physics"] = True
                print("Enabled physics for %s" % n.name)
                to_update.append(n)

    for n in to_update:
        in_scene.nodes.update(n)


def normalize_aabb(aabb):

    # invert Y direction
    aabb[0][1] = -aabb[0][1]
    aabb[1][1] = -aabb[1][1]

    tmp = zip(aabb[0], aabb[1])

    bb_min = [min(c) for c in tmp]
    bb_max = [max(c) for c in tmp]

    return bb_min, bb_max

def find_support(node, candidates):

    #aabb is normalized: first point is min, second point is max
    bb_min, bb_max = node.properties["transformed_aabb"]

    xmin = bb_min[0]
    zmin = bb_min[2]
    xmax = bb_max[0]
    zmax = bb_max[2]

    ymin = -node.aabb[0][1]

    for target in candidates[::-1]:
        tbb_min, tbb_max = target.properties["transformed_aabb"]

        txmin = tbb_min[0]
        tzmin = tbb_min[2]
        txmax = tbb_max[0]
        tzmax = tbb_max[2]

        tymax = tbb_max[1]
        translation = ymin + tymax

        if xmin < txmax and \
           zmax > tzmin and \
           xmax > txmin and \
           zmin < tzmax:
               return target, translation

    # no support!
    return None, None

def filter(in_scene, out_scene):

    static_objs = []
    dynamic_objs = []

    for n in in_scene.nodes:
        if n.type == MESH:
            aabb = transformed_aabb(in_scene, n, children = False)
            aabb = normalize_aabb(aabb)
            
            n.properties["transformed_aabb"] = aabb

            print("%s -> dynamic: %s" % (n.name, n.properties["physics"]))
            if n.properties["physics"]:
                dynamic_objs.append(n)
                print("%s -> aabb: %s" % (n.name, aabb))
            else:
                static_objs.append(n)

    # sorts static object from the highest one to the lowest one
    static_objs.sort(key=lambda node: node.properties["transformed_aabb"][1][1])

    for d in dynamic_objs:
        support, dt = find_support(d, static_objs)
        if support:
            print("%s should be on %s. Moving its center at %.2fm" % (d.name, support.name, dt))

            # be careful: do not modify 'd' (ie, the in_scene node) locally, else bounding box computation will be wrong
            node = out_scene.nodes[d.id]
            node.transformation[2][3] = dt
            out_scene.nodes.update(node)
        else:
            print("No support for %s! Leaving it alone." % d.name)


if __name__ == "__main__":

    # Manage command line options
    import argparse
    parser = argparse.ArgumentParser(description='Move flying objects to rest position'))
    parser.add_argument("input", help="underworlds world to monitor")
    parser.add_argument("output", help="resulting underworlds world")
    args = parser.parse_args()

    with underworlds.Context("flying filter") as ctx:

        in_world = ctx.worlds[args.input]
        setup_physics(in_world.scene)

        out_world = ctx.worlds[args.output]


        out_world.copy_from(in_world) # override previous content

        filter(in_world.scene, out_world.scene)

        try:
            while True:
                in_world.scene.waitforchanges()
                filter(in_world.scene, out_world.scene)

        except KeyboardInterrupt:
            print("Bye bye")



