#!/usr/bin/env python
#-*- coding: UTF-8 -*-

import unittest

import underworlds
import underworlds.server

import logging; logger = logging.getLogger("underworlds.testing")
logging.basicConfig(level=logging.DEBUG)

import time
from underworlds.tools.loader import ModelLoader
from underworlds.types import MESH, CAMERA
import os.path as path

from underworlds.helpers.transformations import compose_matrix

PROPAGATION_TIME=0.01 # time to wait for node update notification propagation (in sec)

class TestModelLoading(unittest.TestCase):
    """Test for a bug where loading a second model reset the transformation of the first model
    """

    def setUp(self):
        self.server = underworlds.server.start()
        time.sleep(0.1) # leave some time to the server to start

        self.ctx = underworlds.Context("unittest - root anchoring transformation issue")

    def test_basic_loading(self):

        world = self.ctx.worlds["test"]
        nodes = ModelLoader().load(path.join("res","tree.blend"), world="test")

        time.sleep(PROPAGATION_TIME)

        self.assertEquals(len(nodes), 2) # <BlenderRoot> and <tree>
        self.assertEquals(len(world.scene.nodes), 2)

        trees = world.scene.nodebyname("tree")
        self.assertEquals(len(trees), 1) # only one tree
        self.assertEquals(trees[0].type, MESH)

    def test_complex_loading(self):

        world = self.ctx.worlds["test"]
        nodes = ModelLoader().load(path.join("res","visibility.blend"), world="test")

        time.sleep(PROPAGATION_TIME)

        self.assertEquals(len(nodes), 8)
        self.assertEquals(len(world.scene.nodes), 8)

        self.assertEquals(len(world.scene.nodebyname("Camera1")), 1)
        cam1 = world.scene.nodebyname("Camera1")[0]
        self.assertEquals(cam1.type, CAMERA)
        self.assertFalse(cam1.hires)

        self.assertEquals(len(world.scene.nodebyname("Cube1")), 1)
        cube1 = world.scene.nodebyname("Cube1")[0]
        self.assertEquals(cube1.type, MESH)
        self.assertTrue(cube1.hires)


    def test_double_loading(self):

        world = self.ctx.worlds["test"]
        ModelLoader().load(path.join("res","tree.blend"), world="test")
        ModelLoader().load(path.join("res","tree.blend"), world="test")

        time.sleep(PROPAGATION_TIME)

        self.assertEquals(len(world.scene.nodes), 3) # one root and 2 trees

        trees = world.scene.nodebyname("tree")
        self.assertEquals(len(trees), 2) # should have 2 trees

        self.assertEquals(trees[0].hires, trees[1].hires)
        self.assertNotEquals(trees[0].id, trees[1].id)

    def test_anchoring(self):

        world = self.ctx.worlds["test"]
        nodes = ModelLoader().load(path.join("res","tree.blend"), world="test")
        tree = world.scene.nodebyname("tree")[0]
        
        self.assertEqual(tree.transformation[0,3], 0)
                
        tree.transformation = compose_matrix(None, None, None, [2, 0, 0], None)
        world.scene.nodes.update(tree)
        
        time.sleep(PROPAGATION_TIME)
        self.assertEqual(world.scene.nodes[tree.id].transformation[0,3], 2)

        # ...loading another model reset the transformation of our original
        # model
        nodes = ModelLoader().load(path.join("res","cow.blend"), world="test")
        
        time.sleep(PROPAGATION_TIME)
        self.assertEqual(world.scene.nodes[tree.id].transformation[0,3], 2)

    def tearDown(self):
        self.ctx.close()
        self.server.stop(0)

def test_suite():
     suite = unittest.TestLoader().loadTestsFromTestCase(TestModelLoading)
     return suite

if __name__ == '__main__':
    unittest.main()
