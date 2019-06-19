#!/usr/bin/env python
#-*- coding: UTF-8 -*-

import underworlds
from underworlds.helpers.geometry import get_bounding_box_for_node
from underworlds.types import MESH
from underworlds.tools.loader import ModelLoader
import logging; loadLogger = logging.getLogger("underworlds.model_loader")
import os.path as path
import sys
import underworlds.server
from math import *
import numpy as np
import json
from underworlds.helpers.transformations import compose_matrix

def load(objName):
    #Get Underworlds path and put together path for blender file.
    #Using simplified models to speed up loading for Underworlds and increase stability
    if objName == "giraffe":
        objName = "giraffe2"
    if objName == "Monkey":
        objName = "Monkey2"
        
    filename= path.join("res", "%s.blend" % (objName))
    #filename = "res\%s.blend" %  (objName)

    loadLogger.setLevel(logging.INFO)
    logging.basicConfig(level=logging.INFO)

    #Load model
    nodes = ModelLoader().load(filename, world="test")
    ids = [node.id for node in nodes if node.name == objName]
    if ids:
        myID = ids[0]
        msg = "test load_move: ID = %s" % (myID)
        logging.info(msg)
        return myID
    logging.info("test load_move: Could not get ID")
    
def transObj(id, x, y, z, xrot, yrot, zrot, xScale, yScale, zScale):
    with underworlds.Context("test load_move") as ctx:
        world = ctx.worlds["test"]
        scene = world.scene
        node = scene.nodes[id]
        
        #Convert from degrees to radians and negative
        xrot = xrot * (-0.01745329252)
        yrot = yrot * (-0.01745329252)
        zrot = zrot * (-0.01745329252)
        
        node.transformation = compose_matrix([xScale, yScale, zScale], None, [xrot, yrot, zrot], [x, y, z], None)
        
        scene.nodes.update(node)
    
if __name__ == "__main__":
    jsonPath = path.join("res", "load.json")
    with open(jsonPath) as f:
    
        pparams = json.loads(json.dumps(json.load(f)))
        print pparams
        objsToLoad = pparams["loadable_objects"]
        
        for item in objsToLoad:
            objName = item["filename"]
            uwID = load(objName)
            
            #Note that Y and Z are reversed in the .json file
            x = item["position"]["x"]
            z = item["position"]["y"]
            y = item["position"]["z"]
            xrot = item["rotation"]["x"]
            zrot = item["rotation"]["y"]
            yrot = item["rotation"]["z"]
            xScale = item["scaling"]["x"]
            zScale = item["scaling"]["y"]
            yScale = item["scaling"]["z"]
        
            transObj(uwID, x, y, z, xrot, yrot, zrot, xScale, yScale, zScale)
        
        print "Done"
