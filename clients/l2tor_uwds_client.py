#!/usr/bin/env python
# coding: utf-8 

import threading
import socket
from threading import Thread
import re
import time
import log_formatter as lf
import sys
import json
from math import *
import numpy as np
import os.path as path

import underworlds
from underworlds.helpers.geometry import get_bounding_box_for_node
from underworlds.helpers.transformations import compose_matrix
from underworlds.types import *
from underworlds.tools.loader import ModelLoader, logger
from underworlds.tools.primitives_3d import *
import logging; loadLogger = logging.getLogger("underworlds.model_loader")
from underworlds.tools.spatial_relations import *
import underworlds.server

class l2tor_uwds_client:

    def __init__(self, logger):
        self.uwds_logger = logger
        # loadLogger = logging.getLogger("underworlds.model_loader")
        logger.addHandler(self.uwds_logger.handlers[0])
        self.uwIDs = {}
        self.l2torIDs = {}

    def updtObjPos(self, pparams):
        self.uwds_logger.info("Underworlds L2TOR client: Updating position")

        # Note that Z and Y are flipped due to tablet game

        id = pparams["id"]
        posX = pparams["position"]["x"]
        posZ = pparams["position"]["y"]
        posY = pparams["position"]["z"]
        rotX = pparams["rotation"]["x"]
        rotZ = pparams["rotation"]["y"]
        rotY = pparams["rotation"]["z"]
        scaleX = pparams["scaling"]["x"]
        scaleZ = pparams["scaling"]["y"]
        scaleY = pparams["scaling"]["z"]

        if id in self.uwIDs:
            uwID = self.uwIDs[id]
            self.__transObj(uwID, posX, posY, posZ, rotX, rotY, rotZ, scaleX, scaleY, scaleZ)

            self.uwds_logger.info("Underworlds L2TOR client: Position updated")

            #objRel = compute_rel_constr(self.l2torIDs)

            #logging.info("Underworlds L2TOR client: Spatial relations calculated")

            #self.sendRelMessage(objRel, socket)

            #logging.info("Underworlds L2TOR client: Message sent to Interaction Manager")

    #Return all the spatial relations
    def getRel(self):
        return self.__compute_rel_constr()

    def isOnline(self):
        return True

    def reset(self):
        with underworlds.Context("l2tor_interface") as ctx:
            ctx.reset()

    def writeChildInfoToLog(self, data):
        self.uwds_logger.info("[CHILD_INFO]%s" % json.dumps(data))

    #Takes in the scene as a dictionary and loads it into the 'l2tor' world in underworlds
    def loadScene(self, pparams):
        self.reset()
        self.uwIDs = {}
        self.l2torIDs = {}
        objsToLoad = pparams["loadable_objects"]
        objsToCreate = pparams["creatable_objects"]

        self.uwds_logger.info("Underworlds L2TOR client: Loading Scene")
        count = 0
        
        loadedObjs = {}
        
        #load all objects
        for item in objsToLoad:
            objName = item["filename"]
            
            #Work out if the object has been loaded before. If it has copy the existing node to increase loading speed.
            if objName in loadedObjs:
                cpyID = loadedObjs[objName]
                uwID = self.__copyNode(cpyID)
            else:
                uwID = self.__load(objName)
                loadedObjs[objName] = uwID

            self.uwIDs.update({item["id"]:uwID})
            self.l2torIDs.update({uwID:item["id"]})
            
            #Note that Z and Y are flipped due to tablet game
            
            posX = item["position"]["x"]
            posZ = item["position"]["y"]
            posY = item["position"]["z"]
            rotX = item["rotation"]["x"]
            rotZ = item["rotation"]["y"]
            rotY = item["rotation"]["z"]
            scaleX = item["scaling"]["x"]
            scaleZ = item["scaling"]["y"]
            scaleY = item["scaling"]["z"]
            
            self.__transObj(uwID, posX, posY, posZ, rotX, rotY, rotZ, scaleX, scaleY, scaleZ)
            
        for item in objsToCreate:
            if item["uwdsCreate"] != "none":
                posX = item["position"]["x"]
                posZ = item["position"]["y"]
                posY = item["position"]["z"]
                rotX = item["rotation"]["x"]
                rotZ = item["rotation"]["y"]
                rotY = item["rotation"]["z"]
                name = item["id"]
                
                if item["uwdsCreate"] == "box":
                    scaleX = (item["scaling"]["x"])*50
                    scaleZ = (item["scaling"]["y"])*50
                    scaleY = item["scaling"]["z"]
                    uwID = self.__createBox(name, scaleX, scaleY, scaleZ)
                
                elif item["uwdsCreate"] == "sphere":
                    #radius = ((item["scaling"]["x"])*10)/2
                    radius = (item["size"]["diameter"])/2
                    
                    #Currently an issue with the sphere mesh that has been logged on underworlds.
                    #uwID = self.__createSphere(name, radius)
                    uwID = self.__createBox(name, (radius*2), (radius*2), (radius*2))
                
                self.uwIDs.update({item["id"]:uwID})
                self.l2torIDs.update({uwID:item["id"]})
                
                self.__transObj(uwID, posX, posY, posZ, rotX, rotY, rotZ, 1, 1, 1)

        self.uwds_logger.info("Underworlds L2TOR client: Scene Loaded")
    
    def __compute_rel_constr(self):
        with underworlds.Context("l2tor_interface") as ctx:
            world = ctx.worlds["l2tor"]
            scene = world.scene

            bbForObjs = {}
            combObj = {}

            self.uwds_logger.info("Underworlds L2TOR client - compute_rel_constr: Getting bounding boxes")
        
            partList = {}
        
            for node in scene.nodes:
                if node.type == MESH:
                    id = self.l2torIDs[node.id]
                
                    msg = "Underworlds L2TOR client - compute_rel_constr: Sorting %s" % (id)
                    self.uwds_logger.info(msg)
                
                    splitID = id.split("?")
                    if len(splitID) == 1:
                        bb_min, bb_max = get_bounding_box_for_node(scene, node)
                        msg = "Underworlds L2TOR client - compute_rel_constr: BB of %s: %d %d %d - %d %d %d" % (id, bb_min[0], bb_min[1], bb_min[2], bb_max[0], bb_max[1], bb_max[2])
                        self.uwds_logger.info(msg)
                        bb = bb_min, bb_max
                        bbForObjs.update({id:bb})
                    else:
                        msg = "Underworlds L2TOR client - compute_rel_constr: Compound Object %s %s" % (splitID[0], splitID[1])
                        self.uwds_logger.info(msg)
                    
                        if splitID[0] in combObj:
                            combObj[splitID[0]][splitID[1]] = node
                        else:
                            combObj[splitID[0]] = {}
                            combObj[splitID[0]][splitID[1]] = node

            self.uwds_logger.info("Underworlds L2TOR client - compute_rel_constr: Constructing bounding boxes from compound objects.")
        
            for key in combObj.keys():
                bb_min = [1e10, 1e10, 1e10] # x,y,z
                bb_max = [-1e10, -1e10, -1e10] # x,y,z
            
                for subKey in combObj[key].keys():
                    node = combObj[key][subKey]
                    bb_node_min, bb_node_max = get_bounding_box_for_node(scene, node)
                
                    bb_min[0] = min(bb_min[0], bb_node_min[0])
                    bb_min[1] = min(bb_min[1], bb_node_min[1])
                    bb_min[2] = min(bb_min[2], bb_node_min[2])
                    bb_max[0] = max(bb_max[0], bb_node_max[0])
                    bb_max[1] = max(bb_max[1], bb_node_max[1])
                    bb_max[2] = max(bb_max[2], bb_node_max[2])
            
                bb = bb_min, bb_max
                bbForObjs.update({key:bb})

            self.uwds_logger.info("Underworlds L2TOR client - compute_rel_constr: Constructing bounding boxes from compound objects.")
        
            rel = {}
            for key in bbForObjs.keys():
                rel_list = []
                for key2 in bbForObjs.keys():
            
                    msg = ""
            
                    if key == key2:
                        continue
                
                    elif isin(bbForObjs[key], bbForObjs[key2]):
                        msg = "Underworlds L2TOR client - compute_rel_constr: %s in %s" % (key, key2)
                        self.uwds_logger.info(msg)
                        rel_list.append({"relation":"in","obj_2":key2})
                        continue
                    
                    elif isontop(bbForObjs[key], bbForObjs[key2]):
                        msg = "Underworlds L2TOR client - compute_rel_constr: %s onTop %s" % (key, key2)
                        self.uwds_logger.info(msg)
                        rel_list.append({"relation":"onTop","obj_2":key2})
                        continue
                
                    elif isabove(bbForObjs[key], bbForObjs[key2]):
                        msg = "Underworlds L2TOR client - compute_rel_constr: %s above %s" % (key, key2)
                        self.uwds_logger.info(msg)
                        rel_list.append({"relation":"above","obj_2":key2})
                        continue
                    
                    elif iswklycont(bbForObjs[key], bbForObjs[key2]):
                        msg = "Underworlds L2TOR client - compute_rel_constr: %s weakly contained by %s" % (key, key2)
                        self.uwds_logger.info(msg)
                        rel_list.append({"relation":"weaklyCont","obj_2":key2})
                        continue
                    
                    elif isclose(bbForObjs[key], bbForObjs[key2]):
                        msg = "Underworlds L2TOR client - compute_rel_constr: %s close %s" % (key, key2)
                        self.uwds_logger.info(msg)
                        rel_list.append({"relation":"next_to","obj_2":key2})
                        #These are currently based on absolute values as the camera is only in one position
                        if istonorth(bbForObjs[key], bbForObjs[key2]):
                            rel_list.append({"relation":"behind","obj_2":key2})
                            msg = "Underworlds L2TOR client - compute_rel_constr: %s behind %s" % (key, key2)
                            self.uwds_logger.info(msg)
                        elif istoeast(bbForObjs[key], bbForObjs[key2]):
                            rel_list.append({"relation":"to_right","obj_2":key2})
                            msg = "Underworlds L2TOR client - compute_rel_constr: %s to right of %s" % (key, key2)
                            self.uwds_logger.info(msg)
                        elif istosouth(bbForObjs[key], bbForObjs[key2]):
                            rel_list.append({"relation":"in_front","obj_2":key2})
                            msg = "Underworlds L2TOR client - compute_rel_constr: %s in front of %s" % (key, key2)
                            self.uwds_logger.info(msg)
                        elif istowest(bbForObjs[key], bbForObjs[key2]):
                            rel_list.append({"relation":"to_left","obj_2":key2})
                            msg = "Underworlds L2TOR client - compute_rel_constr: %s to left of %s" % (key, key2)
                            self.uwds_logger.info(msg)
                        
                        continue
                
                rel.update({key:rel_list})
            
            return rel
            
    def __copyNode(self, cpyID):
        with underworlds.Context("l2tor_interface") as ctx:
            world = ctx.worlds["l2tor"]
            scene = world.scene
        
            nodeToCpy = scene.nodes[cpyID]
            newNode = nodeToCpy.copy()
        
            newNode.parent = scene.rootnode.id
            scene.nodes.append(newNode)
            scene.nodes.update(newNode)
        
            return newNode.id
            
    def __load(self, objName):
        #Get path for l2tor project and put together path for blender file. 
        l2torDir = path.abspath(path.join(__file__, "..", "..", ".."))
        filename = path.join("%s" % (l2torDir), "DataModel", "3dModels", "%s.blend" %  (objName))

        #loadLogger.setLevel(logging.INFO)
        #logging.basicConfig(level=logging.INFO)

        #Load model
        nodes = ModelLoader().load(filename, world="l2tor")
        ids = [node.id for node in nodes if node.name == objName]
        if ids:
            myID = ids[0]
            msg = "Underworlds L2TOR client - load: ID = %s" % (myID)
            self.uwds_logger.info(msg)
            return myID
        self.uwds_logger.info("Underworlds L2TOR client - load: Could not get ID")
            
    def __transObj(self, id, x, y, z, xrot, yrot, zrot, xScale, yScale, zScale):
        with underworlds.Context("l2tor_interface") as ctx:
            world = ctx.worlds["l2tor"]
            scene = world.scene
            node = scene.nodes[id]
        
            #Convert from degrees to radians and negative
            xrot = xrot * (-0.01745329252)
            yrot = yrot * (-0.01745329252)
            zrot = zrot * (-0.01745329252)
        
            node.transformation = compose_matrix([xScale, yScale, zScale], None, [xrot, yrot, zrot], [x, y, z], None)
        
            scene.nodes.update(node)
            
    def __createBox(self, name, xScale, yScale, zScale):
        with underworlds.Context("l2tor_interface") as ctx:
            world = ctx.worlds["l2tor"]
            scene = world.scene
        
            myBox = Box.create(xScale, yScale, zScale)
            ctx.push_mesh(myBox)
            meshID = [myBox.id]
        
            boxNode = Node()
            boxNode.name = name
            boxNode.type = MESH
            boxNode.parent = scene.rootnode.id
        
            boxNode.hires += meshID
            boxNode.cad += meshID
            boxNode.aabb = myBox.aabb
            scene.nodes.append(boxNode)
            scene.nodes.update(boxNode)
        
            return boxNode.id
        
    def __createSphere(name, radius):
        with underworlds.Context("l2tor_interface") as ctx:
            world = ctx.worlds["l2tor"]
            scene = world.scene
        
            mySphere = Sphere.create(radius)
            ctx.push_mesh(mySphere)
            meshID = [mySphere.id]
        
            sphereNode = Node()
            sphereNode.name = name
            sphereNode.type = MESH
            sphereNode.parent = scene.rootnode.id
        
            sphereNode.hires += meshID
            sphereNode.cad += meshID
            scene.nodes.append(sphereNode)
            scene.nodes.update(sphereNode)
        
            return mySphere.id
            
if __name__ == "__main__":
    pass
