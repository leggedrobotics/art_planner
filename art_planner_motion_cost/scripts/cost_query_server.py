#!/usr/bin/env python3
import os
import numpy as np
import cv2 as cv

import rospy

import math
import threading
from grid_map_msgs.msg import GridMap

from art_planner_motion_cost.srv import costQuery, costQueryResponse

from art_planner_motion_cost.predictor import CostPredictor, CostQuery
import art_planner_motion_cost.transformations as transformations



class GPUCostQueryServer(object):
    def __init__(self):
        try:
            cfg = rospy.get_param(rospy.get_name())
        except KeyError:
            rospy.logfatal('Did not find motion cost network parameters: ' +rospy.get_name())


        self.cost_predictor = CostPredictor(cfg)
        self.cost_query = CostQuery(self.cost_predictor, cfg)
        self.elvMap = None
        self.elvMap_holes = None

        # Elevation map
        self.lock = threading.Lock()
        self.elvMap_layer = cfg['map_layer']
        self.mapFrameId = None
        self.patchTrans = np.array([0.0, 0.0, 1.0, 0.0])    # (center(x), center(y), cos(theta), sin(theta))

        # ROS connections
        node_name = rospy.get_name()
        self.elvMapSub = rospy.Subscriber(node_name + '/map', GridMap, self.elevationMapCallback)
        self.costQueryService = rospy.Service(node_name + '/cost_query', costQuery, self.handle_cost_query)
        self.costQueryNoUpdateService = rospy.Service(node_name + '/cost_query_no_update', costQuery, self.handle_cost_query_no_update)

        rospy.loginfo("GPU cost query server initialized")

    def elevationMapCallback(self, gMap):
        """
        Receive elevation map from elevation_mapping_cupy
        """
        rospy.logdebug("Elevation map received")

        # Receive elevation map
        self.mapFrameId = gMap.info.header.frame_id
        qX = gMap.info.pose.orientation.x
        qY = gMap.info.pose.orientation.y
        qZ = gMap.info.pose.orientation.z
        qW = gMap.info.pose.orientation.w
        euler = transformations.euler_from_quaternion((qX, qY, qZ, qW))
        layerIdx = gMap.layers.index(self.elvMap_layer)

        self.patchTrans[0] = gMap.info.pose.position.x
        self.patchTrans[1] = gMap.info.pose.position.y
        self.patchTrans[2] = math.cos(euler[2])
        self.patchTrans[3] = math.sin(euler[2])

        reslu = gMap.info.resolution
        offset = gMap.data[layerIdx].layout.data_offset
        col = gMap.data[layerIdx].layout.dim[0].size
        row = gMap.data[layerIdx].layout.dim[1].size

        # Set map params in cost query object.
        self.cost_query.setMapParams(reslu, gMap.info.length_x, gMap.info.length_y)

        self.elvMap_holes = np.rot90((np.asarray(gMap.data[layerIdx].data)+offset).astype(np.float32).reshape((row, col)), 2).transpose()

    def _elvMapProcess(self):
        """
        Preprocess the elevation map
        """
        if self.elvMap_holes is None:
            rospy.loginfo("WRN: No elevation map found")
            return False
        
        # Copy data from high freq buffers
        self.lock.acquire()

        elvMap_holes = np.copy(self.elvMap_holes)
        self.currentTrans = np.copy(self.patchTrans)

        self.lock.release()

        # Mask invalid values and interpolate
        if np.isnan(elvMap_holes).any() or np.isinf(elvMap_holes).any():
            rospy.loginfo("WRN: Invalid mapping value. Interpolating.")
            # Old.  ~0.1s inference
            # x = np.arange(0, elvMap_holes.shape[1])
            # y = np.arange(0, elvMap_holes.shape[0])
            # ma_elvMap_holes = np.ma.masked_invalid(elvMap_holes)
            # xx, yy = np.meshgrid(x, y)
            # x1 = xx[~ma_elvMap_holes.mask]
            # y1 = yy[~ma_elvMap_holes.mask]
            # elvMap = ma_elvMap_holes[~ma_elvMap_holes.mask]
            # elvMap_interpolated = interpolate.griddata((x1, y1), elvMap.ravel(), (xx, yy), method='nearest')
            # self.elvMap = elvMap_interpolated

            # New.  ~ 0.02s inference
            mask = (~np.isfinite(elvMap_holes)).astype(np.uint8)
            min = np.nanmin(elvMap_holes)
            max = np.nanmax(elvMap_holes)
            elvMap_holes = (elvMap_holes - min) *255 / (max-min)
            self.elvMap = cv.inpaint(elvMap_holes.astype(np.uint8), mask, 3, cv.INPAINT_TELEA)
            self.elvMap = self.elvMap.astype(np.float32) * (max-min) / 255 + min
        else:
            self.elvMap = elvMap_holes 

        # Update features
        self.cost_predictor.updateFeatures(self.elvMap)

        return True

    def handle_cost_query_no_update(self, request):
        rospy.logdebug("Cost query no update handle")

        req_len = len(request.query_poses)
        rospy.loginfo('Got query of length ' + str(req_len//6))

        assert request.header.frame_id == self.mapFrameId, 'Request frame different from map frame: ' + request.header.frame_id
        assert req_len % 6 == 0, 'Request array length not multiple of 6'


        if self.elvMap is not None:
            target_info = np.array(request.query_poses).reshape(-1, 6)
            # Second dim should be [target_x, target_y, target_yaw, start_x, start_y, start_yaw]
            target_info[:,0:2] -= self.currentTrans[:2]
            target_info[:,3:5] -= self.currentTrans[:2]

            # Do cost query.
            costs = self.cost_query(target_info)


            return costQueryResponse(costs[0], costs[1], costs[2])

        return None

    def handle_cost_query(self, request):
        rospy.logdebug("Cost query handle")

        req_len = len(request.query_poses)
        rospy.loginfo('Got query of length ' + str(req_len//6))

        assert request.header.frame_id == self.mapFrameId, 'Request frame different from map frame: ' + request.header.frame_id
        assert req_len % 6 == 0, 'Request array length not multiple of 6'


        # Preprocess elevation map
        map_received = self._elvMapProcess()
        if map_received:
            target_info = np.array(request.query_poses).reshape(-1, 6)
            # Second dim should be [target_x, target_y, target_yaw, start_x, start_y, start_yaw]
            target_info[:,0:2] -= self.currentTrans[:2]
            target_info[:,3:5] -= self.currentTrans[:2]

            # Do cost query.
            costs = self.cost_query(target_info)


            return costQueryResponse(costs[0], costs[1], costs[2])

        return None



if __name__ == "__main__":
    rospy.init_node('art_planner_motion_cost_server', anonymous=False) # log_level=rospy.DEBUG
    server = GPUCostQueryServer()
    rospy.spin()


