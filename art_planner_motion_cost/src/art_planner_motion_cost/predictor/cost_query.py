import os
import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim



class CostQuery(object):

    def __init__(self, predictor, cfg):   
        
        self.featureResFactor = predictor.network.featureResDownsampleFactor
        self.mapClip = predictor.network.mapClip
        self.featureRes = None
        self.rowBias = None
        self.colBias = None

        # Predictor
        self.predictor = predictor



    def setMapParams(self, res, length_x, length_y):
        """
        Sets the parameters of the input height map.
        res:       Map resolution [m]
        length_x:  Map length in x direction [m]
        length_y:  Map length in y direction [m]
        """
        self.featureRes = res * self.featureResFactor
        self.rowBias = int((length_x/res - 2*self.mapClip)/self.featureResFactor*0.5)
        self.colBias = int((length_y/res - 2*self.mapClip)/self.featureResFactor*0.5)



    def __call__(self, target_info): 
        """
        Return the motion cost for the query points.
        target_info: B x 6 size tensor.
                     Second dim should be
                     [target_x, target_y, target_yaw, start_x, start_y, start_yaw]
        """
        assert self.rowBias is not None, 'Map parameters in cost query never set.'

        with torch.no_grad():
            # Compute delta motion.
            target_info = torch.from_numpy(target_info).cuda()
            target_info[:, :3] = target_info[:, :3] - target_info[:, 3:]

            # Get features for points.
            patchRow = torch.clamp(target_info[:, 3]/self.featureRes + self.rowBias, min=1, max=self.predictor.features.shape[2]-2).long()
            patchCol = torch.clamp(target_info[:, 4]/self.featureRes + self.colBias, min=1, max=self.predictor.features.shape[3]-2).long()
            features = self.predictor.features[:, :, patchRow, patchCol]
            features = features.squeeze(0).t()
            features = features.unsqueeze(-1).unsqueeze(-1)

            # Compute network target input.
            target_info = torch.cat((target_info[:,:3], target_info[:,5:6]), dim=1)
            target_info = target_info.unsqueeze(-1).unsqueeze(-1)

            # Query cost.
            cost = self.predictor.getPathCost(features, target_info)

            return cost[0][:,0,0,0].cpu().numpy(), \
                   cost[1][:,0,0,0].cpu().numpy(), \
                   cost[3][:,0,0,0].cpu().numpy()
