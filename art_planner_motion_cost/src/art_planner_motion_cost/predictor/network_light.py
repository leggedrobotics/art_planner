import time
import math
import torch
import torch.nn as nn
import torch.nn.functional as F


# n48convNetwork3LR
class network(nn.Module):

    def __init__(self):
        super(network, self).__init__()

        # Network parameters for height map processing.
        self._featureResDownsampleFactor = 2  # Feature map is downsampled this much compared to input.
        self._mapClip = 24  # Number of cells cut off from the input edges.

        # Init stage
        self.init_conv1 = nn.Conv2d(1, 24, (3, 3), stride=1, dilation=1, bias=False)
        self.init_conv1_bn = nn.BatchNorm2d(24)

        self.init_conv2 = nn.Conv2d(24, 24, (3, 3), stride=1, dilation=1, bias=False)
        self.init_conv2_bn = nn.BatchNorm2d(24)

        self.init_conv3 = nn.Conv2d(24, 48, (3, 3), stride=1, dilation=1, bias=False)
        self.init_conv3_bn = nn.BatchNorm2d(48)

        self.init_conv4 = nn.Conv2d(48, 48, (3, 3), stride=1, dilation=1, bias=False)
        self.init_conv4_bn = nn.BatchNorm2d(48) 

        self.init_conv5 = nn.Conv2d(48, 48, (3, 3), stride=1, dilation=1, bias=False)
        self.init_conv5_bn = nn.BatchNorm2d(48)     

        # Flatten
        self.init_flatten = nn.Conv2d(48, 48, (15, 15), stride=1, dilation=1, bias=False)
        self.init_flatten_bn = nn.BatchNorm2d(48)

        # Dropout
        self.init_dropout = nn.Dropout(p=0.5)

        # Input targets
        self.tar0_conv1 = nn.Conv2d(10, 16, (1, 1), stride=1, dilation=1, bias=False)
        self.tar0_conv1_bn = nn.BatchNorm2d(16)

        # Output stage
        self.out0_conv1 = nn.Conv2d(64, 48, (1, 1), stride=1, dilation=1, bias=False)
        self.out0_conv1_bn = nn.BatchNorm2d(48)

        self.out1_conv1 = nn.Conv2d(48, 24, (1, 1), stride=1, dilation=1, bias=False)
        self.out1_conv1_bn = nn.BatchNorm2d(24)

        self.out1_conv2 = nn.Conv2d(48, 24, (1, 1), stride=1, dilation=1, bias=False)
        self.out1_conv2_bn = nn.BatchNorm2d(24)

        self.out1_conv3 = nn.Conv2d(48, 36, (1, 1), stride=1, dilation=1, bias=False)
        self.out1_conv3_bn = nn.BatchNorm2d(36)

        self.out2_conv1 = nn.Conv2d(24, 1, (1, 1), stride=1, dilation=1, bias=True)

        self.out2_conv2 = nn.Conv2d(24, 1, (1, 1), stride=1, dilation=1, bias=True)

        self.out2_conv3 = nn.Conv2d(36, 1, (1, 1), stride=1, dilation=1, bias=True)

    @property
    def featureResDownsampleFactor(self):
        return self._featureResDownsampleFactor

    @property
    def mapClip(self):
        return self._mapClip

    def forward(self, patch, target):

        self.CNNpart(patch)
        return self.FCpart(target)         


    def CNNpart(self, patch):
        """
        CNN part to update map features
        """

        # Init stage
        temp = self.init_conv1(patch)
        temp = self.init_conv1_bn(temp)
        temp = self.init_conv2(temp)
        temp = self.init_conv2_bn(temp)
        temp = F.leaky_relu(temp, negative_slope=0.3)
        temp = F.max_pool2d(temp, (2, 2), stride=2)

        temp = self.init_conv3(temp)
        temp = self.init_conv3_bn(temp)
        temp = F.leaky_relu(temp, negative_slope=0.3)
        temp = self.init_conv4(temp)
        temp = self.init_conv4_bn(temp)
        temp = F.leaky_relu(temp, negative_slope=0.3)
        temp = F.max_pool2d(temp, (3, 3), stride=1)

        temp = self.init_conv5(temp)
        temp = self.init_conv5_bn(temp)
        temp = F.leaky_relu(temp, negative_slope=0.3)

        # Flatten and dropout
        temp = self.init_flatten(temp)
        temp = self.init_flatten_bn(temp)
        temp = F.leaky_relu(temp, negative_slope=0.3)

        temp = self.init_dropout(temp)
        
        return temp


    def FCpart(self, features, target):
        """
        FC part to querry costs
        """

        tempAngle = target[:, 2, :, :]
        tempAngle = torch.where(tempAngle > math.pi, tempAngle-2*math.pi, tempAngle)
        tempAngle = torch.where(tempAngle < -math.pi, tempAngle+2*math.pi, tempAngle)
        target[:, 2, :, :] = tempAngle

        tarInfo = torch.stack((target[:, 0, :, :], 
                               target[:, 1, :, :], 
                               torch.sqrt(torch.pow(target[:, 0, :, :], 2)+torch.pow(target[:, 1, :, :], 2)),
                               torch.atan2(target[:, 1, :, :], target[:, 0, :, :]),
                               target[:, 2, :, :],
                               torch.cos(target[:, 2, :, :]),
                               torch.sin(target[:, 2, :, :]), 
                               target[:, 3, :, :], 
                               torch.cos(target[:, 3, :, :]), 
                               torch.sin(target[:, 3, :, :])), dim=1)

        # Input targets
        tarInfo = self.tar0_conv1(tarInfo)
        tarInfo = self.tar0_conv1_bn(tarInfo)
        temp = torch.cat((features, tarInfo), dim=1)

        # Output stage
        temp = self.out0_conv1(temp)
        temp = self.out0_conv1_bn(temp)
        temp = F.leaky_relu(temp, negative_slope=0.3)

        power = self.out1_conv1(temp)
        power = self.out1_conv1_bn(power)
        power = F.leaky_relu(power, negative_slope=0.3)
        power = self.out2_conv1(power)
        power = F.relu(power)

        time = self.out1_conv2(temp)
        time = self.out1_conv2_bn(time)
        time = F.leaky_relu(time, negative_slope=0.3)
        time = self.out2_conv2(time)
        time = F.relu(time)

        prob = self.out1_conv3(temp)
        prob = self.out1_conv3_bn(prob)
        prob = F.leaky_relu(prob, negative_slope=0.3)
        prob = self.out2_conv3(prob)
        prob = torch.sigmoid(prob)

        risk = torch.ones(prob.size(), dtype=torch.float16, device='cuda')
        risk.sub_(torch.min(prob))

        return power, time, risk, 1.0-prob


    def getRisk(self, features, target):
        """
        FC part to querry risks only
        """
        tempAngle = target[:, 2, :, :]
        tempAngle = torch.where(tempAngle > math.pi, tempAngle-2*math.pi, tempAngle)
        tempAngle = torch.where(tempAngle < -math.pi, tempAngle+2*math.pi, tempAngle)
        target[:, 2, :, :] = tempAngle

        tarInfo = torch.stack((target[:, 0, :, :], 
                               target[:, 1, :, :], 
                               torch.sqrt(torch.pow(target[:, 0, :, :], 2)+torch.pow(target[:, 1, :, :], 2)),
                               torch.atan2(target[:, 1, :, :], target[:, 0, :, :]),
                               target[:, 2, :, :],
                               torch.cos(target[:, 2, :, :]),
                               torch.sin(target[:, 2, :, :]), 
                               target[:, 3, :, :], 
                               torch.cos(target[:, 3, :, :]), 
                               torch.sin(target[:, 3, :, :])), dim=1)

        # Input targets
        tarInfo = self.tar0_conv1(tarInfo)
        tarInfo = self.tar0_conv1_bn(tarInfo)
        temp = torch.cat((features, tarInfo), dim=1)

        # Output stage
        temp = self.out0_conv1(temp)
        temp = self.out0_conv1_bn(temp)
        temp = F.leaky_relu(temp, negative_slope=0.3)

        prob = self.out1_conv3(temp)
        prob = self.out1_conv3_bn(prob)
        prob = F.leaky_relu(prob, negative_slope=0.3)
        prob = self.out2_conv3(prob)
        prob = torch.sigmoid(prob)

        return 1.0-prob













