import rospkg
import os
import torch
#from .network import network
from .network_light import network



class CostPredictor(object):

    def __init__(self, cfg):   
        
        pkg_dir = rospkg.RosPack().get_path('art_planner_motion_cost')
        modelFile = os.path.join(pkg_dir, cfg['model_file'])

        # Predictor
        with torch.no_grad():
            self.network = network()
            self.network.cuda() 
            self.network.load_state_dict(torch.load(modelFile, map_location='cuda'))
            self.network.eval()
            self.network.half()

        # Buffers
        self.features = None


    def updateFeatures(self, patch):
        """
        Update features using CNN part
        """
        with torch.no_grad():
            # Update feature map
            t_map = torch.from_numpy(patch).half().cuda().view(1, 1, patch.shape[0], patch.shape[1])
            t_map = t_map.half()
            self.features = self.network.CNNpart(t_map).detach()


    def getPathCost(self, features, tarInfo):
        """
        Wrapper for the network FC part
        """
        with torch.no_grad():
            return self.network.FCpart(features.half(), tarInfo.half())


    def getRisk(self, feature, tarInfo):
        """
        Wrapper for the network getRisk(), which returns risks only
        """
        with torch.no_grad():
            return self.network.getRisk(feature.half(), tarInfo.half())

    
