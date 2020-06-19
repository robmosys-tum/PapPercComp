# ------------------------------------------------------------------------------
# PyTorch implementation of the architecture. 
# ------------------------------------------------------------------------------


import torch
import torch.nn as nn
import torch.functional as F
import numpy as np

from torchvision import models


class ConvBlock(nn.Sequential):
    """
    2D Convolution followed by Batch Normalization and finished with a ReLU. 
    """
    def __init__(self, in_feat, out_feat, kernel_size, padding, stride):
        super(ConvBlock, self).__init__()

        self.add_module('Conv2D', nn.Conv2d(in_feat, out_feat, kernel_size=kernel_size, padding=padding, stride=stride))
        self.add_module('BatchNorm', nn.BatchNorm2d(out_feat))

        # Might want to change ReLU to LeakyReLU when LSTM is used.
        self.add_module('ReLU', nn.ReLU())



class DeepLab(nn.Module):
    """
    The embedding head which is a pre-trained DeepLabv3 network with the final layer cut off.
    """
    def __init__(self):
        super(DeepLab, self).__init__()

        # Get DeepLab layers

    
    def forward(self, x):
        return x

# Output of deeplabModel are (21, w, h) images for 21 possible segmentations.
deeplabModel = models.segmentation.deeplabv3_resnet101(pretrained=True, progress=True)

