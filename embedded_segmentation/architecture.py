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



class EmbeddingHead(nn.Module):
    """
    The embedding network that takes an image (shape [N, 3, H, W]) that has been put through DeepLabv3 excluding the last conv-layer (i.e. having 256 feature channels). This output is then embedded into a d-dimensional embedding space namely [N, d, H, W], where we assume it's a pixel-wise embedding, but with spatial local information.
    """
    def __init__(self, embed_dim):
        super(EmbeddingHead, self).__init__()
        
        self.embed = nn.Sequential(
            ConvBlock(256, 128, kernel_size=5, padding=2, stride=1),
            nn.Conv2d(128, 64, kernel_size=3, padding=1, stride=1)
        )

    
    def forward(self, image):
        output = self.embed(image)
        return output


    @staticmethod
    def initialize_weight(module):
        if isinstance(module, nn.Conv2d):
            nn.init.normal_(module.weight, 0.0, 0.02)
        elif isinstance(module, nn.BatchNorm2d):
            # nn.init.constant_(module.weight, 1)
            nn.init.normal_(module.weight, 1.0, 0.02)
            nn.init.constant_(module.bias, 0)




# Output of deeplabModel are [21, W, H] images for 21 possible segmentations.
deeplabModel = models.segmentation.deeplabv3_resnet101(pretrained=True, progress=True)
# Remove last convolution layer such that output is now [256, W, H]
deeplabModel.classifier[4] = nn.Identity()
deeplabModel.eval()

