# ------------------------------------------------------------------------------
# Loads training or testing data for PyTorch network.
# Currently provides train_set and val_set for DAVIS 2016 dataset.
# ------------------------------------------------------------------------------

import torch
import os

from PIL import Image
from torchvision.transforms import ToTensor


class DAVISData(torch.utils.data.Dataset):
    """
    Create a map-style dataset for Video Object Segmentation data from the DAVIS dataset is in "./Data/DAVIS". 
    The data consists of the images and their segmentation masks. Always loads 480p images. 

    As there is too much training data to load into memory all at once, it loads images only when they are used.


    Arguments:
        mode (String) : 'train' or 'val' dataset
    """
    def __init__(self, mode):

        self.davisDir = "Data/DAVIS/"

        self.imagePaths = []
        self.segmentationPaths = []


        if mode == 'train':
            dataPath = os.path.join(self.davisDir, "ImageSets/480p/train.txt")

            with open(dataPath, 'r') as dataFiles:
                for line in dataFiles.read().splitlines():
                    imPath, segPath, _ = line.split(' ')
                    
                    self.imagePaths.append(imPath)
                    self.segmentationPaths.append(segPath)

                    
                    # if len(self.imagePaths) % 100 == 0:
                    #     print("Length currently: %d" % len(self.imagePaths))


        elif mode == 'val':
            dataPath = os.path.join(self.davisDir, "ImageSets/480p/val.txt")

            with open(dataPath, 'r') as dataFiles:
                for line in dataFiles.read().splitlines():
                    imPath, segPath, _ = line.split(' ')
                    
                    self.imagePaths.append(imPath)
                    self.segmentationPaths.append(segPath)


        else:
            print("ERROR CREATING DAVIS DATASET!")


    def __len__(self):
        return len(self.imagePaths)
        

    def __getitem__(self, idx):
        imPath = self.imagePaths[idx]
        segPath = self.segmentationPaths[idx]

        # The above paths include a '/' at the start, so start from second character
        im = Image.open(os.path.join(self.davisDir, imPath[1:]))
        X = ToTensor()(im)         # Shape [3, 480, 854]

        seg = Image.open(os.path.join(self.davisDir, segPath[1:]))
        Y = ToTensor()(seg)       # Shape [1, 480, 854]

        return (X, Y)



train_set = DAVISData('train')
val_set = DAVISData('val')

