# ------------------------------------------------------------------------------
# Loads training or testing data for PyTorch network.
# Currently provides train_set and val_set for DAVIS 2016 dataset, and allows 
# custom datasets with one annoted frame for inference.
# ------------------------------------------------------------------------------

import torch
import os

from PIL import Image
from torchvision import transforms


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
                    #     print("Currently loaded: %d" % len(self.imagePaths))


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

        # DeepLab requires normalized input
        preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # The above paths include a '/' at the start, so start from second character
        im = Image.open(os.path.join(self.davisDir, imPath[1:]))
        X = preprocess(im)                  # Shape [3, 480, 854]

        # You do not want to preprocess the segmentation masks
        seg = Image.open(os.path.join(self.davisDir, segPath[1:]))
        Y = transforms.ToTensor()(seg)      # Shape [1, 480, 854]

        return (X, Y)



class CustomData(torch.utils.data.Dataset):
    """
    Create a map-style dataset for Video Object Segmentation VALIDATION data from all images in a directory, with one single annotated segmentation mask called "AnnotatedFrame.png".

    All data is loaded into memory at once. This is meant for small datasets for validation purposes.

    Arguments:
        dir (String) : path to folder containing images.
    """
    def __init__(self, directory):
        self.images = []
        self.seg_mask = []

        # Go over files in alphabetical order (according to video sequence)
        for f in sorted(os.listdir(directory)):
            im = Image.open(os.path.join(directory, f))

            if f == "AnnotatedFrame.png":
                self.seg_mask = transforms.ToTensor()(im)
            else:
                self.images.append(im)
            
            # if len(self.imagePaths) % 100 == 0:
            #     print("Currently loaded: %d" % len(self.imagePaths))


    def __len__(self):
        return len(self.images)
        

    def __getitem__(self, idx):
        im = self.images[idx]

        ### DeepLab requires normalized input
        preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        X = preprocess(im)          # Shape [3, 480, 854]

        return (X, self.seg_mask)



train_set = DAVISData('train')
val_set = DAVISData('val')

