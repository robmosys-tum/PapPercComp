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
        video (String) : defaults to None, loading all video frames from all video in DAVIS2016 dataset. When given a value, it should refer to the video of which the frames will be loaded (e.g. "bear" or "stroller").
    """
    def __init__(self, mode, video=None):

        ### Prefix-free issue only when using trainval as set. Otherwise train and val separately are prefix-free.
        #assert not video in ['breakdance','dog', 'surf'], "CURRENTLY 'breakdance', 'dog', 'surf' IS NOT SUPPORTED! Dataset names are generally prefix-free, except for these. 'surf'  in train vs 'kite-surf' in val."

        self.davisDir = "Data/DAVIS/"

        self.imagePaths = []
        self.segmentationPaths = []

        # Efficiency: after video is loaded, we can exit the for-loop.
        vid_done = False

        if mode == 'train':
            dataPath = os.path.join(self.davisDir, "ImageSets/480p/train.txt")

            with open(dataPath, 'r') as dataFiles:
                for line in dataFiles.read().splitlines():
                    if video is None or video in line:
                        imPath, segPath, _ = line.split(' ')
                        
                        self.imagePaths.append(imPath)
                        self.segmentationPaths.append(segPath)

                        vid_done = True

                    elif vid_done:
                        break

                    
                    # if len(self.imagePaths) % 100 == 0:
                    #     print("Currently loaded: %d" % len(self.imagePaths))


        elif mode == 'val':
            dataPath = os.path.join(self.davisDir, "ImageSets/480p/val.txt")

            with open(dataPath, 'r') as dataFiles:
                for line in dataFiles.read().splitlines():
                    if video is None or video in line:
                        imPath, segPath, _ = line.split(' ')
                        
                        self.imagePaths.append(imPath)
                        self.segmentationPaths.append(segPath)

                        vid_done = True

                    elif vid_done:
                        break


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

        # Some of the DAVIS data seems off, annotation with two channels instead of one (I assume one is opacity). Only 1 case: 00077.png in bear annotation.
        if Y.shape[0] != 1:
            #print(f"Encountered special case for {imPath}.")
            Y = Y[0:1]

        return (X, Y)



class CustomData(torch.utils.data.Dataset):
    """
    Create a map-style dataset for Video Object Segmentation VALIDATION data from all images in 'directory'. Data can have two forms:
        1. One-Shot VOS data: one single annotated segmentation mask called "AnnotatedFrame.png" must be given in 'directory', 'seg_dir' remains None.
        2. Validation data to calculate IoU metric: 'seg_dir' contains all the ground-truth segmentation masks. 

    All data is loaded into memory at once. This is meant for small datasets for validation purposes.

    Arguments:
        directory (String) : path to folder containing images.
        seg_dir (String, optional) : path to folder containing ground-truth segmentation masks.
    """
    def __init__(self, directory, seg_dir=None):
        self.images = []
        self.seg_masks = []

        # Go over files in alphabetical order (according to video sequence)
        for f in sorted(os.listdir(directory)):
            im = Image.open(os.path.join(directory, f))

            if (seg_dir is None) and (f == "AnnotatedFrame.png"):
                self.seg_masks = im
            else:
                if f != "AnnotatedFrame.png":
                    self.images.append(im)
            

        if seg_dir is not None:
            for f in sorted(os.listdir(seg_dir)):
                seg = Image.open(os.path.join(seg_dir, f))
                self.seg_masks.append(seg)
        
            len(self.images) == len(self.seg_masks), "Number of images and segmentation masks don't match!"


    def __len__(self):
        return len(self.images)
        

    def __getitem__(self, idx):
        im = self.images[idx]

        if isinstance(self.seg_masks, list):
            seg = self.seg_masks[idx]
        else:
            seg = self.seg_masks

        # DeepLab requires normalized input
        preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        X = preprocess(im)                  # Shape [3, 480, 854]
        Y = transforms.ToTensor()(seg)      # Shape [1, 480, 854]

        return (X, Y)


