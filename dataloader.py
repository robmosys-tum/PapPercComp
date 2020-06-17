# ------------------------------------------------------------------------------
# Loads training or testing data for PyTorch network.
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


dataloaders = {}
dataloaders['train'] = torch.utils.data.DataLoader(train_set, batch_size=16, shuffle=True, num_workers=0, pin_memory=True)
dataloaders['val'] = torch.utils.data.DataLoader(val_set, batch_size=16, shuffle=False, num_workers=0, pin_memory=True)
#dataloaders['test'] = torch.utils.data.DataLoader(test_set, batch_size=128, shuffle=False, num_workers=2, pin_memory=True)

# Shapes of the loaded data are [batch_size, channels, width, height]


i=0
for x, y in dataloaders['train']:
    print("Batch numbero %d" % i)
    print(x.shape)
    print(y.shape)
    i += 1

    if i > 10:
        break
