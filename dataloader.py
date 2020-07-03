import torch
from trashnet.data.constants import *
import glob
from os.path import join
from PIL import Image
import torchvision


class AugmentationDescriptor:
    def __init__(self, transform: torchvision.transforms.Compose, increase: float = 1.0):
        self.transform = transform
        self.increase = increase


class TrashNetDataset(torch.utils.data.Dataset):
    def __init__(self, basedir: str = ""):
        self.basedir = basedir
        self.image_list = [(x, GLASS) for x in self.parseImages("glass/*")]
        self.image_list += [(x, PAPER) for x in self.parseImages("paper/*")]
        self.image_list += [(x, CARDBOARD) for x in self.parseImages("cardboard/*")]
        self.image_list += [(x, PLASTIC) for x in self.parseImages("plastic/*")]
        self.image_list += [(x, METAL) for x in self.parseImages("metal/*")]
        self.image_list += [(x, TRASH) for x in self.parseImages("trash/*")]

        self.data_len = len(self.image_list)

    def __len__(self):
        return self.data_len

    def __getitem__(self, index):
        return self.image_list[index]

    def parseImages(self, path: str):
        toTensor = torchvision.transforms.ToTensor()
        return [toTensor(Image.open(x)) for x in glob.glob(join(self.basedir, path))]