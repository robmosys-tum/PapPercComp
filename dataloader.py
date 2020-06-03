import torch
import numpy
from trashnet.data.constants import *
import glob
from os.path import join
from PIL import Image



class TrashNetDataset(torch.utils.data.Dataset):
    def __init__(self, basedir: str = ""):
        self.basedir = basedir
        self.glass_list = [(x, GLASS) for x in self.parseImages("glass/*")]
        self.paper_list = [(x, PAPER) for x in self.parseImages("paper/*")]
        self.cardboard_list = [(x, CARDBOARD) for x in self.parseImages("cardboard/*")]
        self.plastic_list = [(x, PLASTIC) for x in self.parseImages("plastic/*")]
        self.metal_list = [(x, METAL) for x in self.parseImages("metal/*")]
        self.trash_list = [(x, TRASH) for x in self.parseImages("trash/*")]

        self.image_list = self.glass_list + self.paper_list + self.cardboard_list + self.plastic_list + self.metal_list + self.trash_list

        self.data_len = len(self.image_list)

    def __len__(self):
        return self.data_len

    def __getitem__(self, index):
        return torch.Tensor(self.image_list[index][0]), self.image_list[index][1]

    def parseImages(self, path: str):
        return [numpy.asarray(Image.open(x))/255.0 for x in glob.glob(join(self.basedir, path))]
