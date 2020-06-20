# ------------------------------------------------------------------------------
# Utility functions that are used elsewhere but for readabilty reasons have been
# moved here.
# ------------------------------------------------------------------------------

import numpy as np
import os

from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def load_model(model):
    """
    Load the model parameters from an existing trained model. 
    """
    fin = False
    backup1 = False
    backup2 = False

    if os.path.exists("TrainedModel/finalModel.pth"):
        fin = True
    elif os.path.exists("TrainedModel/modelBackup.pth"):
        backup1 = True
    elif os.path.exists("TrainedModel/modelBackupBackup.pth"):
        backup2 = True

    if fin:
        try:
            model.load_state_dict(torch.load("TrainedModel/finalModel.pth"))
            return model
        except:
            print("finalModel seems to be corrupted, trying a backup...")
    
    if fin or backup1:
        try:
            model.load_state_dict(torch.load("TrainedModel/modelBackup.pth"))
            return model
        except:
            print("modelBackup seems to be corrupted, trying a backup...")

    if fin or backup1 or backup2:
        try:
            model.load_state_dict(torch.load("TrainedModel/modelBackupBackup.pth"))
            return model
        except:
            print("modelBackupBackup seems to be corrupted, you're at the end of the line.")

    print("There doesn't seem to be anything to load.")
    return model


def create_video(output_name="video", image_dir=None, seg_dir=None, true_dir=None):
    """
    Creates a video from a sequence of images. Provide the directories containing the images. We require three image types: the real images from image_dir, the computed segmentation masks in seg_dir and the ground-truth data in true_dir.

    Arguments:
        image_dir (String) : path to folder containing real images
        seg_dir (String) : path to folder containing computed segmentation masks
        true_dir (String) : path to folder containing ground-truth segmentation masks
    
    Returns:
        Creates a video in the "Output/" folder
    """

    image_list = []
    for filename in sorted(os.listdir(image_dir)):
        image_list.append(Image.open(os.path.join(image_dir, filename)))

    seg_list = []
    for filename in sorted(os.listdir(seg_dir)):
        if filename[-3:] == "png":
            seg_list.append(Image.open(os.path.join(seg_dir, filename)))


    true_list = []
    for filename in sorted(os.listdir(true_dir)):
        true_list.append(Image.open(os.path.join(true_dir, filename)))

    
    if not os.path.exists("Output"):
        os.makedirs("Output")

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(14, 4))
    ax1.axis('off')
    ax2.axis('off')
    ax3.axis('off')
    fig.tight_layout()

    ims = []
    for i, s, t in zip(image_list, seg_list, true_list):
        im1 = ax1.imshow(i, animated=True)
        im2 = ax2.imshow(s, animated=True)
        im3 = ax3.imshow(t, animated=True)

        ims.append([im1, im2, im3])

    ani = animation.ArtistAnimation(fig, ims, interval=50, blit=True)
    ani.save(f"Output/{output_name}.mp4")
    


if __name__ == "__main__":
    create_video(
        image_dir=r"D:\Documents\MasterRCI\SS20\RobotPerceptionLab\PapPercComp\Data\DAVIS\JPEGImages\480p\soapbox",
        seg_dir="SoapboxSeg",
        true_dir=r"D:\Documents\MasterRCI\SS20\RobotPerceptionLab\PapPercComp\Data\DAVIS\Annotations\480p\soapbox")

