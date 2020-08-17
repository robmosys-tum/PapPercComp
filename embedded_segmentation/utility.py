# ------------------------------------------------------------------------------
# Utility functions that are used elsewhere but for readabilty reasons have been
# moved here.
# ------------------------------------------------------------------------------

import numpy as np
import os
import torch

from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from parse import get_arguments


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


def calc_IoU(seg_mask, true_mask):
    """
    Calculates the Intersection over Union between a computed segmentation mask and the given ground truth.

    For multiple segmentation instances, simply the union over all instances is taken.

    Arguments:
        seg_mask (torch.FloatTensor) : mask with integer values denoting which instance a pixel belongs to. Greater than zero if it belongs to an object.
        true_mask (torch.FloatTensor) : binary mask where 1 denotes a pixel that belongs to an object.

    Return:
        torch.Float measuring the IoU between the two images, or for batches (N, C, H, W) we return N values of IoU.
    """
    eps = 1e-6      # Prevent divide by zeroes

    # Convert seg_mask to binary mask with only 1 or 0 values.
    binary_seg = torch.where(seg_mask > 0, torch.ones_like(seg_mask), torch.zeros_like(seg_mask))

    # Inclusion-exclusion principle
    intersection = binary_seg * true_mask
    union = binary_seg + true_mask - intersection
    

    if len(seg_mask.shape) == 4:
        # Input can be a batch of data with shape (N, C, H, W)
        IoU = intersection.sum(dim=[1,2,3]) / (union.sum(dim=[1,2,3]) + eps)
    else:
        # Or input could be just a single image (C, H, W)
        IoU = intersection.sum() / (union.sum() + eps)

    return IoU


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
    args = get_arguments()
    video = args.videos[0]
    
    create_video(
        image_dir="Data/DAVIS/JPEGImages/480p/"+video,
        seg_dir="Output",
        true_dir="Data/DAVIS/Annotations/480p/"+video
    )

