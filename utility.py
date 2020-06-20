# ------------------------------------------------------------------------------
# Utility functions that are used elsewhere but for readabilty reasons have been
# moved here.
# ------------------------------------------------------------------------------

import numpy as np
import os
import imageio


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


def create_video(output_name="video", image_list=None, image_dir=None):
    """
    Creates a video from a sequence of images. Either provide the list of images directly, or let them be loaded in from a directory image_dir.

    Arguments:
        image_list (List) : images to be processed into video
        image_dir (String) : path to folder containing images
    
    Returns:
        Creates a video in the "Output/" folder
    """
    assert (image_list is not None) or (image_dir is not None), "Must provide some input to create_video()."


    if image_list is None:
        image_list = []
        for filename in sorted(os.listdir(image_dir)):
            image_list.append(imageio.imread(os.path.join(image_dir, filename)))
    
    if not os.path.exists("Output"):
        os.makedirs("Output")

    imageio.mimsave(f'Output/{output_name}.gif', image_list)


if __name__ == "__main__":
    create_video(image_dir="Output")
    create_video(output_name="annotated", image_dir= r"D:\Documents\MasterRCI\SS20\RobotPerceptionLab\PapPercComp\Data\DAVIS\Annotations\480p\libby")
