# ------------------------------------------------------------------------------
# Utility functions that are used elsewhere but for readabilty reasons have been
# moved here.
# ------------------------------------------------------------------------------

import numpy as np
import os



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




