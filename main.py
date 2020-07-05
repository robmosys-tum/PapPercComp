# ------------------------------------------------------------------------------
# This file should be called using command line, the command is then parsed and 
# finds the correct files to run.
# ------------------------------------------------------------------------------

import torch

from parse import get_arguments
from dataloader import DAVISData, CustomData
from training import run_model


if __name__ == "__main__":
    args = get_arguments()

    if args.mode == 'train':
        ### Actual training
        # Shapes of the loaded data are [batch_size, channels, width, height]
        train_DAVIS = DAVISData('train')

        dataloader = torch.utils.data.DataLoader(
            train_DAVIS, 
            batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers, pin_memory=True)

        
    elif args.mode == 'validation':
        ### This is actually the 'test' set, but apparently most papers (including the dataset itself) classify this as the validation set. As I identify as a sheep, I shall follow the crowd.

        if args.custom_data is not None:
            # Using custom data to retrieve IoU scores
            assert args.seg_dir is not None, "Ground truth for segmentation masks must be provided with '--seg_dir <directory>'."
        
            dataloader = torch.utils.data.DataLoader(
                CustomData(args.custom_data, seg_dir=args.seg_dir), 
                batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)

        else:
            # Using DAVIS 2016 validation dataset   
            val_DAVIS = DAVISData('val')

            dataloader = torch.utils.data.DataLoader(
                val_DAVIS, 
                batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)


    elif args.mode == 'inference':
        ### Custom data where only the segmentation mask of the first frame is given. Generate the rest of the segmentation masks.
        assert args.custom_data is not None, "No directory given for custom data. Please use '--custom_data <directory>'."

        dataloader = torch.utils.data.DataLoader(
            CustomData(args.custom_data), 
            batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)


    # No other modes allowed


    run_model(dataloader, args=args)
    

