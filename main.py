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
        # If we receive a list of videos that we want to process, we create a list of dataloaders to pass to our model training/validation.
        if args.videos is not None:
            dataloader = []

            for v in args.videos:
                loader = torch.utils.data.DataLoader(
                    DAVISData('train', video=v), 
                    batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers, pin_memory=True)

                dataloader.append(loader)

        else:
            # Shapes of the loaded data are [batch_size, channels, width, height]
            dataloader = torch.utils.data.DataLoader(
                DAVISData('train'), 
                batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers, pin_memory=True)

        
    elif args.mode == 'validation':
        ### This is actually the 'test' set, but apparently most papers (including the dataset itself) classify this as the validation set. As I identify as a sheep, I shall follow the crowd.

        if args.custom_data is not None:
            # Using custom data to retrieve IoU scores
            assert args.seg_dir is not None, "Ground truth for segmentation masks must be provided with '--seg_dir <directory>'."
        
            dataloader = torch.utils.data.DataLoader(
                CustomData(args.custom_data, seg_dir=args.seg_dir), 
                batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)

        # CustomData has priority over loading selected videos from DAVIS2016 for validation
        elif args.videos is not None:
            dataloader = []

            for v in args.videos:
                loader = torch.utils.data.DataLoader(
                    DAVISData('val', video=v), 
                    batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)

                dataloader.append(loader)

        else:
            # AT THE MOMENT NOT SUPPORTED: when validating OSVOS, we want to stay within the same video. Use args.videos instead.
            
            # Using DAVIS 2016 validation dataset   
            dataloader = torch.utils.data.DataLoader(
                DAVISData('val'), 
                batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)


    elif args.mode == 'inference':
        ### Custom data where only the segmentation mask of the first frame is given. Generate the rest of the segmentation masks.
        assert args.custom_data is not None, "No directory given for custom data. Please use '--custom_data <directory>'."

        dataloader = torch.utils.data.DataLoader(
            CustomData(args.custom_data), 
            batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)


    # No other modes allowed


    run_model(dataloader, args=args)
    

