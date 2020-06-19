# ------------------------------------------------------------------------------
# This file should be called using command line, which is then parsed and finds
# the correct files to run.
# ------------------------------------------------------------------------------

import torch

from parse import get_arguments
from dataloader import train_set, val_set, CustomData
from training import run_model


if __name__ == "__main__":
    args = get_arguments()

    if args.mode == 'train':
        ### Actual training
        # Shapes of the loaded data are [batch_size, channels, width, height]
        dataloader = torch.utils.data.DataLoader(train_set, batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers, pin_memory=True)

        run_model(dataloader, args=args, mode='train')

        
    elif args.mode == 'val':
        ### This is actually the 'test' set, but apparently most papers (including the dataset itself) classify this as the validation set. As I identify as a sheep, I shall follow the crowd.
        dataloader = torch.utils.data.DataLoader(val_set, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)

        run_model(dataloader, args=args, mode='val')


    elif args.mode == 'inference':
        ### Custom data where only the segmentation mask of the first frame is given. Generate the rest of the segmentation masks.
        assert args.custom_data is not None, "No directory given for custom data. Please use '--custom_data <directory>'."

        dataloader = torch.utils.data.DataLoader(CustomData(args.custom_data), batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)

        run_model(dataloader, args=args, mode='inference')


    # No other modes allowed