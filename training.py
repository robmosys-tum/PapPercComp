# ------------------------------------------------------------------------------
# Training the network defined in architecture using the data loaded using data-
# loader.
# ------------------------------------------------------------------------------

import torch
import os

from architecture import DeepLab
from utility import load_model



def run_model(dataloader, args, mode='train'):
    """
    Run a model in either training mode or validation mode. If the model exists, then the weights are loaded.

    Arguments:
        dataloader (torch.utils.data.DataLoader) : contains the data used for training/testing.
        args (argparse.ArgumentParser) : command line arguments
        mode (String) : train or val

    Returns:
        None
    """
    # Load the device on which we're running the model
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

    ### Create model of embedding network
    # TODO: define model
    #model = DeepLab()
    model = model.to(device)

    if args.load:
        # TODO: Check if I need to assign the value to model, or if load_state_dict works on model directly (not sure if passed by reference).
        load_model(model)

    else:
        # TrainedModel directory might not exist yet
        if not os.path.exists("TrainedModel"):
            os.makedirs("TrainedModel")


    ### Create optimizer and scheduler
    optimizer = torch.optim.SGD(
                        model.parameters(), 
                        lr=0.1, 
                        momentum=0.9, 
                        weight_decay=1e-4
                    )

    lr_scheduler = torch.optim.lr_scheduler.MultiStepLR(
                        optimizer, 
                        milestones=[int(args.epochs/3), int(2*args.epochs/3)], 
                        gamma=0.1
                    )


    ### Run epochs
    for epoch in range(args.epochs):
        epoch_loss, epoch_acc = run_epoch(model, optimizer, dataloader, mode)
        # Update the learning rate based on scheduler
        lr_scheduler.step()

        
        # Print statistics
        print(f"Epoch {epoch + 1: >4}/{args.epochs}, Loss: {epoch_loss:.2e}, Accuracy: {epoch_acc * 100:.2f}%")


        # Make double checkpoints, just in case.
        if epoch % args.checkpoint_epochs == 0:
            torch.save(model.state_dict(), "TrainedModel/modelBackup.pth")
        elif epoch % args.checkpoint_epochs-1 == 0:
            torch.save(model.state_dict(), "TrainedModel/modelBackupBackup.pth")
    
    ### Save final model
    torch.save(model.state_dict(), "TrainedModel/finalModel.pth")

    return 



def run_epoch(model, optimizer, dataloader, mode='train'):
    """
    Run one epoch of training or validation.
    
    Arguments:
        model (nn.Sequential) : the embedding model 
        optimizer (torch.optim) : optimization algorithm for the model
        dataloader (torch.utils.data.DataLoader): contains the data used for training/testing.
        mode (String) : train or val
        
    Returns:
        Loss and accuracy in this epoch.
    """
    # Get the device based on the model
    device = next(model.parameters()).device
    #model = model.to(device)

    if mode == 'train':
        model.train()
    else:
        model.eval()

    epoch_loss = 0.0
    epoch_acc = 0.0

    ### Iterate over data
    for image, seg_mask in dataloader:
        image, seg_mask = image.to(device), seg_mask.to(device)

        ### Zero the parameter gradients
        if mode == 'train':
            model.zero_grad()

        ### Forward pass
        with torch.set_grad_enabled(True):
            # pass through deeplab
            # pass both through embedding head
            # get distance

            if mode == 'train':
                loss.backward()
                optimizer.step()

        ### Statistics, using Intersection over Union (IoU) for accuracy
        epoch_loss += loss.item()
        epoch_acc += IoU.item()
    
    epoch_loss /= len(dataloader.dataset)
    epoch_acc /= len(dataloader.dataset)
    return epoch_loss, epoch_acc
