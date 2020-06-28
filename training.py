# ------------------------------------------------------------------------------
# Training the network defined in architecture using the data loaded using data-
# loader.
# Also provides the validation and inference modes on already trained models.
# ------------------------------------------------------------------------------

import torch
import os
import matplotlib.pyplot as plt

from architecture import PMLEmbedding, deeplabModel, EmbeddingHead
from utility import load_model, calc_IoU
from PIL import Image



def run_model(dataloader, args):
    """
    Run a model in either training mode or validation mode. If the model exists, then the weights are loaded.

    Arguments:
        dataloader (torch.utils.data.DataLoader) : contains the data used for training/testing.
        args (argparse.ArgumentParser) : command line arguments
        
    Returns:
        None
    """
    # Load the device on which we're running the model
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

    ### Initialize embedding network and DeepLabv3
    embedModel = EmbeddingHead(args.embedding_dim)
    embedModel.apply(EmbeddingHead.initialize_weight)
    embedModel.to(device)

    deeplabModel.to(device)


    if args.load:
        # TODO: Check if I need to assign the value to model, or if load_state_dict works on model directly (not sure if passed by reference).
        load_model(embedModel)
    else:
        # TrainedModel directory might not exist yet
        if not os.path.exists("TrainedModel"):
            os.makedirs("TrainedModel")


    if args.mode == 'train':
        ### Create optimizer and scheduler
        optimizer = torch.optim.SGD(
                            embedModel.parameters(), 
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
            epoch_loss, epoch_acc = run_epoch(embedModel, optimizer, dataloader, args.mode)
            # Update the learning rate based on scheduler
            lr_scheduler.step()

            
            # Print statistics
            print(f"Epoch {epoch + 1: >4}/{args.epochs}, Loss: {epoch_loss:.2e}, Accuracy: {epoch_acc * 100:.2f}%")


            # Make double checkpoints, just in case.
            if epoch % args.checkpoint_epochs == 0:
                torch.save(embedModel.state_dict(), "TrainedModel/modelBackup.pth")
            elif epoch % args.checkpoint_epochs-1 == 0:
                torch.save(embedModel.state_dict(), "TrainedModel/modelBackupBackup.pth")
        
        ### Save final model
        torch.save(embedModel.state_dict(), "TrainedModel/finalModel.pth")


    elif args.mode == 'inference' or args.mode == 'validation':
        # Create a color pallette, selecting a color for each class
        palette = torch.tensor([2 ** 25 - 1, 2 ** 15 - 1, 2 ** 21 - 1])
        colors = torch.as_tensor([i for i in range(21)])[:, None] * palette
        colors = (colors % 255).numpy().astype("uint8")

        imcount = 0
        IoU_sum = 0

        # Output directory might not exist yet
        if not os.path.exists("Output"):
            os.makedirs("Output")

        for im, ground_truth in dataloader:
            im = im.to(device)
            ground_truth = ground_truth.to(device)
            
            # Using no_grad() is a necessity! Otherwise memory usage will be far too high.
            with torch.no_grad():
                output = deeplabModel(im)['out']
        
            IoU_sum += calc_IoU(output.argmax(1, keepdims=True), ground_truth).sum()

            # Iterate over all images in the batch
            for out, gt in zip(output, ground_truth):
                output_predictions = out.argmax(0)
                
                # Plot the semantic segmentation predictions of 21 classes in each color
                seg = Image.fromarray(output_predictions.byte().cpu().numpy())
                seg.putpalette(colors)
                
                plt.imsave(f"Output/{imcount : 06d}.png", seg)
                imcount += 1

        # Compute mean Intersection over Union
        mean_IoU = IoU_sum / imcount
        print(f"The mean Intersection over Union for this dataset : {mean_IoU: .4f}")

    return 



def run_epoch(embedModel, deeplabModel, optimizer, dataloader, mode='train'):
    """
    Run one epoch of training or validation.
    
    Arguments:
        embedModel (nn.Sequential) : the embedding model.
        deeplabModel (nn.Sequential) : the pretrained DeepLabv3 model.
        optimizer (torch.optim) : optimization algorithm for the model.
        dataloader (torch.utils.data.DataLoader): contains the data used for training/testing.
        mode (String) : train or val
        
    Returns:
        Loss and accuracy in this epoch.
    """
    # Get the device based on the model
    device = next(embedModel.parameters()).device
    #embedModel = embedModel.to(device)

    if mode == 'train':
        embedModel.train()
    else:
        embedModel.eval()

    epoch_loss = 0.0
    epoch_acc = 0.0

    ### Iterate over data
    for image, seg_mask in dataloader:
        image, seg_mask = image.to(device), seg_mask.to(device)

        ### Zero the parameter gradients
        if mode == 'train':
            embedModel.zero_grad()

        ### Forward pass
        # No gradient for DeepLab
        with torch.no_grad():
            deepOut = deeplabModel(image)

        # Gradient for embedding network
        with torch.set_grad_enabled(True):
            embedded_pixels = embedModel(deepOut)

            if mode == 'train':
                loss.backward()
                optimizer.step()

        ### Statistics, using Intersection over Union (IoU) for accuracy
        epoch_loss += loss.item()
        epoch_acc += IoU.item()
    
    epoch_loss /= len(dataloader.dataset)
    epoch_acc /= len(dataloader.dataset)
    return epoch_loss, epoch_acc
