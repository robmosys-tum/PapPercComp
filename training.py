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
        load_model(embedModel)
    else:
        # TrainedModel directory might not exist yet
        if not os.path.exists("TrainedModel"):
            os.makedirs("TrainedModel")


    if args.mode == 'train':
        ### Create optimizer and scheduler
        optimizer = torch.optim.SGD(
                            embedModel.parameters(), 
                            lr=1e-3, 
                            momentum=0.9, 
                            weight_decay=1e-4
                        )

        lr_scheduler = torch.optim.lr_scheduler.MultiStepLR(
                            optimizer, 
                            milestones=[int(args.epochs/3), int(2*args.epochs/3)], 
                            gamma=0.1
                        )

        epoch_history = []

        ### Run epochs
        for epoch in range(args.epochs):
            print(f"Starting Epoch [{epoch+1} / {args.epochs}]")
            
            # When multiple dataloaders are given, we iterate over these as well, running each dataloader for an epoch.
            if isinstance(dataloader, list):
                epoch_loss = 0
                epoch_acc = 0

                for i in range(len(dataloader)):
                    print(f"Processing video [{i+1} / {len(dataloader)}]")

                    loss, acc = run_epoch(embedModel, deeplabModel, optimizer, dataloader[i], mode=args.mode)

                    epoch_loss += loss
                    epoch_acc += acc

                epoch_loss /= len(dataloader)
                epoch_acc /= len(dataloader)


            else:
                epoch_loss, epoch_acc = run_epoch(embedModel, deeplabModel, optimizer, dataloader, mode=args.mode)

            ### Update the learning rate based on scheduler
            lr_scheduler.step()

            
            ### Print statistics
            print("-"*30)
            print(f"Epoch [{epoch + 1: >4}/{args.epochs}] Loss: {epoch_loss:.2e}")
            print("-"*30)

            epoch_history.append(epoch_loss)

            ### Make double checkpoints, just in case.
            if epoch % args.checkpoint_epochs == 0:
                torch.save(embedModel.state_dict(), "TrainedModel/modelBackup.pth")

                ### Create and Store Plots
                plt.figure(figsize=(12,9))
                plt.plot(epoch_history, label='Loss History')

                plt.xlabel('Epoch')
                plt.ylabel('Loss')
                plt.xlim(0, args.epochs)
                plt.legend()
                plt.grid(True)
                plt.savefig("TrainedModel/loss_plot.png", bbox_inches='tight')

            elif epoch % args.checkpoint_epochs-1 == 0:
                torch.save(embedModel.state_dict(), "TrainedModel/modelBackupBackup.pth")
        
        ### Save final model
        torch.save(embedModel.state_dict(), "TrainedModel/finalModel.pth")



    elif args.mode == 'validation':
        # In case we're validating against certain video sequences
        if isinstance(dataloader, list):
                epoch_loss = 0
                epoch_acc = 0

                for i in range(len(dataloader)):
                    print(f"Processing video [{i+1} / {len(dataloader)}]")

                    loss, acc = run_epoch(embedModel, deeplabModel, None, dataloader[i], mode=args.mode)

                    epoch_loss += loss
                    epoch_acc += acc

                    # Print statistics per video
                    print("-"*30)
                    print(f"Validation IoU: {100*epoch_acc:.2f}")
                    print("-"*30)

                epoch_loss /= len(dataloader)
                epoch_acc /= len(dataloader)


        # In case we're dealing with custom data
        else:
            epoch_loss, epoch_acc = run_epoch(embedModel, deeplabModel, None, dataloader, mode=args.mode)


        ### Print statistics
        print("-"*30)
        print(f"Mean Validation IoU: {100*epoch_acc:.2f}")
        print("-"*30)



    elif args.mode == 'inference':
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
        Loss and IoU in this epoch. Loss only relevant for training, IoU only relevant for validation.
    """
    # Get the device based on the model
    device = next(embedModel.parameters()).device

    if mode == 'train':
        embedModel.train()
    else:
        embedModel.eval()

    epoch_loss = 0.0
    IoU_sum = 0.0
    iter_count = 1
    imcount = 0

    # Assign first image and its segmentation mask as reference during first loop through dataloader. DO NOT shuffle the dataloader for validation.
    reference_image = None
    reference_mask = None
    mean_refFG = None
    mean_refBG = None

    
    ### Iterate over data
    for image, seg_mask in dataloader:
        print(f"Processing data [{iter_count}/{len(dataloader)}]")

        image, seg_mask = image.to(device), seg_mask.to(device)


        ### Forward pass
        # No gradient for DeepLab
        with torch.no_grad():
            deepOut = deeplabModel(image)['out']

        if mode == 'train':
            # Gradient for embedding network
            with torch.set_grad_enabled(True):
                ### Zero the parameter gradients
                embedModel.zero_grad()

                embedded_pixelvectors = embedModel(deepOut)

                foreground = embedded_pixelvectors * seg_mask
                background = embedded_pixelvectors * (1 - seg_mask)

                N, d = embedded_pixelvectors.size()[:2]
                eps = 1e-5
                n_tot = embedded_pixelvectors.size()[2] * embedded_pixelvectors.size()[3]
                n_FG = seg_mask.sum(dim=[-1,-2])

                ### Mean and variance resulting in shapes [N, d]
                mean_FG = foreground.view(N, d, -1).sum(dim=2) / (n_FG + eps) 
                std_FG = ((foreground.view(N, d, -1) - mean_FG.view(N, d, 1))**2).sum(dim=2) / (n_FG + eps) 

                mean_BG = background.view(N, d, -1).sum(dim=2) / (n_tot - n_FG + eps) 
                std_BG = ((background.view(N, d, -1) - mean_BG.view(N, d, 1))**2).sum(dim=2) / (n_tot - n_FG + eps)
                
                
                torch.set_printoptions(precision=5)
                print(f"\n Mean FG: \n {mean_FG[0:2,0:4]} \n and Mean BG: \n {mean_BG[0:2,0:4]} \n and STD FG: \n {std_FG[0:2,0:4]} \n and STD BG: \n {std_BG[0:2,0:4]} \n")

                ### Loss function: getting foreground pixels close together, background pixels also close together, then the distance between FG and BG clusters far apart.
                std_loss = std_FG.mean() + std_BG.mean()

                L2 = nn.MSELoss()
                mean_margin_loss = - 0.5 * L2(mean_BG, mean_FG)
                
                # Then add regularization
                reg_strength = 1
                reg_loss = L2(-10, mean_FG) + L2(10, mean_BG)
                
                loss = (
                        0.9 * std_loss 
                        + reg_strength * reg_loss
                    )
                
                print(f"Loss is: {loss.item():.6f} of which std has {std_loss.item():.4f} and reg has {reg_loss.item():.4f} \n")


                # In case you wanna debug memory usage for PyTorch
                #print(torch.cuda.memory_summary(device))

            loss.backward()
            optimizer.step()


        elif mode == 'validation':
            ### Only during first loop: assign references and compute its embedding
            if reference_image is None:
                reference_image = image[0]
                reference_mask = seg_mask[0]

                with torch.no_grad():
                    embedded_reference = embedModel(deepOut[0:])

                    # TODO: I probably want kNN here instead of distance to mean.
                    foreground = embedded_reference * reference_mask
                    background = embedded_reference * (1 - reference_mask)

                    mean_refFG = foreground.mean(dim=[2,3], keepdims=True)
                    mean_refBG = background.mean(dim=[2,3], keepdims=True)


            ### Compute embeddings and see to which mean of the reference image every pixel is closer.
            with torch.no_grad():
                embedded_pixelvectors = embedModel(deepOut)

                distFG = ((embedded_pixelvectors - mean_refFG)**2).mean(dim=1, keepdims=True)    
                # Result has shape [N,1,H,W]

                distBG = ((embedded_pixelvectors - mean_refBG)**2).mean(dim=1, keepdims=True)    
                # Result has shape [N,1,H,W]

                predMask = torch.where(distFG > distBG, torch.ones_like(distFG), torch.zeros_like(distFG))


            IoU = calc_IoU(predMask, seg_mask).sum()

            # Iterate over all images in the batch
            for pred in predMask:
                seg = Image.fromarray(pred.byte().cpu().numpy())
                plt.imsave(f"Output/{imcount : 06d}.png", seg)
                imcount += 1



        ### Statistics, using Intersection over Union (IoU) for accuracy
        epoch_loss += loss.item()
        IoU_sum += IoU.item()

        iter_count += 1


    epoch_loss /= len(dataloader.dataset)
    IoU_sum /= len(dataloader.dataset)

    return epoch_loss, IoU_sum
