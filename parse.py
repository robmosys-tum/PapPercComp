# ------------------------------------------------------------------------------
# Parser function for command line arguments.
# ------------------------------------------------------------------------------

from argparse import ArgumentParser


def get_arguments():
    parser = ArgumentParser()

    ### IO Parameters
    parser.add_argument('--mode', help="Mode in which you want to run the network.", choices=['train', 'validation', 'inference'], default='train')
    parser.add_argument('--load', help="Loads existing trained model if available.", action="store_true")
    parser.add_argument('--custom_data', help="Directory in which the custom data for inference can be found.", default=None)
    parser.add_argument('--seg_dir', help="Directory in which the ground truth segmentation masks for the custom data can be found.", default=None)
    parser.add_argument('--videos', help="One or several video names of which the frames should be loaded.", nargs='+', default=None)


    ### Training Parameters
    parser.add_argument('--batch_size', help="Batch size to use for the dataloader.", type=int, default=1)
    parser.add_argument('--num_workers', help="Number of workers for the dataloader.", type=int, default=0)

    parser.add_argument('--epochs', help="Number of epochs to run the model for.", type=int, default=10)
    parser.add_argument('--checkpoint_epochs', help="Number of epochs between two checkpoints.", type=int, default=1)

    parser.add_argument('--embedding_dim', help="Dimension of the embedding space.", type=int, default=128)


    arguments = parser.parse_args()
    return arguments
