# ------------------------------------------------------------------------------
# Parser function for command line arguments.
# ------------------------------------------------------------------------------

from argparse import ArgumentParser


def get_arguments():
    parser = ArgumentParser()

    ### IO Parameters
    parser.add_argument('--mode', help="Mode in which you want to run the network.", choices=['train', 'val'], default='train')

    ### Training Parameters
    parser.add_argument('--batch_size', help="Batch size to use for the dataloader.", type=int, default=16)
    parser.add_argument('--num_workers', help="Number of workers for the dataloader.", type=int, default=0)

    parser.add_argument('--epochs', help="Number of epochs to run the model for.", type=int, default=1000)
    parser.add_argument('--checkpoint_epochs', help="Number of epochs between two checkpoints.", type=int, default=100)


    arguments = parser.parse_args()
    return arguments
