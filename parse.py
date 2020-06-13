# ------------------------------------------------------------------------------
# Parser function for command line arguments.
# ------------------------------------------------------------------------------

from argparse import ArgumentParser


def get_arguments():
    parser = ArgumentParser()


    ### IO Parameters
    parser.add_argument('--mode', choices=['train', 'test'])

    arguments = parser.parse_args()
    return arguments
