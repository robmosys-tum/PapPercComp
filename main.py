# ------------------------------------------------------------------------------
# This file should be called using command line, which is then parsed and finds
# the correct files to run.
# ------------------------------------------------------------------------------

from parse import get_arguments


if __name__ == "__main__":
    args = get_arguments()

    if args.mode == 'train':
        ### Actual training
        pass
    elif args.mode == 'test':
        pass