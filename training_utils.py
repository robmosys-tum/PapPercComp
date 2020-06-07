import torch
from torch import nn
from torch.optim import Optimizer
from torch.utils.data import DataLoader
from tqdm.autonotebook import tqdm
from trashnet.data.constants import *

CLASSES = {
    GLASS: "glass",
    PAPER: "paper",
    CARDBOARD: "cardboard",
    PLASTIC: "plastic",
    METAL: "metal",
    TRASH: "trash"
}


def train_model(model: nn.Module, train: DataLoader, val: DataLoader, optimizer: Optimizer, epochs: int = 1,
                lossfunc: torch.nn = torch.nn.CrossEntropyLoss(), debugprint=False):
    """
    Train a model on the input dataset.
    Parameters
    ----------
    model: nn.Module
        The input model to be trained.
    dataset: torch.utils.data.Dataset
        The dataset to train on.
    batch_size: int
        The training batch size.
    loss_function: function with signature: (x, y, model, **kwargs) -> (loss, logits).
        The function used to compute the loss.
    optimizer: Optimizer
        The model's optimizer.
    epochs: int
        Number of epochs to train for. Default: 1.
    loss_args: dict or None
        Additional arguments to be passed to the loss function.

    Returns
    -------
    Tuple containing
        * losses: List[float]. The losses obtained at each step.
        * accuracies: List[float]. The accuracies obtained at each step.

    """
    losses = []
    accuracies = []
    # for epoch in tqdm(range(epochs), total=epochs):
    for epoch in range(epochs):
        for x, y in tqdm(iter(train), total=len(train)):
        # for x, y in iter(train):
            ##########################################################
            # YOUR CODE HERE
            x = x.to(model.device)
            y = y.to(model.device)
            optimizer.zero_grad()
            x = x.transpose(1, 3)
            x = x.transpose(2, 3)
            x = model(x)
            loss = lossfunc(x, y)
            if debugprint:
                print("Epoch ", epoch, " Loss: ", loss.item())
            losses.append(loss.item())

            loss.backward()
            optimizer.step()

            acc = 0
            total = 0
            model.eval()
            for valX, valY in iter(val):
                valX = valX.to(model.device)
                valY = valY.to(model.device)
                valX = valX.transpose(1, 3)
                valX = valX.transpose(2, 3)
                valX = model(valX).argmax(1)
                acc += float(torch.sum(valX == valY).item())
                total += len(valY)
            if debugprint:
                print("Epoch ", epoch, " Acc: ", acc / total)
            accuracies.append(acc / total)
            model.train()
            ##########################################################
    return losses, accuracies
