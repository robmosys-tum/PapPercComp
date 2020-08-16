import torch
from torch import nn
from torch.optim import Optimizer
from torch.utils.data import DataLoader
from tqdm.autonotebook import tqdm
from trashnet.data.constants import *
from matplotlib import pyplot as plt

CLASSES = {
    GLASS: "glass",
    PAPER: "paper",
    CARDBOARD: "cardboard",
    PLASTIC: "plastic",
    METAL: "metal",
    TRASH: "trash"
}


def train_model(model: nn.Module, train: DataLoader, val: DataLoader, optimizer: Optimizer, epochs: int = 1,
                lossfunc: torch.nn = torch.nn.CrossEntropyLoss(), transform = None, debugprint=False):
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
            if transform is not None:
                transform_batch(x, transform)
            x = x.to(model.device)
            y = y.to(model.device)
            optimizer.zero_grad()
            x = model(x)
            loss = lossfunc(x, y)
            if debugprint:
                print("Epoch ", epoch, " Loss: ", loss.item())
            losses.append(loss.item())

            loss.backward()
            optimizer.step()

            acc = evaluate(model, val)
            if debugprint:
                print("Epoch ", epoch, " Acc: ", acc)
            accuracies.append(acc)
            model.train()
            ##########################################################
    return losses, accuracies


def evaluate(model: nn.Module, eval_dataloader: DataLoader):
    model.eval()
    acc = 0
    total = 0
    for x, y in iter(eval_dataloader):
        x = x.to(model.device)
        y = y.to(model.device)
        x = model(x)
        x = x.argmax(1)
        acc += float(torch.sum(x == y).item())
        total += len(y)
    return acc / total


def plot(x: torch.Tensor, y: int):
    plt.axis("off")
    plt.imshow(x.permute(1, 2, 0))
    plt.title(f"Label: {CLASSES[y]}")
    plt.show()

def transform_batch(x: torch.Tensor, transform):
    for idx in range(x.shape[0]):
        x[idx] = transform(x[idx])