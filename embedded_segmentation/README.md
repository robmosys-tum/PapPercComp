# Video Object Segmentation through Latent Space Distribution
**Author**: Mike Yan Michelis

We present a video object segmentation method that is inspired by the following embedding method (our pipeline is quite similar to theirs, the image below helps understanding both methods): 

`Chen, Y., Pont-Tuset, J., Montes, A., & Van Gool, L. (2018). Blazingly fast video object segmentation with pixel-wise metric learning.`

![Paper Figure](BlazeFast.PNG)


# Prerequisites

The Python packages we use are **Numpy** (version 1.18.5) and **PyTorch** (version 1.6.0+cu101), for visualization we use **matplotlib** (version 3.2.2).
For the C++ part we additionally use **OpenCV** (version 3.2.0). The versions we used are in the parentheses.

---

# Usage

For training the DAVIS2016 dataset is required, which can be downloaded at <https://davischallenge.org/davis2016/code.html>. This dataset should be extracted into the current directory under "Data/DAVIS/...". We also provide an example that does not require DAVIS2016, see Subsection "Test Examples" for instructions on how to quickly use our trained model for evaluation.

The basic structure of a command looks like:

```
python main.py --mode <mode> --custom_data <custom_data_directory> --seg_dir <ground_truth_segmentation_directory>
```

When no custom data is given, the DAVIS2016 dataset is used for either training or validation. Several choices for `mode` include:

- 'train': currently always trains on DAVIS2016 480p training dataset. No need for `custom_data` or `seg_dir` parameters.

- 'validation': by default takes DAVIS2016 480p validation data, but can be overwritten to use custom data images in `custom_data` with their corresponding ground truth segmentation masks in `seg_dir`. The ground truth is used to compute the Intersection over Union metric at the end.

- 'inference': At test time, no ground truth is known, hence `seg_dir` should be empty. Instead, a single annotated first frame should be given in the same directory as `custom_data` called "AnnotatedFrame.png".

Several other parameters that can be given include:

- `--batch_size <number_of_batches>` for multi-batch processing.

- `--epochs <number_of_epochs>` to specify for how long training should occur.

- `--checkpoint_epochs <number_of_epochs>` will save the model every so many epochs.

- `--load` is a flag (no arguments) to start training from existing model. During validation and inference this flag is required. A trained model is loaded from the same directory where a trained model would be saved during training.

- `--videos <video_name1> <video_name2> ...` will load certain videos from the DAVIS2016 dataset, these can then be trained separately.


We have also added the possibility of creating videos for DAVIS2016 segmentations, where the original image, predicted mask and ground truth are displayed side by side. After running the segmentation, outputs will be generated and stored in the "Output/" directory, a video is then created in this output folder by calling:
```
python utility.py --videos <video_name>
```
Here the `video_name` should match the DAVIS2016 video you used for validation/inference.


#### Example commands
For training:
```
python main.py --mode train --batch_size 5 --epochs 30 -- checkpoint_epochs 1
```

For validation:
```
python main.py --mode validation --batch_size 5 --load --videos soapbox
```


### Test Examples

For testing purposes (without having to download the whole DAVIS2016 dataset) we have included a trained model in this repository together with some custom data (in "Custom/" and "SegDir/", containing the RGB images and the ground truth segmentation masks respectively). After installing PyTorch, you should be able to run:
```
python main.py --mode inference --load --custom_data Custom
```
Or for validation (where the IoU score is also computed):
```
python main.py --mode validation --load --custom_data Custom --seg_dir Segdir
```






# C++ Torch Script

The above **PyTorch** implementation of the model is trained on Python, but might need to be used in a C++ application for evaluation. This is possible through **LibTorch**, the conversion process will be described in the following.

## Conversion

In order to run **PyTorch** models in C++, we need to convert the model into a **Torch Script**. How we do it is through *Tracing*. This mechanism takes an example input, puts it through the PyTorch network and gathers gradients and network information in this manner.

> **Important note**: if you wish to evaluate the model on a CPU, you should be tracing the model on a CPU too.

The first step is to fully load your trained model `model`. Then create an example input with the correct shape (using a random values) `example`. Lastly run it through the network and store the traced model as follows:

```python
from architecture import EmbeddingHead, deeplabModel
import torch

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
embedModel = EmbeddingHead(64)
embedModel.apply(EmbeddingHead.initialize_weight)
embedModel.to(device);
deeplabModel.to(device);

embedModel.load_state_dict(torch.load("TrainedModel/finalModel.pth", map_location=device))

class Lambda(torch.nn.Module):
    def __init__(self, func):
        super().__init__()
        self.func = func

    def forward(self, x):
        return self.func(x)

model = torch.nn.Sequential(deeplabModel,
                      Lambda(lambda x: x['out']),
                      embedModel)

model.to(device);

example = torch.rand(1, 3, 480, 854)
example = example.to(device)

traced_script_module = torch.jit.trace(model, example)
traced_script_module.save("/some_dir/tracedModel.pt")
```

The above code is an example of how tracing could work, however, several details such as image resolution, device, output folder should all be changed accordingly.


## Loading Torch Script in LibTorch

Now that we have the trained model as a traced script, we can load it in C++ using the **LibTorch** library. After installing the library, you can include the `<torch/script.h>` header and run:

```cpp
torch::jit::script::Module module = torch::jit::load(model_path);
```

A more detailed explanation can be found in <https://pytorch.org/tutorials/advanced/cpp_export.html>.


## Linux VM

Due to RAM constraints and CUDA not being directly available on the VM, we suggest running the model in C++ using eval mode and disabling gradient computation:

```cpp
module.eval();
{
    torch::NoGradGuard no_grad;
    at::Tensor output = module.forward(inputs).toTensor();
}
```

From experiments our model seems to use about 2GB of RAM on the VM in this manner, running on CPU.


## Papyrus Component

Using `colcon build` and then `ros2 run vos VOS` you can run the Papyrus component defined in the VOS directory. However, make sure to first change the paths in "VOS/src/VOSCompdef/VOS.cpp" for annotated\_path and model\_path. The first path should point to the image file that contains the annotated first frame for the video, and the second path should point to the traced model that you want to load.

Due to the traced model containing the entire network architecture and model parameters, it has a size of 240 MB, which is too large to provide on this repository. As the trained model's state dict in PyTorch is only 3 MB, we have provided that one in "TrainedModel/finalModel.pth", and ask the user to trace the model manually to run the ROS2 component.

Our VOS component is *subscribed* to the `/image` topic and *publishes* images to the `/segmentation` topic. Both *message types* are "sensor_msgs::msg::Image".


# FAQ

- **Q**: I downloaded the DAVIS2016 dataset and tried running training/validation on the "bear" video, but it tells me there's a shape mismatch error.

    **A**: Segmentation mask `00077.png` in this dataset seems to have an incorrect shape, you should find a way to reduce the number of channels in this segmentation mask from [2, 480, 854] to [1, 480, 854]. I edited the image in NumPy. This extra channel contains all 255 values, presumably opacity/alpha values.

- **Q**: I'm having trouble getting LibTorch to work or loading the traced model into C++.
  
    **A**: Several issues we battled with included: 

    1) We traced the trained model on GPU (e.g. on Google Colab) and tried loading it into C++ LibTorch on CPU (as our Linux VM did not have CUDA). Both must be on GPU or both on CPU. You cannot mix and match those two.

    2) "rclcpp" is compiled on ABI=1, therefore if the code has to run together with LibTorch, that library should also be compiled on ABI=1. Be careful which one you're downloading on the LibTorch homepage.
   
    3) Don't  forget adding LibTorch to PATH.

- **Q**: Inference is not working!

    **A**: Make sure "AnnotatedFrame.png" is given in your custom data folder. It requires this first frame's segmentation mask to evaluate the frames of your custom video.
