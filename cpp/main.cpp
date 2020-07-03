#include <torch/script.h>
#include <opencv2/opencv.hpp>

#define GLASS 0
#define PAPER 1
#define CARDBOARD 2
#define PLASTIC 3
#define METAL 4
#define TRASH 5

static std::unordered_map<int, std::string> CLASSES = {
		{GLASS, "GLASS" },
		{ PAPER, "PAPER" },
		{ CARDBOARD, "CARDBOARD" },
		{ PLASTIC, "PLASTIC" },
		{ METAL, "METAL" },
		{ TRASH, "TRASH" }
};

int
main(int argc, char** argv)
{

	if (argc < 3) {
		std::cout<<"Usage: " << argv[0] << " [Model Path] [Camera Number]\n";
		return 0;
	}

	auto module = torch::jit::load(argv[1]);
	int camNum = atoi(argv[2]);
//	cv::Mat img = cv::imread("/home/seedship/TUM/SS20/Model-Driven\ Approach\ for\ Robotics\ Perception/PapPercComp/trashnet/data/dataset-resized/glass/glass10.jpg");
	cv::Mat img;

	cv::VideoCapture cap;

	if(!cap.open(camNum, cv::CAP_ANY))
		std::cout << "Could not open camera!\n";
	while (cap.isOpened() && cap.read(img))
	{
		torch::Tensor img_tensor = torch::from_blob(img.data, {img.rows, img.cols, 3}, torch::kByte).clone();
		img_tensor = img_tensor.permute({2, 0, 1}); // convert to CxHxW
		img_tensor = img_tensor / 255.0;

		img_tensor = img_tensor.unsqueeze(0).cuda();
		std::vector<torch::jit::IValue> input;
		input.emplace_back(img_tensor);

		torch::Tensor output = module.forward(input).toTensor();
		auto prediction = output.topk(5);
		auto topClasses = std::get<1>(prediction);
		topClasses = topClasses.flatten();

		std::cout << "Predicted Class:";
		for (unsigned idx = 0; idx < 5; idx++) {
			std::cout << " " << CLASSES[topClasses[idx].item().toInt()];
		}
		std::cout << std::endl;

		imshow("ocv", img);
		if (cv::waitKey(10) == 27) break;
	}
	return 0;
}
