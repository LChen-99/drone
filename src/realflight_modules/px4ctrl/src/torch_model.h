#include <iostream>
#include "torch/script.h"
#include "torch/torch.h"
#include <memory>
#include <chrono>
#include <string>
#include <Eigen/Core>
class NetworkModel{
public:
	NetworkModel(std::string model_path){
		try {
	// Deserialize the ScriptModule from a file using torch::jit::load().
			module_ = torch::jit::load("/home/luochen/neural-fly-main/torch_script_eval.pt");
			//第一次传播速度很慢，初始化一下。。。
			std::vector<torch::jit::IValue> inputs;
			inputs.push_back(torch::rand({11}, at::kDouble) );	
			module_.forward(inputs);
		}
		catch (const c10::Error& e) {
			std::cerr << "error loading the model\n";
		}
	}
	Eigen::Vector3d forward(Eigen::VectorXd feature);
	
private:
	torch::jit::script::Module module_;

};