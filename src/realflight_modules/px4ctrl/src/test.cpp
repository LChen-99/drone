#include <iostream>
#include "torch/script.h"
#include "torch/torch.h"
#include <memory>
#include <chrono>
#include "torch_model.h"
// #include "opencv2/core.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgcodecs.hpp"
#include <vector>
using namespace std;
int main(){
    // std::shared_ptr<torch::jit::script::Module> module = torch::jit::load("/home/luochen/neural-fly-main/models/neural-fly_dim-a-3_v-q-pwm-epoch-950.pth");
  NetworkModel model("/home/luochen/neural-fly-main/torch_script_eval.pt");
	Eigen::VectorXd feature = Eigen::VectorXd::Ones(11);
	cout << model.forward(feature) << endl;
	
	
	torch::jit::script::Module module;
	try {
	// Deserialize the ScriptModule from a file using torch::jit::load().
		module = torch::jit::load("/home/luochen/neural-fly-main/torch_script_eval.pt");
	}
	catch (const c10::Error& e) {
				std::cerr << "error loading the model\n";
				return -1;
	}
	// cout << "1" << endl;
	// std::vector<torch::jit::IValue> inputs;
	std::vector<torch::jit::IValue> inputs;
	inputs.push_back(torch::ones({11}, at::kDouble) );	
	cout << module.forward(inputs).toTensor();
	//第一次传播后速度很快。。。
	// auto start = std::chrono::system_clock::now();
	// for(int i = 0; i < 4; i++){
	// 	std::vector<torch::jit::IValue> inputs;
	// 	inputs.push_back(torch::rand({11}, at::kDouble) );	
	// 	module.forward(inputs);
		
	// }
	// cout << "cost " << chrono::duration_cast<chrono::milliseconds>(std::chrono::system_clock::now() - start).count() << "ms" << endl;;
	
	// inputs.push_back(torch::rand({11}, at::kDouble)) ;
	// // std::cout << inputs[0] << '\n';
	// // cout << "2" << endl;
	// // Execute the model and turn its output into a tensor.
	// auto start = std::chrono::system_clock::now();
	// // at::Tensor output = module.forward(inputs).toTensor();
	// module.forward(inputs);
	// cout << "cost " << chrono::duration_cast<chrono::milliseconds>(std::chrono::system_clock::now() - start).count() << "ms" << endl;;
	// cout << "3" << endl;
	// std::cout << output << '\n';
    // assert(module != nullptr);
    
	return 0;
}