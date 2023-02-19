#include "torch_model.h"

Eigen::Vector3d NetworkModel::forward(Eigen::VectorXd feature){
    if(feature.size() != 11) {
        std::cout << "featrue size is not 11" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    std::vector<torch::jit::IValue> inputs;
    auto temp = torch::rand({11}, at::kDouble);
    for(size_t i = 0; i < feature.size(); i++){
        temp[i] = feature[i];
    }
    inputs.push_back(temp);
    auto output = module_.forward(inputs).toTensor();
    return Eigen::Vector3d(output[0].item().toDouble(), output[1].item().toDouble(), output[2].item().toDouble());
}