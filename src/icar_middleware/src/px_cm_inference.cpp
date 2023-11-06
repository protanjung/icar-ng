#include "icar_interfaces/srv/px_cm_inference.hpp"

#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace nvinfer1;
using namespace nvonnxparser;

class Logger : public ILogger {
  void log(Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kWARNING) { std::cout << msg << std::endl; }
  }
} logger;

class PXCMModel {
 private:
  std::string model_path;
  std::string engine_path;

  IRuntime *runtime;
  ICudaEngine *engine;
  IExecutionContext *context;
  std::string input_binding_name;
  std::string output_binding_name;

  static const int num_streams = 1;
  cudaStream_t stream[num_streams];
  float *input_buffer[num_streams];
  float *output_buffer[num_streams];

 public:
  PXCMModel(std::string model_path, std::string engine_path) : model_path(model_path), engine_path(engine_path) {}

  bool build() {
    // Check if model and engine exist and if the engine is newer than the model.
    if (boost::filesystem::exists(model_path)) {
      if (boost::filesystem::exists(engine_path)) {
        auto model_time = boost::filesystem::last_write_time(model_path);
        auto engine_time = boost::filesystem::last_write_time(engine_path);
        if (model_time < engine_time) {
          std::cout << "Engine file (" << engine_path << ") is newer than model file (" << model_path
                    << "). Skipping build." << std::endl;
          return true;
        } else {
          std::cout << "Model file (" << model_path << ") is newer than engine file (" << engine_path
                    << "). Rebuilding engine." << std::endl;
        }
      } else {
        std::cout << "Engine file (" << engine_path << ") does not exist. Building engine." << std::endl;
      }
    } else {
      std::cout << "Model file (" << model_path << ") does not exist. Aborting build." << std::endl;
      return false;
    }

    // Build the network.
    IBuilder *builder = createInferBuilder(logger);
    uint32_t flag = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    INetworkDefinition *network = builder->createNetworkV2(flag);

    IParser *parser = createParser(*network, logger);
    parser->parseFromFile(model_path.c_str(), static_cast<int32_t>(ILogger::Severity::kWARNING));
    for (int32_t i = 0; i < parser->getNbErrors(); ++i) { std::cout << parser->getError(i)->desc() << std::endl; }

    IBuilderConfig *config = builder->createBuilderConfig();
    config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1U << 20);
    IHostMemory *buffer = builder->buildSerializedNetwork(*network, *config);

    // Write the engine file.
    std::ofstream engine_file(engine_path, std::ios::binary);
    engine_file.write(reinterpret_cast<const char *>(buffer->data()), buffer->size());
    engine_file.close();

    // Clean up.
    delete parser;
    delete network;
    delete config;
    delete builder;
    delete buffer;

    return true;
  }

  bool load() {
    // Check if the engine file exists
    if (!boost::filesystem::exists(engine_path)) {
      std::cout << "Engine file (" << engine_path << ") does not exist. Aborting load." << std::endl;
      return false;
    }

    // Open the engine file, get the size of the engine file, and read the engine file into a buffer.
    std::ifstream engine_file(engine_path, std::ios::binary);
    engine_file.seekg(0, std::ios::end);
    size_t size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    engine_file.read(buffer.data(), size);
    engine_file.close();

    // Create the runtime.
    runtime = createInferRuntime(logger);
    if (runtime == nullptr) {
      std::cout << "Failed to create runtime." << std::endl;
      return false;
    }

    // Create the engine.
    engine = runtime->deserializeCudaEngine(buffer.data(), size);
    if (engine == nullptr) {
      std::cout << "Failed to deserialize engine." << std::endl;
      return false;
    }

    try {
      // Check number of bindings
      if (engine->getNbIOTensors() != 2) {
        std::cout << "Expected 2 bindings, but found " << engine->getNbIOTensors() << "." << std::endl;
        return false;
      }

      // Find input and output binding names
      bool input_found = false, output_found = false;
      for (int i = 0; i < 2; i++) {
        if (engine->getTensorIOMode(engine->getIOTensorName(i)) == TensorIOMode::kINPUT && !input_found) {
          input_binding_name = engine->getIOTensorName(i);
          input_found = true;
        } else if (engine->getTensorIOMode(engine->getIOTensorName(i)) == TensorIOMode::kOUTPUT && !output_found) {
          output_binding_name = engine->getIOTensorName(i);
          output_found = true;
        }
      }
      if (!input_found || !output_found) {
        std::cout << "Failed to find input and output binding names." << std::endl;
        return false;
      }

      // Check input tensor shape
      if (engine->getTensorShape(input_binding_name.c_str()).nbDims != 2 ||
          engine->getTensorShape(input_binding_name.c_str()).d[0] != 1 ||
          engine->getTensorShape(input_binding_name.c_str()).d[1] != 2 ||
          engine->getTensorDataType(input_binding_name.c_str()) != DataType::kFLOAT) {
        std::cout << "Input tensor has wrong shape or wrong type." << std::endl;
        return false;
      }

      // Check output tensor shape
      if (engine->getTensorShape(output_binding_name.c_str()).nbDims != 2 ||
          engine->getTensorShape(output_binding_name.c_str()).d[0] != 1 ||
          engine->getTensorShape(output_binding_name.c_str()).d[1] != 2 ||
          engine->getTensorDataType(output_binding_name.c_str()) != DataType::kFLOAT) {
        std::cout << "Output tensor has wrong shape or wrong type." << std::endl;
        return false;
      }
    } catch (const std::exception &e) {
      std::cerr << e.what() << '\n';
      return false;
    }

    // Create execution context
    context = engine->createExecutionContext();
    if (context == nullptr) {
      std::cout << "Failed to create execution context." << std::endl;
      return false;
    }

    // Create streams and allocate buffers
    for (int i = 0; i < num_streams; i++) {
      cudaStreamCreate(&stream[i]);
      cudaMalloc((void **)&input_buffer[i], 2 * sizeof(float));
      cudaMalloc((void **)&output_buffer[i], 2 * sizeof(float));
    }

    return true;
  }

  void forward(float *input, float *output) {
    for (int i = 0; i < num_streams; i++) {
      // Copy the input from the host to the GPU.
      cudaMemcpyAsync(input_buffer[i], input + 2 * i, 2 * sizeof(float), cudaMemcpyHostToDevice, stream[i]);
      // Set input and output buffer addresses.
      context->setTensorAddress(input_binding_name.c_str(), input_buffer[i]);
      context->setTensorAddress(output_binding_name.c_str(), output_buffer[i]);
      // Run inference.
      context->enqueueV3(stream[i]);
      // Copy the output from the GPU to the host.
      cudaMemcpyAsync(output + 2 * i, output_buffer[i], 2 * sizeof(float), cudaMemcpyDeviceToHost, stream[i]);
    }

    for (int i = 0; i < num_streams; i++) {
      // Wait for the stream to finish.
      cudaStreamSynchronize(stream[i]);
    }
  }
};

class PXCMInference : public rclcpp::Node {
 public:
  //-----Service server
  rclcpp::Service<icar_interfaces::srv::PxCmInference>::SharedPtr service_pxcm_mlp;
  rclcpp::Service<icar_interfaces::srv::PxCmInference>::SharedPtr service_cmpx_mlp;

  // Model and engine path
  // =====================
  std::string pxcm_model_path = std::string(getenv("HOME")) + "/icar-ng-data/model/pixel2cm_model.onnx";
  std::string cmpx_model_path = std::string(getenv("HOME")) + "/icar-ng-data/model/cm2pixel_model.onnx";
  std::string pxcm_engine_path = std::string(getenv("HOME")) + "/icar-ng-data/model/pixel2cm_model.trt";
  std::string cmpx_engine_path = std::string(getenv("HOME")) + "/icar-ng-data/model/cm2pixel_model.trt";

  // Multi layer perceptron
  // ======================
  PXCMModel pxcm_mlp = PXCMModel(pxcm_model_path, pxcm_engine_path);
  PXCMModel cmpx_mlp = PXCMModel(cmpx_model_path, cmpx_engine_path);

  PXCMInference() : Node("px_cm_inference") {
    //-----Service server
    service_pxcm_mlp = this->create_service<icar_interfaces::srv::PxCmInference>(
        "pxcm_mlp", std::bind(&PXCMInference::cllbck_sub_pxcm_mlp, this, std::placeholders::_1, std::placeholders::_2));
    service_cmpx_mlp = this->create_service<icar_interfaces::srv::PxCmInference>(
        "cmpx_mlp", std::bind(&PXCMInference::cllbck_sub_cmpx_mlp, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_WARN(this->get_logger(), "Build and load pixel to cm model");
    if (pxcm_mlp.build() == false) {
      RCLCPP_ERROR(this->get_logger(), "Failed to build pixel to cm model");
      rclcpp::shutdown();
    }
    if (cmpx_mlp.load() == false) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load cm to pixel model");
      rclcpp::shutdown();
    }

    RCLCPP_WARN(this->get_logger(), "Build and load cm to pixel model");
    if (cmpx_mlp.build() == false) {
      RCLCPP_ERROR(this->get_logger(), "Failed to build cm to pixel model");
      rclcpp::shutdown();
    }
    if (pxcm_mlp.load() == false) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load pixel to cm model");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_pxcm_mlp(
      const std::shared_ptr<icar_interfaces::srv::PxCmInference::Request> request,
      std::shared_ptr<icar_interfaces::srv::PxCmInference::Response> response) {
    float input[2] = {request->x, request->y};
    float output[2] = {0.0, 0.0};
    pxcm_mlp.forward(input, output);
    response->x = output[0];
    response->y = output[1];
  }

  void cllbck_sub_cmpx_mlp(
      const std::shared_ptr<icar_interfaces::srv::PxCmInference::Request> request,
      std::shared_ptr<icar_interfaces::srv::PxCmInference::Response> response) {
    float input[2] = {request->x, request->y};
    float output[2] = {0.0, 0.0};
    cmpx_mlp.forward(input, output);
    response->x = output[0];
    response->y = output[1];
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_px_cm_inference = std::make_shared<PXCMInference>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_px_cm_inference);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}