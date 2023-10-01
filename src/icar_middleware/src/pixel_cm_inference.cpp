#include "icar_interfaces/srv/pixel_cm_inference.hpp"

#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "boost/date_time.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kINFO) {
            std::cout << msg << std::endl;
        }
    }
} logger;

class PixelCMModel {
   private:
    // Model and engine path
    // =====================
    std::string model_path;
    std::string engine_path;

    // TensorRT objects
    // ================
    nvinfer1::IRuntime* runtime;
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    std::string input_binding;
    std::string output_binding;

    // CUDA streams and buffers
    // ========================
    static const int num_streams = 1;
    cudaStream_t stream[num_streams];
    float* input_buffer[num_streams];
    float* output_buffer[num_streams];

   public:
    PixelCMModel(const std::string& model_path, const std::string& engine_path)
        : model_path(model_path), engine_path(engine_path) {}

    bool build() {
        /* This code block is checking if the model file and engine file exist. If the model file
        exists, it checks if the engine file also exists. If both files exist, it compares the last
        write times of the model file and the engine file. */
        if (boost::filesystem::exists(model_path)) {
            if (boost::filesystem::exists(engine_path)) {
                auto model_time = boost::filesystem::last_write_time(model_path);
                auto engine_time = boost::filesystem::last_write_time(engine_path);
                if (model_time < engine_time) {
                    std::cout << "Engine file is newer than model file, skipping build..." << std::endl;
                    return true;
                } else {
                    std::cout << "Model file is newer than engine file, rebuilding engine..." << std::endl;
                }
            } else {
                std::cout << "Engine file does not exist, building engine..." << std::endl;
            }
        } else {
            std::cerr << "Model file does not exist, aborting build..." << std::endl;
            return false;
        }

        /* The above code is creating an instance of the TensorRT builder, which is used to create and
        optimize TensorRT networks. It also creates an instance of the TensorRT network definition,
        which is used to define the structure of the network. Additionally, it creates an instance
        of the ONNX parser, which is used to parse ONNX models and populate the TensorRT network
        definition with layers and tensors. */
        nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(logger);
        uint32_t flag = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        nvinfer1::INetworkDefinition* network = builder->createNetworkV2(flag);
        nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);

        /* The above code is using a parser object to parse a model file specified by the `model_path`
        variable. It then checks for any errors during the parsing process and prints them to the console.
        If there are any errors, the function returns false. */
        parser->parseFromFile(model_path.c_str(), static_cast<int32_t>(nvinfer1::ILogger::Severity::kWARNING));
        for (int32_t i = 0; i < parser->getNbErrors(); ++i) {
            std::cout << parser->getError(i)->desc() << std::endl;
            return false;
        }

        /* The above code is creating a builder configuration object in the C++ programming language.
        It sets the memory pool limit for the workspace to 1GB (1U << 30). It also checks if the
        platform has fast FP16 support and if so, it sets the FP16 flag in the builder
        configuration. */
        nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1U << 30);
        if (builder->platformHasFastFp16()) {
            config->setFlag(nvinfer1::BuilderFlag::kFP16);
        }

        /* The line is for building a serialized version of the TensorRT network. */
        nvinfer1::IHostMemory* serializedModel = builder->buildSerializedNetwork(*network, *config);

        /* The above code is writing the serialized engine to the engine file. */
        std::ofstream engine_file(engine_path, std::ios::binary);
        engine_file.write(static_cast<const char*>(serializedModel->data()), serializedModel->size());
        engine_file.close();

        /* The code `delete parser; delete network; delete config; delete builder;` is deallocating the
        memory used by the objects created with the `new` keyword. This is done to prevent memory
        leaks and free up resources that are no longer needed. */
        delete parser;
        delete network;
        delete config;
        delete builder;

        return true;
    }

    bool load() {
        /* The code block is checking if the engine file exists. If the engine file does not exist, it prints
        an error message and returns false, indicating that the load operation failed. */
        if (!boost::filesystem::exists(engine_path)) {
            std::cerr << "Engine file does not exist, aborting load..." << std::endl;
            return false;
        }

        /* This code block is reading the contents of the engine file into memory. */
        std::ifstream engine_file(engine_path, std::ios::binary);
        engine_file.seekg(0, std::ios::end);
        size_t modelSize = engine_file.tellg();
        engine_file.seekg(0, std::ios::beg);
        std::vector<char> modelData(modelSize);
        engine_file.read(modelData.data(), modelSize);
        engine_file.close();

        /* `runtime = nvinfer1::createInferRuntime(logger);` is creating an instance of the TensorRT runtime
        using the provided logger. The TensorRT runtime is responsible for executing the inference engine on
        the target hardware. The `createInferRuntime` function takes in a logger object that is used to log
        messages during the runtime's operation. */
        runtime = nvinfer1::createInferRuntime(logger);
        if (runtime == nullptr) {
            std::cerr << "Failed to create TensorRT runtime, aborting load..." << std::endl;
            return false;
        }

        /* The line `engine = runtime->deserializeCudaEngine(modelData.data(), modelSize);` is
        deserializing the CUDA engine from the serialized model data. The `deserializeCudaEngine`
        function takes in the serialized model data and its size, and returns a pointer to the
        deserialized CUDA engine. The deserialized engine can then be used for inference on the
        target hardware. */
        engine = runtime->deserializeCudaEngine(modelData.data(), modelSize);
        if (engine == nullptr) {
            std::cerr << "Failed to deserialize CUDA engine, aborting load..." << std::endl;
            return false;
        }

        // =============================

        try {
            if (engine->getNbIOTensors() != 2) {
                std::cerr << "Expected 2 I/O tensors, aborting load..." << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return false;
        }

        try {
            if (engine->getTensorIOMode(engine->getIOTensorName(0)) != nvinfer1::TensorIOMode::kINPUT &&
                engine->getTensorIOMode(engine->getIOTensorName(1)) != nvinfer1::TensorIOMode::kOUTPUT) {
                input_binding = engine->getIOTensorName(1);
                output_binding = engine->getIOTensorName(0);
            } else if (engine->getTensorIOMode(engine->getIOTensorName(1)) != nvinfer1::TensorIOMode::kINPUT &&
                       engine->getTensorIOMode(engine->getIOTensorName(0)) != nvinfer1::TensorIOMode::kOUTPUT) {
                input_binding = engine->getIOTensorName(0);
                output_binding = engine->getIOTensorName(1);
            } else {
                std::cerr << "Expected 1 input and 1 output tensor, aborting load..." << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return false;
        }

        std::cout << "Input binding: " << input_binding << std::endl;
        std::cout << "Output binding: " << output_binding << std::endl;

        try {
            if (engine->getTensorShape(input_binding.c_str()).nbDims != 2 ||
                engine->getTensorShape(input_binding.c_str()).d[0] != 1 ||
                engine->getTensorShape(input_binding.c_str()).d[1] != 2 ||
                engine->getTensorDataType(input_binding.c_str()) != nvinfer1::DataType::kFLOAT) {
                std::cerr << "Input tensor has invalid data type or shape, aborting load..." << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return false;
        }

        try {
            if (engine->getTensorShape(output_binding.c_str()).nbDims != 2 ||
                engine->getTensorShape(output_binding.c_str()).d[0] != 1 ||
                engine->getTensorShape(output_binding.c_str()).d[1] != 2 ||
                engine->getTensorDataType(output_binding.c_str()) != nvinfer1::DataType::kFLOAT) {
                std::cerr << "Output tensor has invalid data type or shape, aborting load..." << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return false;
        }

        // =============================

        /* The line `context = engine->createExecutionContext();` is creating an execution context for
        the TensorRT engine. The execution context is responsible for managing the execution of the
        inference engine on the target hardware. It provides methods to set input data, run
        inference, and retrieve output data from the engine. */
        context = engine->createExecutionContext();
        if (context == nullptr) {
            std::cerr << "Failed to create execution context, aborting load..." << std::endl;
            return false;
        }

        // =============================

        /* This code is for creating multiple CUDA streams, allocating memory on the GPU for input and output
        arrays, and storing the device pointers to these arrays in device_input and device_output arrays. */
        for (int i = 0; i < num_streams; i++) {
            cudaStreamCreate(&stream[i]);
            cudaMalloc((void**)&input_buffer[i], 2 * sizeof(float));
            cudaMalloc((void**)&output_buffer[i], 2 * sizeof(float));
        }

        return true;
    }

    void forward(float* input, float* output) {
        /* The above code is performing asynchronous memory transfers and tensor operations using CUDA
        streams in C++. */
        for (int i = 0; i < num_streams; i++) {
            cudaMemcpyAsync(input_buffer[i], input + i * 2, 2 * sizeof(float), cudaMemcpyHostToDevice, stream[i]);
            context->setTensorAddress(input_binding.c_str(), input_buffer[i]);
            context->setTensorAddress(output_binding.c_str(), output_buffer[i]);
            context->enqueueV3(stream[i]);
            cudaMemcpyAsync(output + i * 2, output_buffer[i], 2 * sizeof(float), cudaMemcpyDeviceToHost, stream[i]);
        }

        /* The above code is synchronizing multiple CUDA streams. It iterates over each stream and calls
        the `cudaStreamSynchronize` function to ensure that all operations in each stream have
        completed before continuing. */
        for (int i = 0; i < num_streams; i++) {
            cudaStreamSynchronize(stream[i]);
        }
    }
};

class PixelCMInference : public rclcpp::Node {
   private:
    //-----Service server
    rclcpp::Service<icar_interfaces::srv::PixelCMInference>::SharedPtr ser_pixel2cm_mlp;
    rclcpp::Service<icar_interfaces::srv::PixelCMInference>::SharedPtr ser_cm2pixel_mlp;

    // Model and engine path
    // =====================
    std::string pixel2cm_model_path = std::string(getenv("HOME")) + "/icar-ng-data/model/pixel2cm_model.onnx";
    std::string pixel2cm_engine_path = std::string(getenv("HOME")) + "/icar-ng-data/model/pixel2cm_model.trt";
    std::string cm2pixel_model_path = std::string(getenv("HOME")) + "/icar-ng-data/model/cm2pixel_model.onnx";
    std::string cm2pixel_engine_path = std::string(getenv("HOME")) + "/icar-ng-data/model/cm2pixel_model.trt";

    // Multi layer perceptron
    // ======================
    PixelCMModel pixel2cm_mlp = PixelCMModel(pixel2cm_model_path, pixel2cm_engine_path);
    PixelCMModel cm2pixel_mlp = PixelCMModel(cm2pixel_model_path, cm2pixel_engine_path);

   public:
    PixelCMInference() : Node("pixel_cm_inference") {
        //-----Service server
        ser_pixel2cm_mlp = this->create_service<icar_interfaces::srv::PixelCMInference>(
            "/pixel2cm_mlp",
            std::bind(&PixelCMInference::cllbck_ser_pixel2cm_mlp, this, std::placeholders::_1, std::placeholders::_2));
        ser_cm2pixel_mlp = this->create_service<icar_interfaces::srv::PixelCMInference>(
            "/cm2pixel_mlp",
            std::bind(&PixelCMInference::cllbck_ser_cm2pixel_mlp, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Pixel2CM model");
        if (!pixel2cm_mlp.build()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build pixel2cm MLP");
            rclcpp::shutdown();
        }
        if (!pixel2cm_mlp.load()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load pixel2cm MLP");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "CM2Pixel model");
        if (!cm2pixel_mlp.build()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build cm2pixel MLP");
            rclcpp::shutdown();
        }
        if (!cm2pixel_mlp.load()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load cm2pixel MLP");
            rclcpp::shutdown();
        }
    }

    //==================================

    void cllbck_ser_pixel2cm_mlp(const std::shared_ptr<icar_interfaces::srv::PixelCMInference::Request> request,
                                 std::shared_ptr<icar_interfaces::srv::PixelCMInference::Response> response) {
        float input[2] = {request->x_in, request->y_in};
        float output[2] = {0.0f, 0.0f};
        pixel2cm_mlp.forward(input, output);
        response->x_out = output[0];
        response->y_out = output[1];
    }

    void cllbck_ser_cm2pixel_mlp(const std::shared_ptr<icar_interfaces::srv::PixelCMInference::Request> request,
                                 std::shared_ptr<icar_interfaces::srv::PixelCMInference::Response> response) {
        float input[2] = {request->x_in, request->y_in};
        float output[2] = {0.0f, 0.0f};
        cm2pixel_mlp.forward(input, output);
        response->x_out = output[0];
        response->y_out = output[1];
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PixelCMInference>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}