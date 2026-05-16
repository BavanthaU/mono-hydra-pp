import onnx
import onnxoptimizer

# Load the model
model_path = "/home/bavantha/hydra2_ws/src/mono_hydra_vio/params/RealSense_RGBD/modified_model.onnx"
optimized_model_path = "/home/bavantha/hydra2_ws/src/mono_hydra_vio/params/RealSense_RGBD/optimized_model.onnx"

# Load the ONNX model
model = onnx.load(model_path)

# Specify optimization passes
passes = [
    "eliminate_identity",
    "eliminate_deadend",
    "fuse_bn_into_conv",
    "fuse_add_bias_into_conv",
]

# Optimize the model
optimized_model = onnxoptimizer.optimize(model, passes)

# Save the optimized model
onnx.save(optimized_model, optimized_model_path)
print(f"Optimized model saved to {optimized_model_path}")
