import cv2
import numpy as np
import matplotlib.pyplot as plt
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit


# Load TensorRT Engine
def load_engine(engine_path):
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())


# Perform Inference
def infer_with_tensorrt(engine_path, input_data):
    engine = load_engine(engine_path)
    context = engine.create_execution_context()

    # Determine input/output bindings
    input_binding_index = -1
    output_binding_index = -1

    # Loop through bindings to determine input and output indices
    num_bindings = len(engine)
    for binding_index in range(num_bindings):
        binding_name = engine.get_binding_name(binding_index)
        if engine.binding_is_input(binding_index):
            input_binding_index = binding_index
        else:
            output_binding_index = binding_index

    if input_binding_index == -1 or output_binding_index == -1:
        raise ValueError("Could not find valid input or output bindings.")

    # Get binding shapes
    input_shape = tuple(engine.get_binding_shape(input_binding_index))
    output_shape = tuple(engine.get_binding_shape(output_binding_index))

    # Prepare input and output buffers
    input_data = np.ascontiguousarray(input_data, dtype=np.float32)
    output_data = np.empty(output_shape, dtype=np.float32)

    # Allocate memory on device
    d_input = cuda.mem_alloc(input_data.nbytes)
    d_output = cuda.mem_alloc(output_data.nbytes)

    # Transfer input data to device
    cuda.memcpy_htod(d_input, input_data)

    # Execute inference
    bindings = [int(d_input), int(d_output)]
    context.execute_v2(bindings)

    # Transfer output data to host
    cuda.memcpy_dtoh(output_data, d_output)

    return output_data


# Preprocess the Image
def preprocess_image(image_path, input_shape):
    # Read and preprocess the image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"Image not found at {image_path}")
    image_resized = cv2.resize(image, (input_shape[3], input_shape[2]))
    image_normalized = image_resized.astype(np.float32) / 255.0
    input_data = np.expand_dims(image_normalized, axis=(0, 1))  # Shape: [1, 1, H, W]
    return input_data, image


# Postprocess and Visualize Keypoints
def visualize_keypoints(image, keypoints, threshold=0.015):
    h, w = image.shape
    plt.figure(figsize=(10, 10))
    plt.imshow(image, cmap="gray")
    for y in range(keypoints.shape[1]):
        for x in range(keypoints.shape[2]):
            if keypoints[0, y, x] > threshold:
                plt.scatter(x, y, color="red", s=5)
    plt.title("Detected Keypoints")
    plt.axis("off")
    plt.show()


# Main Function
def main(engine_path, image_path):
    # Input shape for the TensorRT model
    input_shape = (1, 1, 240, 320)  # Adjust based on your model input shape

    # Load and preprocess the image
    input_data, original_image = preprocess_image(image_path, input_shape)

    # Perform inference
    keypoints = infer_with_tensorrt(engine_path, input_data)

    # Visualize the detected keypoints
    visualize_keypoints(original_image, keypoints)


# Example Usage
if __name__ == "__main__":
    engine_path = "/home/bavantha/hydra2_ws/src/mono_hydra_vio/params/RealSense_RGBD/optimized_model.trt"
    image_path = "/home/bavantha/Downloads/out.jpg"  # Replace with your image path
    main(engine_path, image_path)
