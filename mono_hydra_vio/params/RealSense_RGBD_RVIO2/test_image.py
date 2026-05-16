import cv2
import numpy as np
import onnxruntime as ort

def preprocess_image(image_path, input_shape):
    """
    Preprocess the input image to match the ONNX model's input format.
    """
    # Replace dynamic dimensions with actual values
    model_height, model_width = 480, 752  # Replace with your desired height and width

    # Load the image
    original_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if original_image is None:
        raise FileNotFoundError(f"Image not found at path: {image_path}")

    # Resize the image to match the model's input size
    image_resized = cv2.resize(original_image, (model_width, model_height))
    image_resized = image_resized.astype(np.float32) / 255.0  # Normalize to [0, 1]

    # Add batch and channel dimensions
    input_tensor = np.expand_dims(np.expand_dims(image_resized, axis=0), axis=0)
    return input_tensor, original_image

def infer_keypoints(model_path, image_path):
    """
    Load the ONNX model, perform inference on the input image, and visualize keypoints.
    """
    # Load the ONNX model
    session = ort.InferenceSession(model_path)
    input_shape = session.get_inputs()[0].shape  # Get input shape from the model

    print(f"Input shape from ONNX model: {input_shape}")

    # Preprocess the input image
    input_tensor, original_image = preprocess_image(image_path, input_shape)

    # Perform inference
    outputs = session.run(None, {session.get_inputs()[0].name: input_tensor})

    # Process outputs (example for scores)
    scores = outputs[0][0, :, :]  # Adjust this indexing based on actual output
    keypoints = np.argwhere(scores > 0.01)  # Example threshold
    keypoints[:, [0, 1]] = keypoints[:, [1, 0]]  # Swap x, y for visualization

    # Rescale keypoints to original image dimensions
    model_height, model_width = 480, 752
    orig_height, orig_width = original_image.shape

    scale_x = orig_width / model_width
    scale_y = orig_height / model_height

    keypoints_rescaled = keypoints.astype(np.float32)
    keypoints_rescaled[:, 0] *= scale_x
    keypoints_rescaled[:, 1] *= scale_y

    # Visualize the keypoints
    for kp in keypoints_rescaled:
        cv2.circle(original_image, (int(kp[0]), int(kp[1])), 3, (0, 255, 0), -1)

    # Display the image with keypoints
    cv2.imshow("Keypoints", original_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    model_path = "/home/bavantha/hydra2_ws/src/mono_hydra_vio/params/RealSense_RGBD/optimized_model.onnx"
    image_path = "/home/bavantha/Downloads/out.jpg"
    infer_keypoints(model_path, image_path)
