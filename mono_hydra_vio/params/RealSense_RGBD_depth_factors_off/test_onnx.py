import onnx
from onnx import helper, TensorProto, checker
import onnxruntime as ort
import numpy as np

# Paths
input_model_path = "/home/bavantha/hydra2_ws/src/mono_hydra_vio/params/RealSense_RGBD/superpoint_v1_sim_int32.onnx"
output_model_path = "/home/bavantha/hydra2_ws/src/mono_hydra_vio/params/RealSense_RGBD/modified_model.onnx"

# Load the ONNX model
onnx_model = onnx.load(input_model_path)

# Function to insert a Cast node
def insert_cast_node(input_name, casted_output_name, target_type):
    return helper.make_node(
        "Cast",
        inputs=[input_name],
        outputs=[casted_output_name],
        to=target_type,  # Target data type
        name=f"Cast_{casted_output_name}"
    )

# Inspect nodes and identify type mismatches
print("Inspecting all Concat nodes...")
concat_nodes = []
for node in onnx_model.graph.node:
    if node.op_type == "Concat":
        concat_nodes.append(node)
        print(f"Node Name: {node.name}, Inputs: {node.input}, Outputs: {node.output}")

print("Inspecting all Mul nodes...")
mul_nodes = []
for node in onnx_model.graph.node:
    if node.op_type == "Mul":
        mul_nodes.append(node)
        print(f"Node Name: {node.name}, Inputs: {node.input}, Outputs: {node.output}")

# Fix type mismatches
print("Fixing node type mismatches...")
nodes_to_insert = []
casted_initializers = {}  # Track casted initializers to avoid duplication
for node in mul_nodes + concat_nodes:
    for i, input_name in enumerate(node.input):
        # Check if the input is an initializer
        initializer = next((init for init in onnx_model.graph.initializer if init.name == input_name), None)
        if initializer and initializer.data_type != TensorProto.INT64:
            if input_name not in casted_initializers:
                # Create a unique casted output name
                cast_output_name = f"casted_{input_name}_{i}"
                cast_node = insert_cast_node(initializer.name, cast_output_name, TensorProto.INT64)
                casted_initializers[input_name] = cast_output_name
                nodes_to_insert.append((cast_node, node, i))
            else:
                cast_output_name = casted_initializers[input_name]
            # Update node input to use Cast output
            node.input[i] = cast_output_name

# Insert Cast nodes into the graph
print("Inserting Cast nodes into the graph...")
for cast_node, target_node, input_index in nodes_to_insert:
    # Find the index of the target node
    target_node_index = next(
        i for i, n in enumerate(onnx_model.graph.node) if n == target_node
    )
    # Insert the Cast node before the target node
    onnx_model.graph.node.insert(target_node_index, cast_node)

# Validate and save the modified model
print("Validating modified model...")
checker.check_model(onnx_model)
onnx.save(onnx_model, output_model_path)
print(f"Modified model saved to {output_model_path}")

# Test the modified model
print("Testing modified model...")
try:
    session = ort.InferenceSession(output_model_path)
    input_name = session.get_inputs()[0].name
    dummy_input = np.random.rand(1, 1, 480, 752).astype(np.float32)
    output = session.run(None, {input_name: dummy_input})
    print("Model outputs:", output)
except Exception as e:
    print(f"Error during testing: {e}")
