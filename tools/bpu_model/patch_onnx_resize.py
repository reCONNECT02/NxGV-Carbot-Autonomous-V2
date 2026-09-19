import onnx
from onnx import helper, TensorProto, numpy_helper

def patch_onnx(file_path):
    print(f"Loading {file_path}...")
    model = onnx.load(file_path)
    
    # 1. Ensure IR version <= 9
    print(f"Original IR version: {model.ir_version}")
    model.ir_version = 9
    
    # 2. Ensure opset version is 11
    for opset in model.opset_import:
        if opset.domain == "" or opset.domain == "ai.onnx":
            print(f"Original opset version: {opset.version}")
            opset.version = 11

    # Get initializer dictionary
    init_dict = {init.name: init for init in model.graph.initializer}

    # 3. Create a dummy initializer for roi
    roi_dummy_name = "roi_dummy"
    if roi_dummy_name not in init_dict:
        print("Creating dummy initializer for roi...")
        roi_dummy = helper.make_tensor(
            name=roi_dummy_name,
            data_type=TensorProto.FLOAT,
            dims=[0],
            vals=[]
        )
        model.graph.initializer.append(roi_dummy)
    
    # 4. Patch nodes
    patched_resize_nodes = 0
    patched_reshape_nodes = 0
    patched_split_nodes = 0
    
    # Opset 11 Resize allowed attributes
    allowed_resize_attrs = {
        "coordinate_transformation_mode",
        "cubic_coeff_a",
        "exclude_outside",
        "extrapolation_value",
        "mode",
        "nearest_mode"
    }
    
    for node in model.graph.node:
        # Patch Resize nodes
        if node.op_type == "Resize":
            if len(node.input) > 1 and node.input[1] == "":
                print(f"Patching node inputs for {node.name}...")
                node.input[1] = roi_dummy_name
                patched_resize_nodes += 1
            
            # Remove attributes not allowed in Opset 11 Resize
            attrs_to_remove = []
            for attr in node.attribute:
                if attr.name not in allowed_resize_attrs:
                    print(f"Removing unsupported attribute {attr.name} from {node.name}...")
                    attrs_to_remove.append(attr)
            
            for attr in attrs_to_remove:
                node.attribute.remove(attr)
                
        # Patch Reshape nodes
        elif node.op_type == "Reshape":
            attrs_to_remove = []
            for attr in node.attribute:
                if attr.name == "allowzero":
                    print(f"Removing unsupported attribute allowzero from Reshape node {node.name}...")
                    attrs_to_remove.append(attr)
            
            for attr in attrs_to_remove:
                node.attribute.remove(attr)
                patched_reshape_nodes += 1
                
        # Patch Split nodes
        elif node.op_type == "Split":
            # In opset 11, Split takes 1 input and split sizes are in the 'split' attribute.
            if len(node.input) == 2:
                split_tensor_name = node.input[1]
                if split_tensor_name in init_dict:
                    print(f"Converting Split node {node.name} to opset 11 (second input -> attribute)...")
                    split_tensor = init_dict[split_tensor_name]
                    split_array = numpy_helper.to_array(split_tensor)
                    split_values = [int(x) for x in split_array]
                    
                    # Add split attribute
                    new_attr = helper.make_attribute("split", split_values)
                    node.attribute.append(new_attr)
                    
                    # Remove the second input
                    node.input.pop(1)
                    patched_split_nodes += 1
                else:
                    print(f"Warning: Split node {node.name} has 2 inputs, but the second input '{split_tensor_name}' is not in initializers.")
                    
    print(f"Patched {patched_resize_nodes} Resize nodes.")
    print(f"Patched {patched_reshape_nodes} Reshape nodes.")
    print(f"Patched {patched_split_nodes} Split nodes.")
    
    # 5. Validate the model
    print("Running onnx checker...")
    try:
        onnx.checker.check_model(model)
        print("ONNX model is valid!")
    except Exception as e:
        print(f"ONNX checker failed: {e}")
        
    print(f"Saving patched model to {file_path}...")
    onnx.save(model, file_path)
    print("Done!")

if __name__ == "__main__":
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "best.onnx")
    patch_onnx(model_path)
