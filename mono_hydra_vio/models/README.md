# ITC SuperPoint model

Model binaries are supplied locally and ignored by Git. To run the ITC
SuperPoint variant, export `superpoint_480x848.onnx` into this directory using
`../scripts/export_superpoint_onnx.py` and separately obtained SuperPoint v1
weights. See the package README for the ONNX Runtime build settings.
The classical RVIO2 fallback does not use this model.
