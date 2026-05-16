# Kimera-RPGO
Small Python bound library for outlier robust pose-graph optimization (RPGO).

## Dependencies
This project depends on GTSAM.

## Install (C++)
```
git clone git@github.com:MIT-SPARK/Kimera-RPGO.git kimera_rpgo
cd kimera_rpgo
mkdir build && cd build
cmake .. && make -j4
```

## Install (Python)
```
git clone git@github.com:MIT-SPARK/Kimera-RPGO.git kimera_rpgo
cd kimera_rpgo
pip install -e .
```

## Experiment
Simple example is given in `experiments/simple_example.py`
For example, using a nonlinear least squres solver simply do
```bash
python experiments/simple_example.py <input-g2o> <output-g2o> -nls
```
To run with a Huber loss
```bash
python experiments/simple_example.py <input-g2o> <output-g2o> -nls -r huber
```
Gradient descent is also supported
```bash
python experiments/simple_example.py <input-g2o> <output-g2o> -gd
```
To run with GNC (Graduated Non-Convexity) with some inlier probability (0.7)
```bash
python experiments/simple_example.py <input-g2o> <output-g2o> -nls -g 0.7
```