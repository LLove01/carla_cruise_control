# carla_cruise_control
Implementing cruise control in Carla simulator.

# carla_autonomous
Implementing cruise control in Carla simulator 

# setup
works for python 3.8 <br/>
carla currently supports 3.6, 3.7, 3.8  <br/>
object_detection library requires python > 3.5, != 3.7

carla python api reference: https://carla.readthedocs.io/en/latest/python_api/

# object detection 
current version uses tensorflow object detection api  <br/><br/>
tensorflow/models repo: https://github.com/tensorflow/models  <br/>
protobuf instalation: https://github.com/protocolbuffers/protobuf/releases  <br/>
good example: protoc-3.20.1-win64.zip  <br/><br/>
protobuf compilation:  <br/>
From within TensorFlow/models/research/
C:/location/on/disc/protoc object_detection/protos/*.proto --python_out=.  <br/><br/>

help with instalation and compilation: https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/install.html#protobuf-installation-compilation

# additions 
package must be installed:  <br/> 
pip install object_detection_api

# running the code 
STARTING CARLA: .\CarlaUE4 -dx11 -quality-level=Low -carla-port=3000 <br/> <br/>
PYTHON API::  <br/>
first activate local python 3.8 environment: py38_env\Scripts\activate  <br/>
cd to code folder  <br/>
python control_node.py

