from roboflow import Roboflow
 
rf = Roboflow(api_key='Od85usHEWCQhNI7ZCKOD')
project = rf.workspace('mercy2green').project('circle-ch9fd')
 
version = project.version(1)
version.deploy("yolov5", f"/media/m2g/workdisk/workspace/GitHub/m2g_racing/yolov5/runs/train/exp8")