import yaml, os
import rospy

objects_config_file = rospy.get_param(f'/cgras/objects_config', default=os.path.join(os.path.dirname(__file__), '../../../config/objects.yaml'))
with open(objects_config_file, 'r') as f:
    objects_config = yaml.safe_load(f)
print(objects_config)
    
for object in objects_config['attached_objects']:
    print(object)