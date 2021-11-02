#!/usr/bin/env python
import rospy
# import yaml
# import rospkg
# import os



rospy.init_node('yaml_test')
arg1 = rospy.get_param('use_sim_time')
print(arg1)
arg2 = rospy.get_param('kd')
print(arg2)


# # Get the path to the package
# rospack = rospkg.RosPack()
# pkg_path = rospack.get_path('launch_config')
# print(pkg_path)

# path_to_yaml = os.path.join(pkg_path, "launch", "mallard_params.yaml")
# print(path_to_yaml)



# # path_to_yaml = "yaml_example2.yaml"

# with open(path_to_yaml, 'r') as file:
#     try:
#         parsed_yaml_file = yaml.safe_load(file)
#         # print(parsed_yaml_file)
        
#     except yaml.YAMLError as err:
#         print(err)

# print("yaml_example2.yaml")
# print(parsed_yaml_file)
# for key, value in parsed_yaml_file.items():
#     print('{}: {}'.format(key, value))

# print('')









# path_to_yaml = "/home/gyal/mallard_ws/src/mallard_sim/launch_config/launch/yaml_example.yaml"

# with open(path_to_yaml, 'r') as file:
#     try:
#         parsed_yaml_file = yaml.safe_load(file)
#         # print(parsed_yaml_file)
        
#     except yaml.YAMLError as err:
#         print(err)

# print("yaml_example.yaml")
# print(parsed_yaml_file["a_dictionary"], "\n")
# print(parsed_yaml_file.get("a_list"), "\n")


# path_to_yaml = "/home/gyal/mallard_ws/src/mallard_sim/launch_config/cfg/yaml/vectored_mallard.yaml"

# with open(path_to_yaml, 'r') as file:
#     try:
#         parsed_yaml_file = yaml.safe_load(file)
#         # print(parsed_yaml_file)
        
#     except yaml.YAMLError as err:
#         print(err)

# print("vectored_mallard.yaml")
# print(parsed_yaml_file["controllers"], "\n")
# print(parsed_yaml_file.get("controllers"), "\n")
# print(parsed_yaml_file["controllers"]["config"], "\n")
# vel = parsed_yaml_file["controllers"]["x_thr_left"]["velocity"]
# print(vel['p'], "\n")

# print(parsed_yaml_file.get("x_thr_left"), "\n")


# print(parsed_yaml_file["x_thr_left"], "\n") # error