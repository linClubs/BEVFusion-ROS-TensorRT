from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
	
	node = Node(
		package="bevfusion",  
		executable="bevfusion_node",  
		name='bevfusion_node',       
		output='screen', 
		
		# 参数列表
		parameters=[
			{'model_name': 'resnet50'},
			{'precision' : 'int16'}
		]
	)

	return LaunchDescription([
		node
	])