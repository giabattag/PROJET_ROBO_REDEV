from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    sl.node('slider_publisher', 'slider_publisher', arguments = [sl.find('projet_robo_redev', 'drone_cmd.yaml')])
    sl.node('projet_robo_redev', 'control')
    sl.include('projet_robo_redev', 'demo.launch.py')

    return sl.launch_description()