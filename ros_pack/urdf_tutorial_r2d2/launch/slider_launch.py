from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    sl.node('slider_publisher', 'slider_publisher', arguments = [sl.find('urdf_tutorial_r2d2', 'drone_cmd.yaml')])
    sl.node('urdf_tutorial_r2d2', 'control')
    sl.include('urdf_tutorial_r2d2', 'demo.launch.py')

    return sl.launch_description()