# realsense_gazebo

This is a Gazebo-ROS integration package for simulated Realsense D400 series cameras.

This package provides slightly edited official URDF model of the camera which has proper mass and inertia in all parts. It also contains everything that is needed to get the camera running in a Gazebo simulation, providing the color, infra, depth and pointcloud streams (of course, infra streams are normal mono color, not real infra). This package tries to mimick the real RealSense as much as possible, e.g. by properly naming the topics etc.

## Example usage

Put this snippet in your model's Xacro file to get the camera in simulation:

```XML
<!-- camera_bottom_attach is a link that should exist in your model and the camera will attach to it -->
<xacro:include filename="$(find realsense_gazebo)/urdf/_d435.urdf.xacro"/>
<xacro:gzrs_sensor_d435 parent="camera_bottom_attach" name="camera_bottom" use_nominal_extrinsics="true" add_plug="false" use_mesh="false">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:gzrs_sensor_d435>

<xacro:include filename="$(find realsense_gazebo)/urdf/realsense.gazebo.xacro" />
<xacro:realsense_gazebo camera_name="camera_bottom" visualize_color="true" />
```

When running Xacro, you should pass it argument `rendering_target:=sdf-classic`, otherwise no simulation sensors will be generated.

And also add this to your simulation launch file:

```XML
<launch>
    ...
    <include file="$(find realsense_gazebo)/launch/gazebo_classic.launch">
        <arg name="camera_name" value="camera_bottom" />
    </include>
</launch>
```

The `<xacro:realsense_gazebo>` macro has a lot of configuration options deciding which parts of the camera should be simulated.