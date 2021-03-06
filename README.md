# sdf-modelica
Simulation Description Format to Modelica models converter. 

[![ZenHub](https://img.shields.io/badge/Shipping_faster_with-ZenHub-blue.svg?style=flat-square)](https://zenhub.com)

# Overview
- [About versioning](#️about-versioning)
- [Background](#background)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Naming](#naming)

# About versioning
**The project is undergoing _heavy_ development and should be consired in alpha development stage: the APIs and the tools behaviour will be subject to changes quite often.
 Furthermore, the amount of implemented feature is quite limited.**.

# Background
This software provides the user a tool (`sdf2modelica`) and a C++ library to convert 3D rigid body mechanical
systems described by the [Gazebo Simulator](http://gazebosim.org/)'s [Simulation Description Format files](http://sdformat.org/) to 
[Modelica models](https://www.modelica.org/) that use the [Modelica.Mechanics.MultiBody](https://doc.modelica.org/help/Modelica_Mechanics_MultiBody.html) library.

The software uses the standard reference [`sdformat` library](http://sdformat.org) to parse SDF file. As this library provides support to automatically
convert [ROS](http://www.ros.org/)'s [Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf) to SDF files, `sdf-modelica` can also be used to
convert mechanical systems represented using URDF to Modelica simulation models.

# Dependencies
sdf-modelica library depends on
- [SDFormat](http://sdformat.org/) - `version >= 6.0`

# Installation

As the first step, install [SDFormat](http://sdformat.org/) and make sure that
it can be found by [CMake](https://cmake.org/), either by installing it on a
system location or by adding its install prefix to the [`CMAKE_PREFIX_PATH` enviromental variable](https://cmake.org/cmake/help/v3.5/command/find_package.html).

After that, use the following commands to build and install `sdf-modelica`.

With `make` facilities:
```bash
$ git clone https://github.com/robotology-playground/sdf-modelica
$ cd sdf-modelica
$ mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX=<install_prefix> ..
$ make
$ [sudo] make install
```

With IDE build tool facilities, such as Visual Studio on Windows:
```bash
$ git clone https://github.com/robotology-playground/sdf-modelica
$ cd sdf-modelica
$ mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX=<install_prefix> ..
$ cmake --build . --target ALL_BUILD --config Release
$ cmake --build . --target INSTALL --config Release
```

# Usage
Add the directory `<install_prefix>/bin` (where `<install_prefix>` was specified in the CMake configuration step) to your [`PATH`](https://en.wikipedia.org/wiki/PATH_(variable)).

Then you can use `sdf2modelica` to convert a URDF or SDF model to a `.mo` Modelica model.

## Current limitations
* URDF files are converted to SDF files via sdformat before being converted to Modelica files.
  For more information in this automatic conversion process, see http://gazebosim.org/tutorials/?tut=ros_urdf .
  For this reason, all links connected to their parent links via fixed joints are automatically lumped in their parent.
* Only models connected to the world ( http://gazebosim.org/tutorials/?tut=ros_urdf#RigidlyFixingAModeltotheWorld ) are currently supported.
* No collision or contact simulation is supported.
* No animation/visualization information is converted from SDF to Modelica.

## Example usage
If you have an example urdf, contained in the file `twoLinks.urdf`:
~~~xml
<robot name="twoLinks">
    <!-- See http://gazebosim.org/tutorials/?tut=ros_urdf#RigidlyFixingAModeltotheWorld -->
    <link name="world"/>
    <joint name="fixed" type="fixed">
        <parent link="world"/>
        <child link="link1"/>
    </joint>
    <link name="link1">
        <inertial>
            <mass value="1" />
            <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
        </inertial>
    </link>
    <joint name="joint_1_2" type="continuous">
        <origin xyz="1 0 0" rpy="0 -0 0" />
        <axis xyz="0 0 1" />
        <parent link="link1" />
        <child link="link2" />
    </joint>
    <link name="link2">
        <inertial>
            <mass value="1" />
            <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
            <origin xyz="1 0 0" rpy="0 -0 0" />
        </inertial>
    </link>
</robot>
~~~
The relative Modelica model generated by the command:
~~~
sdf2modelica twoLinks.urdf twoLinks.mo
~~~
is the following model:
~~~modelica
// Modelica model automatically generated by sdf_modelica from ../../src/sdf-modelica-lib/test/twoLinks.urdf
// Model stucture heavily inspired by Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.MechanicalStructure
model twoLinks
  "Model of the mechanical part of the twoLinks robot"
  import SI = Modelica.SIunits;
  parameter Boolean animation=true "= true, if animation shall be enabled";
  parameter SI.Acceleration g=9.81 "Gravity acceleration";

  // Position, velocity, acceleration and torques/force variables for each joint







  SI.Angle q_joint_1_2 "Joint angle for joint joint_1_2";
  SI.AngularVelocity qd_joint_1_2 "Joint speed for joint joint_1_2";
  SI.AngularAcceleration qdd_joint_1_2 "Joint acceleration for joint joint_1_2";
  SI.Torque tau_joint_1_2 "Joint torque for joint joint_1_2";






  // Interface of the model: the driving flanges for each joint






  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis_joint_1_2 annotation(Placement(transformation(origin = {-210, -170}, extent={{-10, -10}, {10, 10}})));





  // TODO(traversaro) : document this
  inner Modelica.Mechanics.MultiBody.World world(
    g=(g),
    n={0,-1,0},
    animateWorld=false,
    animateGravity=false,
    enableAnimation=animation) annotation();

  // Instantiate a Modelica.Mechanics.MultiBody.Joints for each joint



  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixed_fixedRotation(angle=0.000000, n={1,0,0}, r={0,0,0}, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis) annotation();


  Modelica.Mechanics.MultiBody.Joints.Revolute joint_1_2(n={0,0,1}, useAxisFlange=true,
      animation=animation) annotation();


  Modelica.Mechanics.MultiBody.Parts.FixedRotation joint_1_2_fixedRotation(angle=0.000000, n={1,0,0}, r={-1,0,0}, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis) annotation();


  // Instantiate a Modelica.Mechanics.MultiBody.Parts.Body for each link

  Modelica.Mechanics.MultiBody.Parts.Body link1(
    r_CM={0,0,0},
    m=1.000000,
    I_11=0.010000,
    I_22=0.010000,
    I_33=0.010000,
    I_21=0.000000,
    I_32=0.000000,
    I_31=0.000000) annotation();

  Modelica.Mechanics.MultiBody.Parts.Body link2(
    r_CM={1,0,0},
    m=1.000000,
    I_11=0.010000,
    I_22=0.010000,
    I_33=0.010000,
    I_21=0.000000,
    I_32=0.000000,
    I_31=0.000000) annotation();


equation







    connect(world.frame_b, fixed_fixedRotation.frame_a) annotation();




    connect(fixed_fixedRotation.frame_b, link1.frame_a) annotation();



    q_joint_1_2 = joint_1_2.phi;
    qd_joint_1_2 = der(q_joint_1_2);
    qdd_joint_1_2 = der(qd_joint_1_2);
    tau_joint_1_2 = joint_1_2.tau;






    connect(link1.frame_a, joint_1_2_fixedRotation.frame_a) annotation();


    connect(joint_1_2_fixedRotation.frame_b, joint_1_2.frame_a) annotation();
    connect(joint_1_2.frame_b, link2.frame_a) annotation();
    connect(joint_1_2.axis, axis_joint_1_2) annotation();




annotation (
    Documentation(info="<html>
<p>
This model contains the mechanical components of a SDF
multibody system.
</p>
</html>"),
Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={ {-200,-200},{200,200} }), graphics={
        Rectangle(
          extent={ {-200,200},{200,-200} },
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid)
        ,Text(extent={ {-200, -150},{-140, -190 } }, textString="joint_1_2", lineColor={0,0,255})}),
    Diagram(coordinateSystem(
        preserveAspectRatio=true,
extent={ {-200,-200},{200,200} })));

end twoLinks;
~~~

Note: the additional empty lines in the generated modelica model are a known issue, see https://github.com/robotology-playground/sdf-modelica/issues/5 .

This model is render in [OpenModelica](https://openmodelica.org/) as:

![twoLinks model](https://user-images.githubusercontent.com/1857049/40165638-c92b527c-59bc-11e8-94a3-6253e3fe5417.png)

The flange indicated with the `joint_1_2` label is a classical Modelica acausal flange of type [`Modelica.Mechanics.Rotational.Interfaces.Flange_a`](https://doc.modelica.org/help/Modelica_Mechanics_Rotational_Interfaces.html#Modelica.Mechanics.Rotational.Interfaces.Flange_a), so the model can be interfaced with arbitrary Modelica models.  

## Naming
Due to the Modelica naming conventions, the URDF or SDF file that is translated should contain model name, link names and joint names with only alphanumeric or underscore characters.
See also Modelica language specification Appendix B https://www.modelica.org/documents.
