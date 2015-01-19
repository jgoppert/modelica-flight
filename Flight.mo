package Flight
  import SI = Modelica.SIunits;
  import NonSI = Modelica.SIunits.Conversions.NonSIunits;
  import Modelica.SIunits.Conversions.*;

  model MotorProp
    parameter SI.Time time_constant = 0.1 "motor time constant";
    parameter SI.Force thrust_max = 10 "motor maximum thrust";
    parameter Real thrust_to_torque = 0 "thrust to torque gain";
    parameter Integer rotation_direction = 1 "rotation direction (CW=1, CCW=-1)";
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(Placement(visible = true, transformation(origin = {100, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -2.55351e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-102, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 3.10862e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve) annotation(Placement(visible = true, transformation(origin = {78, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.Multiplex3 multiplex31 annotation(Placement(visible = true, transformation(origin = {32, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.Multiplex3 multiplex32 annotation(Placement(visible = true, transformation(origin = {30, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-8, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = -thrust_max) annotation(Placement(visible = true, transformation(origin = {-74, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferfunction1(b = {1}, a = {time_constant, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = 0) annotation(Placement(visible = true, transformation(origin = {-42, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = thrust_to_torque * rotation_direction) annotation(Placement(visible = true, transformation(origin = {-36, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(gain1.u, transferfunction1.y) annotation(Line(points = {{-48, 28}, {-72, 28}, {-72, -18}, {-18, -18}, {-18, -36}, {-30, -36}, {-30, -36}}, color = {0, 0, 127}));
    connect(multiplex32.u3[1], gain1.y) annotation(Line(points = {{18, 27}, {-25, 27}, {-25, 28}}, color = {0, 0, 127}));
    connect(gain3.y, transferfunction1.u) annotation(Line(points = {{-63, -36}, {-54, -36}}, color = {0, 0, 127}));
    connect(transferfunction1.y, multiplex31.u3[1]) annotation(Line(points = {{-31, -36}, {10, -36}, {10, -14}, {18, -14}}, color = {0, 0, 127}));
    connect(u, gain3.u) annotation(Line(points = {{-102, -36}, {-86, -36}, {-86, -36}, {-86, -36}}, color = {0, 0, 127}));
    connect(multiplex32.u1[1], const.y) annotation(Line(points = {{18, 41}, {6, 41}, {6, 0}, {3, 0}}, color = {0, 0, 127}));
    connect(multiplex32.u2[1], const.y) annotation(Line(points = {{18, 34}, {6, 34}, {6, 0}, {3, 0}}, color = {0, 0, 127}));
    connect(multiplex31.u2[1], const.y) annotation(Line(points = {{20, -8}, {3.5, -8}, {3.5, 0}, {3, 0}}, color = {0, 0, 127}));
    connect(const.y, multiplex31.u1[1]) annotation(Line(points = {{3, 0}, {5.5, 0}, {5.5, -2}, {18, -2}}, color = {0, 0, 127}));
    connect(multiplex32.y, forceAndTorque.torque) annotation(Line(points = {{41, 34}, {64, 34}, {64, 34}, {64, 34}}, color = {0, 0, 127}));
    connect(multiplex31.y, forceAndTorque.force) annotation(Line(points = {{43, -8}, {47.5, -8}, {47.5, -8}, {50, -8}, {50, 22}, {64, 22}, {64, 22}, {64, 22}, {64, 22}, {64, 22}, {64, 22}}, color = {0, 0, 127}));
    connect(forceAndTorque.frame_resolve, forceAndTorque.frame_b) annotation(Line(points = {{78, 38}, {88, 38}, {88, 28}, {88, 28}, {88, 28}}, color = {95, 95, 95}));
    connect(forceAndTorque.frame_b, frame_b) annotation(Line(points = {{88, 28}, {100, 28}, {100, 34}}, color = {95, 95, 95}));
    annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Ellipse(origin = {-52, 42}, extent = {{-40, 4}, {40, -4}}, endAngle = 360), Ellipse(origin = {50, 42}, extent = {{-40, 4}, {40, -4}}, endAngle = 360), Rectangle(origin = {0, 30}, extent = {{-4, 10}, {4, -32}}), Rectangle(origin = {1, -35}, extent = {{-21, 29}, {21, -29}}), Rectangle(origin = {1, -1}, extent = {{-101, 101}, {99, -99}}), Text(origin = {0, 121}, lineColor = {0, 0, 255}, extent = {{-140, 19}, {140, -19}}, textString = "%name")}));
  end MotorProp;

  model Quad
    parameter SI.Mass mass = 1 "mass";
    parameter SI.Time time_constant = 0.1 "motor time constant";
    parameter SI.Force thrust_max = 10 "motor maximum thrust";
    parameter Real thrust_to_torque = 0.1 "thrust to torque gain";
    parameter SI.Length arm_lengths[4] = {0.2, 0.2, 0.2, 0.2} "length of arms";
    parameter NonSI.Angle_deg arm_angles[4] = {0, 90, 180, 270} "rotation of arms";
    parameter SI.Inertia J_x = 1, J_y = 1, J_z = 1, J_xz = 0 "inertia terms";
    Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedrotation1(r = {arm_lengths[1], 0, 0}, n = {0, 0, 1}, angle = -arm_angles[1], rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis) annotation(Placement(visible = true, transformation(origin = {10, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flight.MotorProp motorprop1(time_constant = time_constant, thrust_max = thrust_max, thrust_to_torque = thrust_to_torque) annotation(Placement(visible = true, transformation(origin = {-20, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flight.MotorProp motorprop2(rotation_direction = -1, time_constant = time_constant, thrust_max = thrust_max, thrust_to_torque = thrust_to_torque) annotation(Placement(visible = true, transformation(origin = {-20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedrotation2(animation = true, r = {arm_lengths[2], 0, 0}, n = {0, 0, 1}, angle = -arm_angles[2]) annotation(Placement(visible = true, transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedrotation3(r = {arm_lengths[3], 0, 0}, n = {0, 0, 1}, angle = -arm_angles[3]) annotation(Placement(visible = true, transformation(origin = {10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedrotation4(r = {arm_lengths[4], 0, 0}, angle = -arm_angles[4], n = {0, 0, 1}) annotation(Placement(visible = true, transformation(origin = {10, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flight.MotorProp motorprop4(rotation_direction = -1, time_constant = time_constant, thrust_max = thrust_max, thrust_to_torque = thrust_to_torque) annotation(Placement(visible = true, transformation(origin = {-20, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Routing.DeMultiplex4 demultiplex41 annotation(Placement(visible = true, transformation(origin = {-72, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(Placement(visible = true, transformation(origin = {100, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -3.33067e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Flight.MotorProp motorprop3(rotation_direction = 1, time_constant = time_constant, thrust_max = thrust_max, thrust_to_torque = thrust_to_torque) annotation(Placement(visible = true, transformation(origin = {-20, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body1(r_CM = {0, 0, 0}, m = mass, r_0(each fixed = true), v_0(each fixed = true), w_0_fixed = true, angles_fixed = true, sequence_start = {1, 2, 3}, sequence_angleStates = {1, 2, 3}, I_11 = J_x, I_22 = J_y, I_33 = J_z, I_31 = J_xz, enforceStates = true, z_0_fixed = true, useQuaternions = true, w_a(each fixed = true), z_a(each fixed = true)) annotation(Placement(visible = true, transformation(origin = {70, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u[4] annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    connect(fixedrotation1.frame_b, body1.frame_a) annotation(Line(points = {{20, 80}, {40, 80}, {40, 20}, {60, 20}}, color = {95, 95, 95}));
    connect(fixedrotation4.frame_b, body1.frame_a) annotation(Line(points = {{20, -10}, {40, -10}, {40, 20}, {60, 20}}, color = {95, 95, 95}));
    connect(fixedrotation3.frame_b, body1.frame_a) annotation(Line(points = {{20, 20}, {60, 20}}, color = {95, 95, 95}));
    connect(fixedrotation2.frame_b, body1.frame_a) annotation(Line(points = {{20, 50}, {40, 50}, {40, 20}, {60, 20}}, color = {95, 95, 95}));
    connect(body1.frame_a, frame_b) annotation(Line(points = {{60, 20}, {60, -2}, {100, -2}}, color = {95, 95, 95}));
    connect(demultiplex41.y3[1], motorprop3.u) annotation(Line(points = {{-61, -3}, {-46, -3}, {-46, 20}, {-30, 20}}, color = {0, 0, 127}));
    connect(motorprop3.frame_b, fixedrotation3.frame_a) annotation(Line(points = {{-10, 20}, {0, 20}}, color = {95, 95, 95}));
    connect(demultiplex41.y2[1], motorprop2.u) annotation(Line(points = {{-61, 3}, {-50, 3}, {-50, 50}, {-30, 50}}, color = {0, 0, 127}));
    connect(demultiplex41.y4[1], motorprop4.u) annotation(Line(points = {{-61, -9}, {-30, -9}, {-30, -10}, {-30, -10}}, color = {0, 0, 127}));
    connect(demultiplex41.y1[1], motorprop1.u) annotation(Line(points = {{-61, 9}, {-54, 9}, {-54, 80}, {-32, 80}, {-32, 80}}, color = {0, 0, 127}));
    connect(u, demultiplex41.u) annotation(Line(points = {{-100, 0}, {-84, 0}, {-84, 0}, {-84, 0}}, color = {0, 0, 127}));
    connect(motorprop4.frame_b, fixedrotation4.frame_a) annotation(Line(points = {{-10, -10}, {0, -10}, {0, -10}, {0, -10}}, color = {95, 95, 95}));
    connect(motorprop2.frame_b, fixedrotation2.frame_a) annotation(Line(points = {{-10, 50}, {0, 50}, {0, 50}, {0, 50}}, color = {95, 95, 95}));
    connect(motorprop1.frame_b, fixedrotation1.frame_a) annotation(Line(points = {{-10, 80}, {0, 80}, {0, 80}, {0, 80}}, color = {95, 95, 95}));
    annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Ellipse(origin = {0, 60}, extent = {{-26, 24}, {26, -24}}, endAngle = 360), Ellipse(origin = {60, 0}, extent = {{-26, 24}, {26, -24}}, endAngle = 360), Ellipse(origin = {0, -60}, extent = {{-26, 24}, {26, -24}}, endAngle = 360), Ellipse(origin = {-60, 0}, extent = {{-26, 24}, {26, -24}}, endAngle = 360), Rectangle(origin = {0, -1}, extent = {{-4, 67}, {4, -67}}), Rectangle(extent = {{-64, 4}, {64, -4}}), Text(origin = {0, 120}, lineColor = {0, 0, 255}, extent = {{-140, 20}, {140, -20}}, textString = "%name"), Rectangle(origin = {-1, -1}, extent = {{-99, 101}, {101, -99}})}));
  end Quad;

  model QuadTest
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, 1}, enableAnimation = false) annotation(Placement(visible = true, transformation(origin = {-80, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flight.SimBus simbus1 annotation(Placement(visible = true, transformation(origin = {66, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {54, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flight.Quad quad1(arm_angles = {-45, 45, 135, -135}) annotation(Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u[4] annotation(Placement(visible = true, transformation(origin = {-78, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-78, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add add1[4] annotation(Placement(visible = true, transformation(origin = {-32, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flight.QuadController quadcontroller1 annotation(Placement(visible = true, transformation(origin = {-78, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y[3] annotation(Placement(visible = true, transformation(origin = {80, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {38, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absolutesensor1(get_r = true, get_v = true, get_a = true, get_w = true, get_z = true, get_angles = true, sequence = {1, 2, 3}) annotation(Placement(visible = true, transformation(origin = {64, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(absolutesensor1.r, y) annotation(Line(points = {{54, 19}, {48, 19}, {48, -26}, {72, -26}, {72, -26}}, color = {0, 0, 127}));
    connect(absolutesensor1.frame_resolve, absolutesensor1.frame_a) annotation(Line(points = {{74, 30}, {84, 30}, {84, 51}, {54, 51}, {54, 33}}, color = {95, 95, 95}));
    connect(absolutesensor1.z, simbus1.z) annotation(Line(points = {{74, 19}, {74, 11.5}, {66, 11.5}, {66, -2}}, color = {0, 0, 127}));
    connect(absolutesensor1.w, simbus1.w) annotation(Line(points = {{70, 19}, {70, 12.5}, {66, 12.5}, {66, -2}}, color = {0, 0, 127}));
    connect(absolutesensor1.angles, simbus1.angles) annotation(Line(points = {{66, 19}, {66, -2}}, color = {0, 0, 127}));
    connect(absolutesensor1.a, simbus1.a) annotation(Line(points = {{62, 19}, {62, 8.5}, {66, 8.5}, {66, -2}}, color = {0, 0, 127}));
    connect(absolutesensor1.v, simbus1.v) annotation(Line(points = {{58, 19}, {58, 7.5}, {66, 7.5}, {66, -2}}, color = {0, 0, 127}));
    connect(absolutesensor1.r, simbus1.r) annotation(Line(points = {{54, 19}, {54, 7.5}, {66, 7.5}, {66, -2}}, color = {0, 0, 127}));
    connect(absolutesensor1.frame_a, quad1.frame_b) annotation(Line(points = {{54, 30}, {10, 30}}, color = {95, 95, 95}));
    connect(u, add1.u2) annotation(Line(points = {{-78, -18}, {-54, -18}, {-54, 24}, {-44, 24}, {-44, 24}}, color = {0, 0, 127}));
    connect(quadcontroller1.y, add1.u1) annotation(Line(points = {{-68, 36}, {-46, 36}, {-46, 38}, {-46, 38}}, color = {0, 0, 127}));
    connect(add1.y, quad1.u) annotation(Line(points = {{-21, 30}, {-12, 30}, {-12, 30}, {-12, 30}}, color = {0, 0, 127}));
  initial equation
    // can only specify 4 conditions, since 4 unknown motor inputs
    absolutesensor1.a[3] = 0;
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02));
  end QuadTest;

  expandable connector SimBus "Simulation bus that is adapted to the signals connected to it"
    extends Modelica.Icons.SignalBus;
    annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}));
  end SimBus;

  model BusTest "Demonstrates the usage of a signal bus"
    extends Modelica.Icons.Example;
  public
    Modelica.Blocks.Sources.IntegerStep integerStep(height = 1, offset = 2, startTime = 0.5) annotation(Placement(transformation(extent = {{-60, -40}, {-40, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.BooleanStep booleanStep(startTime = 0.5) annotation(Placement(transformation(extent = {{-58, 0}, {-38, 20}}, rotation = 0)));
    Modelica.Blocks.Sources.Sine sine(freqHz = 1) annotation(Placement(transformation(extent = {{-60, 40}, {-40, 60}}, rotation = 0)));
    Modelica.Blocks.Examples.BusUsage_Utilities.Part part annotation(Placement(transformation(extent = {{-60, -80}, {-40, -60}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain(k = 1) annotation(Placement(transformation(extent = {{-40, 70}, {-60, 90}}, rotation = 0)));
  protected
    BusUsage_Utilities.Interfaces.ControlBus controlBus annotation(Placement(visible = true, transformation(origin = {74, 10}, extent = {{-20, 20}, {20, -20}}, rotation = 90)));
  equation
    connect(part.subControlBus, controlBus.subControlBus) annotation(Line(points = {{-40, -70}, {74, -70}, {74, 10}}, color = {255, 204, 51}, thickness = 0.5));
    connect(integerStep.y, controlBus.integerSignal) annotation(Line(points = {{-39, -30}, {0, -30}, {0, 10}, {74, 10}}, color = {255, 127, 0}));
    connect(booleanStep.y, controlBus.booleanSignal) annotation(Line(points = {{-37, 10}, {74, 10}}, color = {255, 0, 255}));
    connect(gain.u, controlBus.sineSignal) annotation(Line(points = {{-38, 80}, {74, 80}, {74, 10}}, color = {0, 0, 127}));
    connect(sine.y, controlBus.sineSignal) annotation(Line(points = {{-39, 50}, {6, 50}, {6, 10}, {74, 10}}, color = {0, 0, 127}));
    annotation(Documentation(info = "<html>
<p><b>Signal bus concept</b></p>
<p>
In technical systems, such as vehicles, robots or satellites, many signals
are exchanged between components. In a simulation system, these signals
are usually modelled by signal connections of input/output blocks.
Unfortunately, the signal connection structure may become very complicated,
especially for hierarchical models.
</p>

<p>
The same is also true for real technical systems. To reduce complexity
and get higher flexibility, many technical systems use data buses to
exchange data between components. For the same reasons, it is often better
to use a \"signal bus\" concept also in a Modelica model. This is demonstrated
at hand of this model (Modelica.Blocks.Examples.BusUsage):
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/BusUsage.png\"
     alt=\"BusUsage.png\">

<ul>
<li> Connector instance \"controlBus\" is a hierarchical connector that is
     used to exchange signals between different components. It is
     defined as \"expandable connector\" in order that <b>no</b> central definition
     of the connector is needed but is automatically constructed by the
     signals connected to it (see also Modelica specification 2.2.1).</li>
<li> Input/output signals can be directly connected to the \"controlBus\".</li>
<li> A component, such as \"part\", can be directly connected to the \"controlBus\",
     provided it has also a bus connector, or the \"part\" connector is a
     sub-connector contained in the \"controlBus\". </li>
</ul>

<p>
The control and sub-control bus icons are provided within Modelica.Icons.
In <a href=\"modelica://Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces\">Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces</a>
the buses for this example are defined. Both the \"ControlBus\" and the \"SubControlBus\" are
<b>expandable</b> connectors that do not define any variable. For example,
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces.ControlBus#text\">Interfaces.ControlBus</a>
is defined as:
</p>
<pre>  <b>expandable connector</b> ControlBus
      <b>extends</b> Modelica.Icons.ControlBus;
      <b>annotation</b> ();
  <b>end</b> ControlBus;
</pre>
<p>
Note, the \"annotation\" in the connector is important since the color
and thickness of a connector line are taken from the first
line element in the icon annotation of a connector class. Above, a small rectangle in the
color of the bus is defined (and therefore this rectangle is not
visible). As a result, when connecting from an instance of this
connector to another connector instance, the connecting line has
the color of the \"ControlBus\" with double width (due to \"thickness=0.5\").
</p>

<p>
An <b>expandable</b> connector is a connector where the content of the connector
is constructed by the variables connected to instances of this connector.
For example, if \"sine.y\" is connected to the \"controlBus\", the following
menu pops-up in Dymola:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/BusUsage2.png\"
     alt=\"BusUsage2.png\">

<p>
The \"Add variable/New name\" field allows the user to define the name of the signal on
the \"controlBus\". When typing \"realSignal1\" as \"New name\", a connection of the form:
</p>

<pre>     <b>connect</b>(sine.y, controlBus.realSignal1)
</pre>

<p>
is generated and the \"controlBus\" contains the new signal \"realSignal1\". Modelica tools
may give more support in order to list potential signals for a connection.
For example, in Dymola all variables are listed in the menu that are contained in
connectors which are derived by inheritance from \"controlBus\". Therefore, in
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces\">BusUsage_Utilities.Interfaces</a>
the expected implementation of the \"ControlBus\" and of the \"SubControlBus\" are given.
For example \"Internal.ControlBus\" is defined as:
</p>

<pre>  <b>expandable connector</b> StandardControlBus
    <b>extends</b> BusUsage_Utilities.Interfaces.ControlBus;

    <b>import</b> SI = Modelica.SIunits;
    SI.AngularVelocity    realSignal1   \"First Real signal\";
    SI.Velocity           realSignal2   \"Second Real signal\";
    Integer               integerSignal \"Integer signal\";
    Boolean               booleanSignal \"Boolean signal\";
    StandardSubControlBus subControlBus \"Combined signal\";
  <b>end</b> StandardControlBus;
</pre>

<p>
Consequently, when connecting now from \"sine.y\" to \"controlBus\", the menu
looks differently:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/BusUsage3.png\"
     alt=\"BusUsage3.png\">

<p>
Note, even if the signals from \"Internal.StandardControlBus\" are listed, these are
just potential signals. The user might still add different signal names.
</p>

</html>"), experiment(StopTime = 2));
  end BusTest;

  package BusUsage_Utilities "Utility models and connectors for example Modelica.Blocks.Examples.BusUsage"
    extends Modelica.Icons.UtilitiesPackage;

    package Interfaces "Interfaces specialised for this example"
      extends Modelica.Icons.InterfacesPackage;

      expandable connector ControlBus "Control bus that is adapted to the signals connected to it"
        extends Modelica.Icons.SignalBus;
        import SI = Modelica.SIunits;
        SI.AngularVelocity realSignal1 "First Real signal (angular velocity)" annotation(HideResult = false);
        SI.Velocity realSignal2 "Second Real signal" annotation(HideResult = false);
        Integer integerSignal "Integer signal" annotation(HideResult = false);
        Boolean booleanSignal "Boolean signal" annotation(HideResult = false);
        SubControlBus subControlBus "Combined signal" annotation(HideResult = false);
        annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}), Documentation(info = "<html>
<p>
This connector defines the \"expandable connector\" ControlBus that
is used as bus in the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
Note, this connector contains \"default\" signals that might be utilized
in a connection (the input/output causalities of the signals
are determined from the connections to this bus).
</p>
</html>"));
      end ControlBus;

      expandable connector SubControlBus "Sub-control bus that is adapted to the signals connected to it"
        extends Modelica.Icons.SignalSubBus;
        Real myRealSignal annotation(HideResult = false);
        Boolean myBooleanSignal annotation(HideResult = false);
        annotation(defaultComponentPrefixes = "protected", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}), Documentation(info = "<html>
<p>
This connector defines the \"expandable connector\" SubControlBus that
is used as sub-bus in the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
Note, this is an expandable connector which has a \"default\" set of
signals (the input/output causalities of the signals are
determined from the connections to this bus).
</p>
</html>"));
      end SubControlBus;
      annotation(Documentation(info = "<html>
<p>
This package contains the bus definitions needed for the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
</p>
</html>"));
    end Interfaces;

    model Part "Component with sub-control bus"
      Interfaces.SubControlBus subControlBus annotation(Placement(transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
      Modelica.Blocks.Sources.RealExpression realExpression(y = time) annotation(Placement(transformation(extent = {{-6, 0}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 0.5) annotation(Placement(transformation(extent = {{-6, -30}, {20, -10}}, rotation = 0)));
    equation
      connect(realExpression.y, subControlBus.myRealSignal) annotation(Line(points = {{21.3, 10}, {88, 10}, {88, 6}, {98, 6}, {98, 0}, {100, 0}}, color = {0, 0, 127}, smooth = Smooth.None));
      connect(booleanExpression.y, subControlBus.myBooleanSignal) annotation(Line(points = {{21.3, -20}, {60, -20}, {60, 0}, {100, 0}}, color = {255, 0, 255}, smooth = Smooth.None));
      annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 60}, {100, -60}}, fillColor = {159, 159, 223}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 127}), Text(extent = {{-106, 124}, {114, 68}}, textString = "%name", lineColor = {0, 0, 255})}), Documentation(info = "<html>
<p>
This model is used to demonstrate the bus usage in example
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a>.
</p>
</html>"));
    end Part;
    annotation(Documentation(info = "<html>
<p>
This package contains utility models and bus definitions needed for the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
</p>
</html>"));
  end BusUsage_Utilities;

  model QuadController
    parameter Real u0[4](each fixed = false, start = {0.2, 0.2, 0.2, 0.2}, min = {0, 0, 0, 0}, max = {1, 1, 1, 1});
    Modelica.Blocks.Interfaces.RealOutput y[4] annotation(Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 1.55431e-14}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    connect(u0, y);
    annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-35, 88}, lineColor = {0, 0, 255}, extent = {{-105, 52}, {175, 12}}, textString = "%name"), Rectangle(origin = {3, 15}, extent = {{-43, 53}, {43, -53}})}));
  end QuadController;
end Flight;