package Flight
  package UserGuide
    annotation(Documentation(info = "<html>
<p>
Licensed by James Goppert under the Modelica License 2<br />
Copyright &copy 2015, James Goppert.
</p>

<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"http://www.modelica.org/licenses/ModelicaLicense2\"> http://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p></html>"));
  end UserGuide;

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
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absolutesensor1(get_r = true, get_v = true, get_a = true, get_w = true, get_z = true, get_angles = true, sequence = {1, 2, 3}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve) annotation(Placement(visible = true, transformation(origin = {64, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
    absolutesensor1.a[3] = 0;
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02));
  end QuadTest;

  expandable connector SimBus "Simulation bus that is adapted to the signals connected to it"
    extends Modelica.Icons.SignalBus;
    annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}));
  end SimBus;

  model QuadController
    parameter Real u0[4](each fixed = false, start = {0.2, 0.2, 0.2, 0.2}, min = {0, 0, 0, 0}, max = {1, 1, 1, 1});
    Modelica.Blocks.Interfaces.RealOutput y[4] annotation(Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 1.55431e-14}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    connect(u0, y);
    annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-35, 88}, lineColor = {0, 0, 255}, extent = {{-105, 52}, {175, 12}}, textString = "%name"), Rectangle(origin = {3, 15}, extent = {{-43, 53}, {43, -53}})}));
  end QuadController;
end Flight;