within SimpleZoneModel;
package BuildingZones "Package with the simple hourly zone model per ISO 13790"
  extends Modelica.Icons.Package;
  model ISO13790 "Room model using RC-network according to ISO 13790"
    parameter Real n_ve(unit="1/h") = 0.5 "Air change rate"
        annotation(Evaluate=true, Dialog(tab = "Ventilation"));
    parameter Modelica.SIunits.CoefficientOfHeatTransfer U_win = 0.6
      "Coefficient of heat transfer for windows"
        annotation(Evaluate=true, Dialog(tab = "Transmission", group = "Windows"));
    parameter Modelica.SIunits.CoefficientOfHeatTransfer U_opaque = 0.2
      "Coefficient of heat transfer for opaque walls"
        annotation(Evaluate=true, Dialog(tab = "Transmission", group = "Opaque walls"));
    parameter Modelica.SIunits.Area A_win = 5
      "Sum of effective area of windows."
        annotation(Evaluate=true, Dialog(tab = "Transmission", group = "Windows"));
    parameter Modelica.SIunits.Area A_opaque = 25
      "Sum of effective area of opaque walls."
        annotation(Evaluate=true, Dialog(tab = "Transmission", group = "Opaque walls"));
    parameter Modelica.SIunits.Area A_f = 25 "Net conditioned floor area";
    parameter Real masFac = 3.5 "Mass factor multiplier.  Values from the standard are 2.5 - Very Light, 2.5 - Light,
  2.5 - Medium, 3.0 - Heavy, 3.5 Very Heavy";
    parameter Modelica.SIunits.Area A_m = masFac * A_f "Effective mass area";
    parameter Modelica.SIunits.Area A_t = 4.5 * A_f
      "Total inside surface area of room walls";
    parameter Modelica.SIunits.Volume V_room = A_f * 2.5 "Volume of room";
    parameter Real f_ms = 1 "Calibration factor for H_tr_ms (mass/surface)"
        annotation(Evaluate=true, Dialog(tab = "Transmission", group = "Calibration factors"));
    parameter Real f_is = 1 "Calibration factor for H_tr_is (surface/air)"
        annotation(Evaluate=true, Dialog(tab = "Transmission", group = "Calibration factors"));
    parameter Real buiMasFac = 260000.0 " Mass factor for entire building.  Values from standard are 80000.0 - Very Light,
  110000.0 - Light, 165000.0 - Medium, 260000 - Heavy, 370000 - Very heavy";
    parameter Modelica.SIunits.HeatCapacity C_mass = buiMasFac * A_f
      "Effective mass of building";
    parameter Modelica.SIunits.SpecificHeatCapacity cp_air = 1005
        annotation(Evaluate=true, Dialog(tab = "General", group = "Properties of air"));
    parameter Modelica.SIunits.Density rho_air = 1.293
        annotation(Evaluate=true, Dialog(tab = "General", group = "Properties of air"));
    Modelica.SIunits.Temperature T_surf;
    Modelica.SIunits.HeatFlowRate Qdot_transm;
    Modelica.SIunits.HeatFlowRate Qdot_transm_win;
    Modelica.SIunits.HeatFlowRate Qdot_transm_opaque;
    Modelica.SIunits.HeatFlowRate Qdot_vent;
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor H_vent(G=n_ve*
          V_room*cp_air*rho_air/3600)
      "Heat transfer due to ventilation ISO 13790 p. 39 Eq. 21"
      annotation (Placement(transformation(extent={{-50,50},{-30,70}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor H_tr_as(G=A_t*3.45*
          f_is) "Heat transfer due to transmission. ISO 13790 p. 25 Eq. 9"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,30})));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor H_tr_ms(G=A_m*9.1*
          f_ms) "Heat transfer due to coupling between nodes m & s.  
        ISO 13790 p. 66 Eq. 64"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,-30})));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor H_tr_win(G=A_win*
          U_win) "Heat transfer due to doors, windows, curtain walls, glazed walls.
        IsO 13790-2008 §8.3 Eq. 18"
      annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor H_tr_em(G=1/(1/(
          A_opaque*U_opaque) - 1/H_tr_ms.G)) "Heat transfer from exterior to ambient
        ISO 13790 p. 66 Eq. 63"
      annotation (Placement(transformation(extent={{-50,-70},{-30,-50}})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor C_m(C=C_mass)
      annotation (Placement(transformation(extent={{-10,-80},{10,-100}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow phi_ia
      annotation (Placement(transformation(extent={{50,50},{30,70}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow phi_st
      annotation (Placement(transformation(extent={{50,-10},{30,10}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow phi_m
      annotation (Placement(transformation(extent={{50,-70},{30,-50}})));
    Modelica.Blocks.Sources.RealExpression qIntAir(y=0.5*Qdot_int)
      annotation (Placement(transformation(extent={{100,74},{60,94}})));
    Modelica.Blocks.Sources.RealExpression qSur(y=(1 - A_m/A_t - H_tr_win.G/(9.1*
          A_t))*(0.5*Qdot_int + Qdot_sol))
      annotation (Placement(transformation(extent={{100,14},{60,34}})));
    Modelica.Blocks.Sources.RealExpression qMas(y=A_m/A_t*(0.5*Qdot_int +
          Qdot_sol))
      annotation (Placement(transformation(extent={{100,-46},{60,-26}})));
    Modelica.Blocks.Interfaces.RealInput Qdot_sol
      annotation (Placement(transformation(extent={{-128,68},{-88,108}}),
          iconTransformation(extent={{-124,68},{-96,96}})));
  public
    Modelica.Blocks.Interfaces.RealOutput zonTem
      annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature ambTem
      annotation (Placement(transformation(extent={{-78,-6},{-66,6}})));
    Modelica.Blocks.Interfaces.RealInput Qdot_int
      annotation (Placement(transformation(extent={{-128,24},{-88,64}}),
          iconTransformation(extent={{-124,36},{-96,64}})));
    Modelica.Blocks.Interfaces.RealInput T_amb annotation (Placement(
          transformation(extent={{-128,-20},{-88,20}}), iconTransformation(extent=
             {{-124,-4},{-96,24}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a theta_s;
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a theta_m;
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a theta_air
      annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  equation
    T_surf = theta_s.T;
    zonTem = theta_air.T;
    Qdot_transm = Qdot_transm_win + Qdot_transm_opaque;
    Qdot_transm_win = H_tr_win.port_a.Q_flow;
    Qdot_transm_opaque = H_tr_em.port_a.Q_flow;
    Qdot_vent = H_vent.port_a.Q_flow;
    connect(qIntAir.y, phi_ia.Q_flow)          annotation (Line(
        points={{58,84},{54,84},{54,60},{50,60}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(qSur.y, phi_st.Q_flow)             annotation (Line(
        points={{58,24},{54,24},{54,6.66134e-16},{50,6.66134e-16}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(qMas.y, phi_m.Q_flow)             annotation (Line(
        points={{58,-36},{54,-36},{54,-60},{50,-60}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(ambTem.port, H_tr_win.port_a) annotation (Line(
        points={{-66,-1.88738e-16},{-50,6.10623e-16}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ambTem.port, H_vent.port_a) annotation (Line(
        points={{-66,-1.88738e-16},{-58,6.10623e-16},{-58,60},{-50,60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ambTem.port, H_tr_em.port_a) annotation (Line(
        points={{-66,-1.88738e-16},{-58,6.10623e-16},{-58,-60},{-50,-60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ambTem.T, T_amb) annotation (Line(
        points={{-79.2,-1.55431e-16},{-89.6,-1.55431e-16},{-89.6,1.11022e-15},{
            -108,1.11022e-15}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(phi_st.port, theta_s) annotation (Line(
        points={{30,6.10623e-16},{16,6.10623e-16},{16,5.55112e-16},{5.55112e-16,5.55112e-16}},
        color={191,0,0},
        smooth=Smooth.None));

    connect(H_tr_as.port_a, theta_s) annotation (Line(
        points={{-1.12703e-16,20},{-1.12703e-16,11},{5.55112e-16,11},{5.55112e-16,
            5.55112e-16}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(H_tr_win.port_b, theta_s) annotation (Line(
        points={{-30,6.10623e-16},{-16,6.10623e-16},{-16,5.55112e-16},{5.55112e-16,
            5.55112e-16}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(H_tr_ms.port_b, theta_s) annotation (Line(
        points={{1.1119e-15,-20},{5.55112e-16,-20},{5.55112e-16,5.55112e-16}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(H_tr_ms.port_a, theta_m) annotation (Line(
        points={{-1.12703e-16,-40},{5.55112e-16,-40},{5.55112e-16,-60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(phi_m.port, theta_m) annotation (Line(
        points={{30,-60},{5.55112e-16,-60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(H_tr_em.port_b, theta_m) annotation (Line(
        points={{-30,-60},{5.55112e-16,-60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(C_m.port, theta_m) annotation (Line(
        points={{6.10623e-16,-80},{5.55112e-16,-80},{5.55112e-16,-60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(phi_ia.port, theta_air) annotation (Line(
        points={{30,60},{5.55112e-16,60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(H_vent.port_b, theta_air) annotation (Line(
        points={{-30,60},{5.55112e-16,60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(H_tr_as.port_b, theta_air) annotation (Line(
        points={{1.1119e-15,40},{5.55112e-16,40},{5.55112e-16,60}},
        color={191,0,0},
        smooth=Smooth.None));
    annotation (Diagram(graphics), Icon(graphics={Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-140,160},{160,120}},
            textString="%name",
            lineColor={0,0,255})}));
  end ISO13790;

  model Facades "Calculations for irradiance on surfaces and energy transmittance through the facades:
"
    import Buildings;
    parameter Modelica.SIunits.Area[4] A_win "Rough area of windows."
          annotation(Evaluate=true, Dialog(tab = "General", group = "Window data"));
    parameter Real[4] win_frame "Frame fraction of windows."
          annotation(Evaluate=true, Dialog(tab = "General", group = "Window data"));
    parameter Real[4] g_factor "Energy transmittance of glazings"
          annotation(Evaluate=true, Dialog(tab = "General", group = "Window data"));
    parameter Real shad_fac
      "Shading factor when shading is activated; 0...closed, 1...open"
          annotation(Evaluate=true, Dialog(tab = "General", group = "Shading"));
    parameter Modelica.SIunits.Irradiance Irr_shading
      "Threshold for activating shading"
          annotation(Evaluate=true, Dialog(tab = "General", group = "Shading"));
    parameter Modelica.SIunits.Angle[4] surfaceTilt "Tilt angle of surfaces"
          annotation(Evaluate=true, Dialog(tab = "General", group = "Window directions"));
    parameter Modelica.SIunits.Angle[4] surfaceAzimuth
      "Azimuth angle of surfaces"
          annotation(Evaluate=true, Dialog(tab = "General", group = "Window directions"));
    parameter Real groundReflectance = 0.2 "Ground reflectance";
    parameter Modelica.SIunits.Angle latitude "Latitude of surfaces";
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      annotation (Placement(transformation(extent={{-130,-10},{-110,10}})));
    Modelica.Blocks.Interfaces.RealOutput y
      annotation (Placement(transformation(extent={{116,-10},{136,10}})));
    Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
      HDirTil_north(til=surfaceTilt[1], azi=surfaceAzimuth[1],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,100},{-70,120}})));
    Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil_north(rho=
          groundReflectance,
      til=surfaceTilt[1],
      azi=surfaceAzimuth[1],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,100},{-70,80}})));
    Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
      HDirTil_east(til=surfaceTilt[2], azi=surfaceAzimuth[2],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
    Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil_east(rho=
          groundReflectance,
      til=surfaceTilt[2],
      azi=surfaceAzimuth[2],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,40},{-70,20}})));
    Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
      HDirTil_south(til=surfaceTilt[3], azi=surfaceAzimuth[3],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,-20},{-70,0}})));
    Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil_south(rho=
          groundReflectance,
      til=surfaceTilt[3],
      azi=surfaceAzimuth[3],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,-20},{-70,-40}})));
    Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
      HDirTil_west(til=surfaceTilt[4], azi=surfaceAzimuth[4],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
    Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil_west(rho=
          groundReflectance,
      til=surfaceTilt[4],
      azi=surfaceAzimuth[4],
      lat=latitude)
      annotation (Placement(transformation(extent={{-90,-80},{-70,-100}})));
    Modelica.Blocks.Math.Add Irradiance_north
      annotation (Placement(transformation(extent={{-60,90},{-40,110}})));
    Modelica.Blocks.Math.Gain A_win_north(k=A_win[1])
      annotation (Placement(transformation(extent={{-32,90},{-12,110}})));
    Modelica.Blocks.Math.Gain Frame_north(k=1 - win_frame[1])
      annotation (Placement(transformation(extent={{-4,90},{16,110}})));
    Modelica.Blocks.Math.Gain g_north(k=g_factor[1])
      annotation (Placement(transformation(extent={{24,90},{44,110}})));
    Modelica.Blocks.Math.Sum sum1(nin=4)
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Blocks.Math.Add Irradiance_east
      annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
    Modelica.Blocks.Math.Gain A_win_east(k=A_win[2])
      annotation (Placement(transformation(extent={{-32,30},{-12,50}})));
    Modelica.Blocks.Math.Gain Frame_east(k=1 - win_frame[2])
      annotation (Placement(transformation(extent={{-4,30},{16,50}})));
    Modelica.Blocks.Math.Gain g_east(k=g_factor[2])
      annotation (Placement(transformation(extent={{24,30},{44,50}})));
    Modelica.Blocks.Math.Add Irradiance_south
      annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
    Modelica.Blocks.Math.Gain A_win_south(k=A_win[3])
      annotation (Placement(transformation(extent={{-32,-30},{-12,-10}})));
    Modelica.Blocks.Math.Gain Frame_south(k=1 - win_frame[3])
      annotation (Placement(transformation(extent={{-4,-30},{16,-10}})));
    Modelica.Blocks.Math.Gain g_south(k=g_factor[3])
      annotation (Placement(transformation(extent={{24,-30},{44,-10}})));
    Modelica.Blocks.Math.Add Irradiance_west
      annotation (Placement(transformation(extent={{-60,-90},{-40,-70}})));
    Modelica.Blocks.Math.Gain A_win_west(k=A_win[4])
      annotation (Placement(transformation(extent={{-32,-90},{-12,-70}})));
    Modelica.Blocks.Math.Gain Frame_west(k=1 - win_frame[4])
      annotation (Placement(transformation(extent={{-4,-90},{16,-70}})));
    Modelica.Blocks.Math.Gain g_west(k=g_factor[4])
      annotation (Placement(transformation(extent={{24,-90},{44,-70}})));
    Modelica.Blocks.Math.Product Shading_north
      annotation (Placement(transformation(extent={{58,90},{78,110}})));
    Modelica.Blocks.Math.Product Shading_east
      annotation (Placement(transformation(extent={{58,30},{78,50}})));
    Modelica.Blocks.Math.Product Shading_south
      annotation (Placement(transformation(extent={{58,-30},{78,-10}})));
    Modelica.Blocks.Math.Product Shading_west
      annotation (Placement(transformation(extent={{58,-90},{78,-70}})));
    Modelica.Blocks.Sources.RealExpression Shading_factor_north(y=
          Irradiance_north.y)
      annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
    Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=Irr_shading + 50, uHigh=
          Irr_shading - 50)
      annotation (Placement(transformation(extent={{-32,60},{-12,80}})));
    Modelica.Blocks.Sources.RealExpression Shading_factor_east(y=Irradiance_east.y)
      annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
    Modelica.Blocks.Logical.Hysteresis hysteresis1(uLow=Irr_shading + 50, uHigh=
          Irr_shading - 50)
      annotation (Placement(transformation(extent={{-32,0},{-12,20}})));
    Modelica.Blocks.Sources.RealExpression Shading_factor_south(y=
          Irradiance_south.y)
      annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
    Modelica.Blocks.Logical.Hysteresis hysteresis2(uLow=Irr_shading + 50, uHigh=
          Irr_shading - 50)
      annotation (Placement(transformation(extent={{-32,-60},{-12,-40}})));
    Modelica.Blocks.Sources.RealExpression Shading_factor_west(y=Irradiance_west.y)
      annotation (Placement(transformation(extent={{-60,-120},{-40,-100}})));
    Modelica.Blocks.Logical.Hysteresis hysteresis3(uLow=Irr_shading + 50, uHigh=
          Irr_shading - 50)
      annotation (Placement(transformation(extent={{-32,-120},{-12,-100}})));
    Modelica.Blocks.Logical.Switch switch1
      annotation (Placement(transformation(extent={{24,60},{44,80}})));
    Modelica.Blocks.Sources.Constant const(k=shad_fac)
      annotation (Placement(transformation(extent={{0,74},{10,84}})));
    Modelica.Blocks.Sources.Constant const1(k=1)
      annotation (Placement(transformation(extent={{0,56},{10,66}})));
    Modelica.Blocks.Logical.Switch switch2
      annotation (Placement(transformation(extent={{24,0},{44,20}})));
    Modelica.Blocks.Sources.Constant const2(k=shad_fac)
      annotation (Placement(transformation(extent={{0,14},{10,24}})));
    Modelica.Blocks.Sources.Constant const3(k=1)
      annotation (Placement(transformation(extent={{0,-4},{10,6}})));
    Modelica.Blocks.Logical.Switch switch3
      annotation (Placement(transformation(extent={{24,-60},{44,-40}})));
    Modelica.Blocks.Sources.Constant const4(k=shad_fac)
      annotation (Placement(transformation(extent={{0,-46},{10,-36}})));
    Modelica.Blocks.Sources.Constant const5(k=1)
      annotation (Placement(transformation(extent={{0,-64},{10,-54}})));
    Modelica.Blocks.Logical.Switch switch4
      annotation (Placement(transformation(extent={{24,-120},{44,-100}})));
    Modelica.Blocks.Sources.Constant const6(k=shad_fac)
      annotation (Placement(transformation(extent={{0,-106},{10,-96}})));
    Modelica.Blocks.Sources.Constant const7(k=1)
      annotation (Placement(transformation(extent={{0,-124},{10,-114}})));
  equation
    connect(Irradiance_north.y, A_win_north.u) annotation (Line(
        points={{-39,100},{-34,100}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(A_win_north.y, Frame_north.u) annotation (Line(
        points={{-11,100},{-6,100}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Frame_north.y, g_north.u) annotation (Line(
        points={{17,100},{22,100}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDirTil_north.H, Irradiance_north.u1) annotation (Line(
        points={{-69,110},{-66,110},{-66,106},{-62,106}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDifTil_north.H, Irradiance_north.u2) annotation (Line(
        points={{-69,90},{-66,90},{-66,94},{-62,94}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Irradiance_east.y, A_win_east.u) annotation (Line(
        points={{-39,40},{-34,40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(A_win_east.y, Frame_east.u) annotation (Line(
        points={{-11,40},{-6,40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Frame_east.y, g_east.u) annotation (Line(
        points={{17,40},{22,40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Irradiance_south.y, A_win_south.u) annotation (Line(
        points={{-39,-20},{-34,-20}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(A_win_south.y, Frame_south.u) annotation (Line(
        points={{-11,-20},{-6,-20}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Frame_south.y, g_south.u) annotation (Line(
        points={{17,-20},{22,-20}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Irradiance_west.y, A_win_west.u) annotation (Line(
        points={{-39,-80},{-34,-80}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(A_win_west.y, Frame_west.u) annotation (Line(
        points={{-11,-80},{-6,-80}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Frame_west.y, g_west.u) annotation (Line(
        points={{17,-80},{22,-80}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDirTil_east.H, Irradiance_east.u1) annotation (Line(
        points={{-69,50},{-66,50},{-66,46},{-62,46}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDifTil_east.H, Irradiance_east.u2) annotation (Line(
        points={{-69,30},{-66,30},{-66,34},{-62,34}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDirTil_south.H, Irradiance_south.u1) annotation (Line(
        points={{-69,-10},{-66,-10},{-66,-14},{-62,-14}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDifTil_south.H, Irradiance_south.u2) annotation (Line(
        points={{-69,-30},{-66,-30},{-66,-26},{-62,-26}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDirTil_west.H, Irradiance_west.u1) annotation (Line(
        points={{-69,-70},{-66,-70},{-66,-74},{-62,-74}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDifTil_west.H, Irradiance_west.u2) annotation (Line(
        points={{-69,-90},{-66,-90},{-66,-86},{-62,-86}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HDirTil_north.weaBus, weaBus) annotation (Line(
        points={{-90,110},{-100,110},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDifTil_north.weaBus, weaBus) annotation (Line(
        points={{-90,90},{-100,90},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDirTil_east.weaBus, weaBus) annotation (Line(
        points={{-90,50},{-100,50},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDifTil_east.weaBus, weaBus) annotation (Line(
        points={{-90,30},{-100,30},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDirTil_south.weaBus, weaBus) annotation (Line(
        points={{-90,-10},{-100,-10},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDifTil_south.weaBus, weaBus) annotation (Line(
        points={{-90,-30},{-100,-30},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDirTil_west.weaBus, weaBus) annotation (Line(
        points={{-90,-70},{-100,-70},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDifTil_west.weaBus, weaBus) annotation (Line(
        points={{-90,-90},{-100,-90},{-100,5.55112e-16},{-120,5.55112e-16}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(g_north.y, Shading_north.u1) annotation (Line(
        points={{45,100},{50,100},{50,106},{56,106}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(g_east.y, Shading_east.u1) annotation (Line(
        points={{45,40},{50,40},{50,46},{56,46}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(g_south.y, Shading_south.u1) annotation (Line(
        points={{45,-20},{50,-20},{50,-14},{56,-14}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(g_west.y, Shading_west.u1) annotation (Line(
        points={{45,-80},{50,-80},{50,-74},{56,-74}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_north.y, sum1.u[1]) annotation (Line(
        points={{79,100},{84,100},{84,-1.5},{88,-1.5}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_east.y, sum1.u[2]) annotation (Line(
        points={{79,40},{84,40},{84,-0.5},{88,-0.5}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_south.y, sum1.u[3]) annotation (Line(
        points={{79,-20},{84,-20},{84,0.5},{88,0.5}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_west.y, sum1.u[4]) annotation (Line(
        points={{79,-80},{84,-80},{84,1.5},{88,1.5}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_factor_north.y, hysteresis.u) annotation (Line(
        points={{-39,70},{-34,70}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_factor_east.y, hysteresis1.u) annotation (Line(
        points={{-39,10},{-34,10}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_factor_south.y, hysteresis2.u) annotation (Line(
        points={{-39,-50},{-34,-50}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Shading_factor_west.y, hysteresis3.u) annotation (Line(
        points={{-39,-110},{-34,-110}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(sum1.y, y) annotation (Line(
        points={{111,6.10623e-16},{115.5,6.10623e-16},{115.5,5.55112e-16},{126,
            5.55112e-16}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(switch1.u2, hysteresis.y) annotation (Line(
        points={{22,70},{-11,70}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(const.y, switch1.u1) annotation (Line(
        points={{10.5,79},{16.25,79},{16.25,78},{22,78}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(const1.y, switch1.u3) annotation (Line(
        points={{10.5,61},{15.25,61},{15.25,62},{22,62}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(switch1.y, Shading_north.u2) annotation (Line(
        points={{45,70},{50,70},{50,94},{56,94}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(const2.y, switch2.u1) annotation (Line(
        points={{10.5,19},{16.25,19},{16.25,18},{22,18}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(const3.y, switch2.u3) annotation (Line(
        points={{10.5,1},{15.25,1},{15.25,2},{22,2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(switch2.y, Shading_east.u2) annotation (Line(
        points={{45,10},{50,10},{50,34},{56,34}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(switch2.u2, hysteresis1.y) annotation (Line(
        points={{22,10},{-11,10}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(const4.y, switch3.u1) annotation (Line(
        points={{10.5,-41},{16.25,-41},{16.25,-42},{22,-42}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(const5.y, switch3.u3) annotation (Line(
        points={{10.5,-59},{15.25,-59},{15.25,-58},{22,-58}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(switch3.y, Shading_south.u2) annotation (Line(
        points={{45,-50},{50,-50},{50,-26},{56,-26}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(switch3.u2, hysteresis2.y) annotation (Line(
        points={{22,-50},{-11,-50}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(const6.y, switch4.u1) annotation (Line(
        points={{10.5,-101},{12.25,-101},{12.25,-102},{22,-102}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(const7.y, switch4.u3) annotation (Line(
        points={{10.5,-119},{15.25,-119},{15.25,-118},{22,-118}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(switch4.y, Shading_west.u2) annotation (Line(
        points={{45,-110},{50,-110},{50,-86},{56,-86}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hysteresis3.y, switch4.u2) annotation (Line(
        points={{-11,-110},{22,-110}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-120,
              -120},{120,120}}),
                        graphics), Icon(coordinateSystem(preserveAspectRatio=true,
            extent={{-120,-120},{120,120}}),
                                        graphics={Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,255},
            fillPattern=FillPattern.Solid,
            fillColor={255,255,255}),
          Text(
            extent={{-140,160},{160,120}},
            textString="%name",
            lineColor={0,0,255})}),
            Documentation( info=" <html>
          <p>
          This model distributes the direct and diffuse radiation onto the various surfaces
          of the building.  For each glazed surface the effective glazed area is calculated as:
          <PRE><code> A_win*shad_fac*g_factor*(1 - win_frame)</code> </PRE>with each parameter having four
          values with each vector position corresponding to the following:<br/>
           [1] north<br/>
 [2] east<br/>
 [3] south<br/>
 [4] west<br/>
           </p>
           
           <p>
           The g_factor is a vector of the total solar energy transmittance of the transparente portion of the window.
           win_frame is a vector of the frame area fraction, which is the ratio of the projected frame area to the overall
           rough area of the window.  <code> surfaceTilt </code> is the number of radians each surface makes with the 
           horizontal and <code> surfaceAzimuth </code> is the corresponding azimuthal rotation of each surface.  The
           convention for azimuth is that North = 180, East = -90, South = 0 and West = 90.
           </p>
           <h4>Limitations</h4>
           <p>
           The ISO 13790 standard requires that the glazed and opaque areas be treated as solar sources.  Also, the sources
           must emit energy to the blackbody sky.  This current model does not account for long wave emission and does not
           account for opaque elements as solar radiation sources.  These issues to be corrected in the next release version.
           </p>
           </html>"));
  end Facades;
annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains a dynamic model of a building based on equations for the simple hourly simulation per the ISO 13790 standard.  The model
used is based on an equivalent resistance-capacitance (R-C) model of the normative Annex C.  While the model uses the same equations,
it is not a simple hourly model.  Instead the equations are continuously solved, however, the external heat inputs only update hourly.
</p>
<p> 
The external inputs to the model are the heat flow rate from solar sources, the heat flow rate from internal sources and the internal
set point temperature at each hour.
</p>
<p align=\"center\"><img alt=\"image\" src=\"modelica://SimpleZoneModel/Resources/Images/3r2cnetwork.png\"/>
</p>
<p>
According to the standard, the model is designed to, ''facilitate the use of hourly schedules and produces hourly results, but the results 
for individual hours are not validated and individual hourly values can have large relative errors.  The model
is a simplification of a dynamic simulation, with the following intention:
</p>
<ol>
<li>
clearly specified, limited set of equations, enabling traceability of the calculation process;
</li>
<li>
reduction of the input data as much as possible;
</li>
<li>
unambiguous calculation procedures;
</li>
</ol>
<p>
In addition, the model
<ol>
<li>
makes new development easy by using directly the physical behavior to be implemented;
</li>
<li>
keeps an adequate level of accuracy, especially for room-conditioned buildings where the
thermal dynamic of the room behavior is of high impact.
</li>
</p>

<h4>References</h4>
<p>
ISO/IEC 13790:2008: Energy performance of buildings - Calculation of energy use for space heating
and cooling.  International Organization for Standardization, Geneva, Switzerland.
<A HREF=\"http://www.iso.org/iso/catalogue_detail.htm%3Fcsnumber=41974\">ISO 13790:2008</A>


</html>"));
end BuildingZones;
