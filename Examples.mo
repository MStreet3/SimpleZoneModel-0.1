within SimpleZoneModel;
package Examples
  extends Modelica.Icons.ExamplesPackage;
  model ISO13790Constant
    extends Modelica.Icons.Example;
    BuildingZones.ISO13790 iSO13790_1
      annotation (Placement(transformation(extent={{0,0},{20,20}})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      annotation (Placement(transformation(extent={{-74,54},{-54,74}})));
    Modelica.Blocks.Sources.Constant gai(k=0)
      annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  equation
    connect(weaDat.weaBus, weaBus) annotation (Line(
        points={{-80,90},{-64,90},{-64,64}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(weaBus.TDryBul, iSO13790_1.T_amb) annotation (Line(
        points={{-64,64},{-64,11},{-1,11}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(gai.y, iSO13790_1.Qdot_sol) annotation (Line(
        points={{-19,50},{-10,50},{-10,18.2},{-1,18.2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(gai.y, iSO13790_1.Qdot_int) annotation (Line(
        points={{-19,50},{-10,50},{-10,15},{-1,15}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(graphics));
  end ISO13790Constant;

  model ISO13790Dynamic
    extends Modelica.Icons.Example;
    BuildingZones.ISO13790 iSO13790_1(
      U_opaque=.2,
      A_win=5,
      A_opaque=25) annotation (Placement(transformation(extent={{0,0},{20,20}})));
    BuildingZones.Facades facades(
      shad_fac=1,
      Irr_shading=0,
      latitude=weaDat.lat,
      A_win={5,5,5,5},
      g_factor={0.75,0.75,0.75,0.75},
      surfaceAzimut={3.1415926535898,-1.5707963267949,0,1.5707963267949},
      win_frame={0.1,0.1,0.1,0.1},
      surfaceTilt={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949},
      surfaceAzimuth={3.1415926535898,-1.5707963267949,0,1.5707963267949})
      annotation (Placement(transformation(extent={{-42,18},{-18,42}})));

    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam="modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
    Modelica.Blocks.Sources.Trapezoid intGai(
      rising=3600*2,
      width=3600*4,
      falling=3600*2,
      period=3600*24,
      startTime=3600*7,
      offset=0,
      amplitude=250)
      annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      annotation (Placement(transformation(extent={{-74,54},{-54,74}})));
  equation
    connect(weaDat.weaBus, weaBus) annotation (Line(
        points={{-80,90},{-64,90},{-64,64}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(weaBus, facades.weaBus) annotation (Line(
        points={{-64,64},{-64,30},{-42,30}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(facades.y, iSO13790_1.Qdot_sol) annotation (Line(
        points={{-17.4,30},{-10,30},{-10,18.2},{-1,18.2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(weaBus.TDryBul, iSO13790_1.T_amb) annotation (Line(
        points={{-64,64},{-64,-6},{-14,-6},{-14,11},{-1,11}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(intGai.y, iSO13790_1.Qdot_int) annotation (Line(
        points={{-19,-30},{-6,-30},{-6,15},{-1,15}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(graphics),
    Documentation(info = "<html>
  <p>
  This model implements the simple zone model and applies a weather file
  of TMY3 data from San Francisco for solar data and ambient temperatures.
  The output of the simulation is the internal air temperature, which 
  can then be used to calculate the heat flow rate needed to maintain
  a set point temperature.
  </p>
  </html>"));
  end ISO13790Dynamic;

  model ISO13790Dynamic_ClosedLoopControl
    extends Modelica.Icons.Example;
    parameter Modelica.SIunits.HeatFlowRate maxCooCap = -5000;
    parameter Modelica.SIunits.HeatFlowRate maxHeaCap = 5000;
    BuildingZones.ISO13790 iSO13790_1(
      U_opaque=.2,
      A_opaque=25,
      A_win=sum(facades.A_win))
                   annotation (Placement(transformation(extent={{62,-8},{82,12}})));
    BuildingZones.Facades facades(
      shad_fac=1,
      Irr_shading=0,
      latitude=weaDat.lat,
      A_win={5,5,5,5},
      g_factor={0.75,0.75,0.75,0.75},
      win_frame={0.1,0.1,0.1,0.1},
      surfaceTilt={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949},
      surfaceAzimuth={3.1415926535898,-1.5707963267949,0,1.5707963267949})
      annotation (Placement(transformation(extent={{-42,18},{-18,42}})));

    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          "modelica://SingleZoneModel/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
    Modelica.Blocks.Sources.Trapezoid intGai(
      rising=3600*2,
      width=3600*4,
      falling=3600*2,
      period=3600*24,
      startTime=3600*7,
      offset=0,
      amplitude=250)
      annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      annotation (Placement(transformation(extent={{-74,54},{-54,74}})));
    Modelica.Blocks.Sources.Constant rooSet(k=273.15 + 25)
      annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      Ti=.1,
      yMax=maxHeaCap,
      yMin=maxCooCap)
      annotation (Placement(transformation(extent={{0,60},{20,80}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow qNee
      "Building thermal need in Watts"
      annotation (Placement(transformation(extent={{44,60},{64,80}})));
  equation
    connect(weaDat.weaBus, weaBus) annotation (Line(
        points={{-80,90},{-64,90},{-64,64}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(weaBus, facades.weaBus) annotation (Line(
        points={{-64,64},{-64,30},{-42,30}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(qNee.port, iSO13790_1.theta_air)    annotation (Line(
        points={{64,70},{72,70},{72,8}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(facades.y, iSO13790_1.Qdot_sol) annotation (Line(
        points={{-17.4,30},{61,30},{61,10.2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(PID.y, qNee.Q_flow) annotation (Line(
        points={{21,70},{44,70}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(rooSet.y, PID.u_s) annotation (Line(
        points={{-19,70},{-2,70}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(iSO13790_1.zonTem, PID.u_m) annotation (Line(
        points={{83,-4},{86,-4},{86,-54},{10,-54},{10,58}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(intGai.y, iSO13790_1.Qdot_int) annotation (Line(
        points={{-19,-30},{0,-30},{0,7},{61,7}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(weaBus.TDryBul, iSO13790_1.T_amb) annotation (Line(
        points={{-64,64},{-64,3},{61,3}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    annotation (Diagram(graphics),
    Documentation(info = "<html>
  <p>
  This model implements the simple zone model and applies a weather file
  of TMY3 data from San Francisco for solar data and ambient temperatures.
  The output of the simulation is the internal air temperature, which 
  can then be used to calculate the heat flow rate needed to maintain
  a set point temperature.
  </p>
  
  <p>
  Temperature control is accomplished with a simplified PI controller with a maximum
  value defined by the parameter <code>maxHeaCap</code> and a minimum value
  given by the parameter <code>maxCooCap</code>.  The model is simplified
  by having a single room set point.  The example also shows that the facade
  model output is too large.  This is most likely casued by the implementation
  error mentioned in <a href=\"modelica://SingleZoneModel.BuildingZones.Facades\">SingleZoneModel.BuildingZones.Facades
  </a> documentation.  This bug is listed and noted for a fix in the next build.
  </p>
  </html>"));
  end ISO13790Dynamic_ClosedLoopControl;
end Examples;
