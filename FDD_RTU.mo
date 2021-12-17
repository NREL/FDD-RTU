within ;
package FDD_RTU
  model Five_Zone_Load_Check
   package Medium = Modelica.Media.Air.MoistAir "Medium model";

  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Core(
      redeclare package Medium = Medium,
      zoneName="Core_ZN",
      T_start=298.15,
      nPorts=3) "Thermal zone"
      annotation (Placement(transformation(extent={{-90,62},{-64,90}})));
    inner Buildings.ThermalZones.EnergyPlus.Building building(
      idfName=Modelica.Utilities.Files.loadResource(
          "/mnt/hgfs/Development/RefBldgSmallOfficePost1980_v1.4_7.2_3C_USA_CA_SAN_FRANCISCO.idf"),
      weaName=Modelica.Utilities.Files.loadResource("/home/jling/Dymola/Library/Buildings-v8.0.0/Buildings 8.0.0/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"),
      showWeatherData=true,
      computeWetBulbTemperature=false) "Building level declarations"
      annotation (Placement(transformation(extent={{-138,76},{-118,96}})));

  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_1(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_1",
      T_start=298.15,
      nPorts=3) "Thermal zone"
      annotation (Placement(transformation(extent={{-48,62},{-22,90}})));
  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_2(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_2",
      T_start=298.15,
      nPorts=3) "Thermal zone"
      annotation (Placement(transformation(extent={{-8,62},{18,90}})));
  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_3(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_3",
      T_start=298.15,
      nPorts=3) "Thermal zone"
      annotation (Placement(transformation(extent={{32,62},{58,90}})));
  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_4(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_4",
      T_start=298.15,
      nPorts=3) "Thermal zone"
      annotation (Placement(transformation(extent={{72,62},{98,90}})));
  protected
    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant qGai_flow[3](k={0,0,0})
      "Internal heat gain (computed already in EnergyPlus"
      annotation (Placement(transformation(extent={{-118,18},{-98,38}})));
  equation
    connect(qGai_flow.y,Core. qGai_flow) annotation (Line(points={{-96,28},{-96,83},
            {-91.3,83}},     color={0,0,127}));
    connect(qGai_flow.y,Zn_1. qGai_flow) annotation (Line(points={{-96,28},{-96,56},
            {-54,56},{-54,83},{-49.3,83}},     color={0,0,127}));
    connect(qGai_flow.y,Zn_2. qGai_flow) annotation (Line(points={{-96,28},{-96,56},
            {-16,56},{-16,83},{-9.3,83}},     color={0,0,127}));
    connect(qGai_flow.y,Zn_3. qGai_flow) annotation (Line(points={{-96,28},{-96,56},
            {26,56},{26,83},{30.7,83}},     color={0,0,127}));
    connect(qGai_flow.y,Zn_4. qGai_flow) annotation (Line(points={{-96,28},{-96,56},
            {66,56},{66,83},{70.7,83}},     color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Five_Zone_Load_Check;

  model Five_Zone_RTU
   //package Medium = Modelica.Media.Air.MoistAir "Medium model";
   package Medium = Buildings.Media.Air;

  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Core(
      redeclare package Medium = Medium,
      zoneName="Core_ZN",
      T_start=300.15,
      nPorts=2) "Thermal zone"
      annotation (Placement(transformation(extent={{-90,62},{-64,90}})));
    inner Buildings.ThermalZones.EnergyPlus.Building building(
      idfName=Modelica.Utilities.Files.loadResource("/mnt/hgfs/Development/RefBldgSmallOfficeNew2004_v1.4_7.2_3C_USA_CA_SAN_FRANCISCO.idf"),
      weaName=Modelica.Utilities.Files.loadResource("/home/jling/Dymola/Library/Buildings-v8.0.0/Buildings 8.0.0/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"),
      showWeatherData=true,
      computeWetBulbTemperature=false) "Building level declarations"
      annotation (Placement(transformation(extent={{-174,60},{-154,80}})));

  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_1(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_1",
      T_start=300.15,
      nPorts=2) "Thermal zone"
      annotation (Placement(transformation(extent={{-48,62},{-22,90}})));
  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_2(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_2",
      T_start=300.15,
      nPorts=2) "Thermal zone"
      annotation (Placement(transformation(extent={{-8,62},{18,90}})));
  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_3(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_3",
      T_start=300.15,
      nPorts=2) "Thermal zone"
      annotation (Placement(transformation(extent={{32,62},{58,90}})));
  public
    Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_4(
      redeclare package Medium = Medium,
      zoneName="Perimeter_ZN_4",
      T_start=300.15,
      nPorts=2) "Thermal zone"
      annotation (Placement(transformation(extent={{72,62},{98,90}})));
    VaporCycle.Compressors.Examples.ReciprocatingR134a compressor(
      redeclare package Medium =
          VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
      V_MaxDisplacement(displayUnit="ml") = 27*5/1e6,
      initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
      p_start=2000000,
      h_start=400e3 - 142e3)
      annotation (Placement(transformation(extent={{10,10},{-10,-10}},
          rotation=90,
          origin={100,-56})));
    VaporCycle.HeatExchangers.TwoPhaseAir.Examples.Evaporator
                                                          condenser(
      init_air(
        dp(displayUnit="Pa") = 50,
        T_in=298.15,
        m_flow=1,
        phi_in=0.5,
        phi_outlet=0.2),
      init_wf(
        p_in=2000000,
        p_out=1996000,
        h_in=350e3 - 142e3,
        h_out=280e3 - 142e3,
        m_flow=0.1),
      redeclare package WorkingFluid =
          VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
      n=12,
      channelDensity=fill(80*100, 12),
      n_channels=fill(20*100, 12),
      L=fill(2/12, 12),
      redeclare model HeatTransfer =
          VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.ConstantCoefficient
          (alpha0=2000),
      hx_type=1,
      redeclare model HeatTransfer_air =
          VaporCycle.Pipes.SubComponents.HeatTransfer.SinglePhase.ConstantCoefficient
          (alpha=150),
      condenseMoisture=false,
      initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
      redeclare model Friction =
          VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.DensityProfileFriction
          (
          h0_in=430e3,
          h0_out=270e3,
          mflow0=0.05,
          p0_in=2220000,
          p0_out=2200000),
      redeclare package Air = Modelon.Media.PreDefined.CondensingGases.MoistAir)
                                                                                 annotation (Placement(transformation(extent={{42,-72},
              {22,-92}})));
    VaporCycle.HeatExchangers.TwoPhaseAir.Examples.Evaporator evaporator1(
      initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
      redeclare package WorkingFluid =
          VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
      init_wf(
        m_flow=0.1,
        p_in=1200000,
        p_out=1196000),
      redeclare package Air = Modelon.Media.PreDefined.CondensingGases.MoistAir,
      init_air(
        m_flow=0.4*0.25,
        T_in=297.15,
        T_out=285.15),
      n=12,
      channelDensity=fill(66*100, 12),
      n_channels=fill(20*100, 12),
      redeclare model HeatTransfer =
          VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.ConstantCoefficient
          (alpha0=2500),
      hx_type=2,
      redeclare model HeatTransfer_air =
          VaporCycle.Pipes.SubComponents.HeatTransfer.SinglePhase.ConstantCoefficient
          (alpha=100))
      annotation (Placement(transformation(extent={{16,-28},{36,-8}})));
    VaporCycle.Valves.SimpleTXV
                     expansionValve(
      redeclare package Medium =
          VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
      SuperHeatSetPoint=6.5,
      initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
      mflow_start=0.1,
      positiveFlow=true,
      from_dp=false,
      yMax=0.8,
      yMin(displayUnit="m3/h") = 0.002,
      p_start=1500000,
      h_start=280e3 - 142e3,
      V=1e-08)                 annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=270,
          origin={-2,-54})));
    VaporCycle.Sources.AirFlowSource flowSource1(
      redeclare package Medium =
          Modelon.Media.PreDefined.CondensingGases.MoistAir,
      m_flow=0.943*1.2,
      T=297.15,
      use_flow_in=true,
      use_T_in=true,
      use_phi_in=false)
      annotation (Placement(transformation(extent={{74,-16},{54,4}})));
    VaporCycle.Sources.AirPressureSource pressureSource2(redeclare package
        Medium = Modelon.Media.PreDefined.CondensingGases.MoistAir,
                                                                N=1)
      annotation (Placement(transformation(extent={{-58,-20},{-38,0}})));
    VaporCycle.Sources.AirFlowSource flowSource2(
      redeclare package Medium =
          Modelon.Media.PreDefined.CondensingGases.MoistAir,
      m_flow=1.86*1.2,
      T=297.15,
      phi=0.5,
      use_flow_in=false,
      use_T_in=true,
      use_phi_in=false)
      annotation (Placement(transformation(extent={{-36,-102},{-16,-82}})));
    Buildings.Fluid.Sources.Boundary_pT sin1(redeclare package Medium =
          Buildings.Media.Air, nPorts=1)
      annotation (Placement(transformation(extent={{-108,-104},{-128,-84}})));
    Buildings.Fluid.Sources.MassFlowSource_WeatherData bou(
      redeclare package Medium = Buildings.Media.Air,
      m_flow=1,
      nPorts=1)
      annotation (Placement(transformation(extent={{-166,24},{-150,40}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(
      redeclare package Medium = Buildings.Media.Air,
      allowFlowReversal=false,
      m_flow_nominal=1)
      annotation (Placement(transformation(extent={{10,10},{-10,-10}},
          rotation=90,
          origin={-134,-4})));
    VaporCycle.Sources.AirPressureSource pressureSource1(redeclare package
        Medium =
          Modelon.Media.PreDefined.CondensingGases.MoistAirIce, N=1)
      annotation (Placement(transformation(extent={{-10,-122},{10,-102}})));
    VaporCycle.Sources.AirPressureSource pressureSource3(redeclare package
        Medium = Modelon.Media.PreDefined.CondensingGases.MoistAir,
                                                                N=1)
      annotation (Placement(transformation(extent={{78,-104},{58,-84}})));
    VaporCycle.Sensors.SuperHeatSensor superHeatSensor(redeclare package Medium =
          VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure, allowNegative=false)
      annotation (Placement(transformation(extent={{72,-38},{92,-14}})));
    VaporCycle.Sources.Speed
                  rps
      annotation (Placement(transformation(extent={{138,-66},{118,-46}})));
    Modelica.Blocks.Sources.Ramp speed(
      height=0,
      startTime=1e6,
      duration=0,
      offset=60)
      annotation (Placement(transformation(extent={{176,-62},{164,-50}})));
    Modelica.Fluid.Sources.MassFlowSource_T Core_SF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=true,
      m_flow=0.313*1.2,
      nPorts=1)
      annotation (Placement(transformation(extent={{-118,36},{-104,50}})));
    VaporCycle.Sensors.AirTemperature          airTemperatureSensor(redeclare
        package Medium = Modelon.Media.PreDefined.CondensingGases.MoistAir,
        isCelsius=false)
      annotation (Placement(transformation(extent={{5,6},{-5,-6}},
          rotation=270,
          origin={-9,-6})));
    Modelica.Fluid.Sources.MassFlowSource_T Core_RF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      m_flow=-1,
      nPorts=1)
      annotation (Placement(transformation(extent={{-68,110},{-82,124}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn1_SF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=true,
      m_flow=0.409*1.2,
      nPorts=1) annotation (Placement(transformation(extent={{-68,36},{-54,50}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn1_RF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      m_flow=-1,
      nPorts=1)
      annotation (Placement(transformation(extent={{-4,118},{-18,132}})));
    Modelica.Blocks.Logical.Hysteresis hysteresis(
      uLow=24 + 273 - 0.5,
      uHigh=24 + 273 + 0.5,
      pre_y_start=true)
      annotation (Placement(transformation(extent={{124,14},{144,34}})));
    Modelica.Blocks.Logical.TriggeredTrapezoid triggeredTrapezoid(
      amplitude=-0.47,
      rising(displayUnit="min") = 600,
      offset=0.67)
      annotation (Placement(transformation(extent={{194,-18},{214,2}})));
    Modelica.Blocks.Math.Gain gain(k=-1)
      annotation (Placement(transformation(extent={{-108,116},{-96,128}})));
    Modelica.Blocks.Math.Gain gain1(k=-1)
      annotation (Placement(transformation(extent={{-56,120},{-44,132}})));
    Modelica.Blocks.Logical.Not not1
      annotation (Placement(transformation(extent={{166,8},{176,18}})));
    Modelica.Blocks.Logical.TriggeredTrapezoid triggeredTrapezoid3(
      amplitude=-0.943*1.2*0.5,
      rising(displayUnit="min") = 600,
      offset=0.943*1.2)
      annotation (Placement(transformation(extent={{110,-6},{90,14}})));
    Buildings.Controls.SetPoints.Table Zn_1_Control(table=[23.5 + 273.15,0.409*
          0.05*1.2; 24.5 + 273.15,1.2*0.409*0.25; 25.5 + 273.15,1.2*0.409*0.5])
      annotation (Placement(transformation(extent={{-218,10},{-204,24}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=if time <=
          1.728e+07 + 3600 then 24 + 273.15 else Zn_1.TAir)
      annotation (Placement(transformation(extent={{-180,-42},{-160,-22}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn2_SF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=true,
      m_flow=0.409*1.2,
      nPorts=1) annotation (Placement(transformation(extent={{-10,36},{4,50}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn2_RF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=false,
      m_flow=0.409*1.2,
      nPorts=1)
      annotation (Placement(transformation(extent={{30,118},{16,132}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn3_SF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=true,
      m_flow=0.409*1.2,
      nPorts=1) annotation (Placement(transformation(extent={{24,36},{38,50}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn3_RF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=false,
      m_flow=0.409*1.2,
      nPorts=1)
      annotation (Placement(transformation(extent={{68,118},{54,132}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn4_RF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=false,
      m_flow=0.409*1.2,
      nPorts=1)
      annotation (Placement(transformation(extent={{108,120},{94,134}})));
    Modelica.Fluid.Sources.MassFlowSource_T Zn4_SF(
      redeclare package Medium = Medium,
      use_m_flow_in=true,
      use_T_in=true,
      m_flow=0.409*1.2,
      nPorts=1) annotation (Placement(transformation(extent={{70,36},{84,50}})));
    Buildings.Controls.SetPoints.Table Core_Control(table=[23.5 + 273.15,0.261*
          0.05*1.2; 24.5 + 273.15,1.2*0.261*0.25; 25.5 + 273.15,1.2*0.261*0.3])
      annotation (Placement(transformation(extent={{-218,44},{-204,58}})));
    Buildings.Controls.SetPoints.Table Zn_2_Control(table=[23.5 + 273.15,0.239*
          0.05*1.2; 24.5 + 273.15,1.2*0.239*0.25; 25.5 + 273.15,1.2*0.239*0.3])
      annotation (Placement(transformation(extent={{-218,-16},{-204,-2}})));
    Modelica.Blocks.Math.Gain gain2(k=-1)
      annotation (Placement(transformation(extent={{-2,140},{10,152}})));
    Buildings.Controls.SetPoints.Table Zn_3_Control(table=[23.5 + 273.15,0.163*
          0.05*1.2; 24.5 + 273.15,1.2*0.163*0.25; 25.5 + 273.15,1.2*0.163*0.3])
      annotation (Placement(transformation(extent={{120,90},{134,104}})));
    Modelica.Blocks.Math.Gain gain3(k=-1)
      annotation (Placement(transformation(extent={{80,146},{68,158}})));
    Buildings.Controls.SetPoints.Table Zn_4_Control(table=[23.5 + 273.15,0.258*
          0.05*1.2; 24.5 + 273.15,1.2*0.258*0.25; 25.5 + 273.15,1.2*0.258*0.3])
      annotation (Placement(transformation(extent={{120,66},{134,80}})));
    Modelica.Blocks.Math.Gain gain4(k=-1)
      annotation (Placement(transformation(extent={{132,134},{120,146}})));
  protected
    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant qGai_flow[3](k={0,0,0})
      "Internal heat gain (computed already in EnergyPlus"
      annotation (Placement(transformation(extent={{-112,-56},{-92,-36}})));
  protected
    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant qGai_flow1
                                                                    [3](k={0,0,0})
      "Internal heat gain (computed already in EnergyPlus"
      annotation (Placement(transformation(extent={{-88,-56},{-68,-36}})));
  equation
    connect(qGai_flow.y,Core. qGai_flow) annotation (Line(points={{-90,-46},{
            -90,83},{-91.3,83}},
                             color={0,0,127}));
    connect(qGai_flow.y,Zn_2. qGai_flow) annotation (Line(points={{-90,-46},{
            -90,56},{-16,56},{-16,83},{-9.3,83}},
                                              color={0,0,127}));
    connect(qGai_flow.y,Zn_3. qGai_flow) annotation (Line(points={{-90,-46},{
            -90,56},{26,56},{26,83},{30.7,83}},
                                            color={0,0,127}));
    connect(qGai_flow.y,Zn_4. qGai_flow) annotation (Line(points={{-90,-46},{
            -90,56},{66,56},{66,83},{70.7,83}},
                                            color={0,0,127}));
    connect(compressor.portB, condenser.portA_prim)
      annotation (Line(points={{100,-66},{100,-76},{42,-76}}, color={0,190,0}));
    connect(condenser.portB_prim, expansionValve.portA)
      annotation (Line(points={{22,-76},{-2,-76},{-2,-62}}, color={0,190,0}));
    connect(expansionValve.portB, evaporator1.portA_prim)
      annotation (Line(points={{-2,-46},{-2,-24},{16,-24}}, color={0,190,0}));
    connect(flowSource1.port, evaporator1.portA_sec) annotation (Line(points={{56,
            -6},{44,-6},{44,-12},{36,-12}}, color={85,170,255}));
    connect(building.weaBus, bou.weaBus) annotation (Line(
        points={{-154,70},{-154,32.16},{-166,32.16}},
        color={255,204,51},
        thickness=0.5));
    connect(bou.ports[1], senTem.port_a)
      annotation (Line(points={{-150,32},{-150,22},{-134,22},{-134,6}},
                                                    color={0,127,255}));
    connect(senTem.T, flowSource2.T_in)
      annotation (Line(points={{-123,-4},{-26,-4},{-26,-85}}, color={0,0,127}));
    connect(flowSource2.port, condenser.portA_sec)
      annotation (Line(points={{-18,-92},{2,-92},{2,-88},{22,-88}},
                                                    color={85,170,255}));
    connect(condenser.portB_sec, pressureSource3.port[1]) annotation (Line(points={{42,-88},
            {52,-88},{52,-94},{60,-94}},          color={85,170,255}));
    connect(evaporator1.portB_prim, superHeatSensor.portA)
      annotation (Line(points={{36,-24},{72,-24}}, color={0,190,0}));
    connect(superHeatSensor.portB, compressor.portA)
      annotation (Line(points={{92,-24},{100,-24},{100,-46}}, color={0,190,0}));
    connect(superHeatSensor.y, expansionValve.DeltaT_SH) annotation (Line(points={
            {82,-15},{82,16},{-16,16},{-16,-57.5},{-9.5,-57.5}}, color={0,0,127}));
    connect(compressor.flange, rps.flange)
      annotation (Line(points={{109,-56},{118,-56}}, color={0,0,0}));
    connect(rps.inPort, speed.y)
      annotation (Line(points={{140,-56},{163.4,-56}}, color={0,0,127}));
    connect(senTem.port_b, sin1.ports[1]) annotation (Line(points={{-134,-14},{-134,
            -98},{-128,-98},{-128,-94}}, color={0,127,255}));
    connect(Core.ports[1], Core_SF.ports[1]) annotation (Line(points={{-78.3,62.63},
            {-78.3,43},{-104,43}}, color={0,127,255}));
    connect(Zn_1.ports[1], Zn1_SF.ports[1]) annotation (Line(points={{-36.3,
            62.63},{-36.3,43},{-54,43}},
                                  color={0,127,255}));
    connect(Core_RF.ports[1], Core.ports[2]) annotation (Line(points={{-82,117},{-98,
            117},{-98,60},{-75.7,60},{-75.7,62.63}}, color={0,127,255}));
    connect(airTemperatureSensor.port, evaporator1.portB_sec) annotation (Line(
          points={{-10,-11},{2,-11},{2,-12},{16,-12}}, color={85,170,255}));
    connect(airTemperatureSensor.port, pressureSource2.port[1]) annotation (Line(
          points={{-10,-11},{-26,-11},{-26,-10},{-40,-10}}, color={85,170,255}));
    connect(airTemperatureSensor.outPort, Core_SF.T_in) annotation (Line(points={{
            -15,-6},{-20,-6},{-20,0},{-32,0},{-32,32},{-124,32},{-124,45.8},{-119.4,
            45.8}}, color={0,0,127}));
    connect(airTemperatureSensor.outPort, Zn1_SF.T_in) annotation (Line(points={{-15,
            -6},{-42,-6},{-42,20},{-74,20},{-74,45.8},{-69.4,45.8}}, color={0,0,127}));
    connect(Zn_1.TAir, hysteresis.u) annotation (Line(points={{-21.35,88.6},{-14,88.6},
            {-14,24},{122,24}}, color={0,0,127}));
    connect(gain.y, Core_RF.m_flow_in) annotation (Line(points={{-95.4,122},{-82,122},
            {-82,122.6},{-68,122.6}}, color={0,0,127}));
    connect(gain1.y, Zn1_RF.m_flow_in) annotation (Line(points={{-43.4,126},{
            -34,126},{-34,130.6},{-4,130.6}},
                                      color={0,0,127}));
    connect(triggeredTrapezoid.y, compressor.relativeDisplacement) annotation (
        Line(points={{215,-8},{220,-8},{220,-36},{106,-36},{106,-46}}, color={0,0,
            127}));
    connect(qGai_flow1.y, Zn_1.qGai_flow) annotation (Line(points={{-66,-46},{-64,
            -46},{-64,2},{-60,2},{-60,83},{-49.3,83}}, color={0,0,127}));
    connect(hysteresis.y, not1.u) annotation (Line(points={{145,24},{154,24},{
            154,13},{165,13}}, color={255,0,255}));
    connect(not1.y, triggeredTrapezoid.u) annotation (Line(points={{176.5,13},{
            182,13},{182,-8},{192,-8}}, color={255,0,255}));
    connect(triggeredTrapezoid3.y, flowSource1.m_flow_in)
      annotation (Line(points={{89,4},{69,4},{69,-1}}, color={0,0,127}));
    connect(not1.y, triggeredTrapezoid3.u) annotation (Line(points={{176.5,13},
            {176.5,4},{112,4}}, color={255,0,255}));
    connect(Zn_1.TAir, Zn_1_Control.u) annotation (Line(points={{-21.35,88.6},{
            -21.35,40},{-226,40},{-226,17},{-219.4,17}}, color={0,0,127}));
    connect(Zn_1_Control.y, Zn1_SF.m_flow_in) annotation (Line(points={{-203.3,
            17},{-142,17},{-142,16},{-82,16},{-82,48.6},{-68,48.6}}, color={0,0,
            127}));
    connect(Zn_1_Control.y, gain1.u) annotation (Line(points={{-203.3,17},{-186,
            17},{-186,126},{-57.2,126}}, color={0,0,127}));
    connect(Zn_1.ports[2], Zn1_RF.ports[1]) annotation (Line(points={{-33.7,
            62.63},{-33.7,60},{-18,60},{-18,125}},        color={0,127,255}));
    connect(Zn2_SF.ports[1], Zn_2.ports[1]) annotation (Line(points={{4,43},{6,
            43},{6,62.63},{3.7,62.63}}, color={0,127,255}));
    connect(Zn_2.ports[2], Zn2_RF.ports[1]) annotation (Line(points={{6.3,62.63},
            {6.3,58},{24,58},{24,116},{10,116},{10,125},{16,125}}, color={0,127,
            255}));
    connect(Zn3_SF.ports[1], Zn_3.ports[1]) annotation (Line(points={{38,43},{
            42,43},{42,62.63},{43.7,62.63}}, color={0,127,255}));
    connect(Zn_3.ports[2], Zn3_RF.ports[1]) annotation (Line(points={{46.3,
            62.63},{46.3,48},{60,48},{60,116},{44,116},{44,125},{54,125}},
          color={0,127,255}));
    connect(Zn4_SF.ports[1], Zn_4.ports[1]) annotation (Line(points={{84,43},{
            82,43},{82,44},{85,44},{85,62.63},{83.7,62.63}}, color={0,127,255}));
    connect(Zn_4.ports[2], Zn4_RF.ports[1]) annotation (Line(points={{86.3,
            62.63},{86.3,54},{110,54},{110,116},{80,116},{80,127},{94,127}},
          color={0,127,255}));
    connect(Core.TAir, Core_Control.u) annotation (Line(points={{-63.35,88.6},{
            -60,88.6},{-60,98},{-226,98},{-226,51},{-219.4,51}}, color={0,0,127}));
    connect(Core_Control.y, Core_SF.m_flow_in) annotation (Line(points={{-203.3,
            51},{-161.65,51},{-161.65,48.6},{-118,48.6}}, color={0,0,127}));
    connect(Core_Control.y, gain.u) annotation (Line(points={{-203.3,51},{-192,
            51},{-192,122},{-109.2,122}}, color={0,0,127}));
    connect(Zn_2.TAir, Zn_2_Control.u) annotation (Line(points={{18.65,88.6},{
            18,88.6},{18,108},{-234,108},{-234,-9},{-219.4,-9}}, color={0,0,127}));
    connect(Zn_2_Control.y, Zn2_SF.m_flow_in) annotation (Line(points={{-203.3,
            -9},{-172,-9},{-172,48.6},{-10,48.6}}, color={0,0,127}));
    connect(Zn_2_Control.y, gain2.u) annotation (Line(points={{-203.3,-9},{-196,
            -9},{-196,146},{-3.2,146}}, color={0,0,127}));
    connect(gain2.y, Zn2_RF.m_flow_in) annotation (Line(points={{10.6,146},{24,
            146},{24,130.6},{30,130.6}}, color={0,0,127}));
    connect(Zn_3.TAir, Zn_3_Control.u) annotation (Line(points={{58.65,88.6},{
            64,88.6},{64,97},{118.6,97}}, color={0,0,127}));
    connect(Zn_3_Control.y, Zn3_SF.m_flow_in) annotation (Line(points={{134.7,
            97},{146,97},{146,52},{16,52},{16,48.6},{24,48.6}}, color={0,0,127}));
    connect(Zn_3_Control.y, gain3.u) annotation (Line(points={{134.7,97},{140,
            97},{140,152},{81.2,152}}, color={0,0,127}));
    connect(gain3.y, Zn3_RF.m_flow_in) annotation (Line(points={{67.4,152},{68,
            152},{68,130.6},{68,130.6}}, color={0,0,127}));
    connect(gain4.y, Zn4_RF.m_flow_in) annotation (Line(points={{119.4,140},{
            114,140},{114,132.6},{108,132.6}}, color={0,0,127}));
    connect(Zn_4_Control.y, Zn4_SF.m_flow_in) annotation (Line(points={{134.7,
            73},{142,73},{142,48.6},{70,48.6}}, color={0,0,127}));
    connect(Zn_4.TAir, Zn_4_Control.u) annotation (Line(points={{98.65,88.6},{
            106,88.6},{106,73},{118.6,73}}, color={0,0,127}));
    connect(Zn_4_Control.y, gain4.u) annotation (Line(points={{134.7,73},{156,
            73},{156,140},{133.2,140}}, color={0,0,127}));
    connect(airTemperatureSensor.outPort, Zn2_SF.T_in) annotation (Line(points=
            {{-15,-6},{-18,-6},{-18,45.8},{-11.4,45.8}}, color={0,0,127}));
    connect(airTemperatureSensor.outPort, Zn3_SF.T_in) annotation (Line(points=
            {{-15,-6},{-24,-6},{-24,34},{18,34},{18,45.8},{22.6,45.8}}, color={
            0,0,127}));
    connect(airTemperatureSensor.outPort, Zn4_SF.T_in) annotation (Line(points=
            {{-15,-6},{-20,-6},{-20,30},{60,30},{60,45.8},{68.6,45.8}}, color={
            0,0,127}));
    connect(realExpression.y, flowSource1.T_in) annotation (Line(points={{-159,
            -32},{-76,-32},{-76,-20},{4,-20},{4,6},{64,6},{64,1}}, color={0,0,
            127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StartTime=17280000,
        StopTime=25920000,
        __Dymola_NumberOfIntervals=5000,
        __Dymola_Algorithm="Cvode"));
  end Five_Zone_RTU;
  annotation (uses(
      Buildings(version="8.0.0"),
      Modelica(version="3.2.3"),
      VaporCycle(version="2.7"),
      Modelon(version="3.7")));
end FDD_RTU;
