within ;
model Five_Zone_RTU_chargestudy_simpleTXVv2_v4
 //package Medium = Modelica.Media.Air.MoistAir "Medium model";
 package Medium = Buildings.Media.Air;

public
  Buildings.ThermalZones.EnergyPlus.ThermalZone Core(
    redeclare package Medium = Medium,
    zoneName="Core_ZN",
    T_start=300.15,
    nPorts=3) "Thermal zone"
    annotation (Placement(transformation(extent={{-88,62},{-62,90}})));
  inner Buildings.ThermalZones.EnergyPlus.Building building(
    idfName=Modelica.Utilities.Files.loadResource("/mnt/hgfs/Development/RefBldgSmallOfficeNew2004_v1.4_7.2_3C_USA_CA_SAN_FRANCISCO.idf"),
    epwName=Modelica.Utilities.Files.loadResource(
        "/home/jling/Dymola/Library/Buildings-v8.0.0/Buildings 8.0.0/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.epw"),
    weaName=Modelica.Utilities.Files.loadResource("/home/jling/Dymola/Library/Buildings-v8.0.0/Buildings 8.0.0/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"),
    computeWetBulbTemperature=false) "Building level declarations"
    annotation (Placement(transformation(extent={{-174,60},{-154,80}})));
   // epwName=Modelica.Utilities.Files.loadResource(
     //   "/home/jling/Dymola/Library/Buildings-v8.0.0/Buildings 8.0.0/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.epw"),
   // epwName=Modelica.Utilities.Files.loadResource(
     //   "/home/jling/Dymola/Library/Buildings-v8.0.0/Buildings 8.0.0/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.epw"),

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
    T_start=300.15,
    nPorts=3) "Thermal zone"
    annotation (Placement(transformation(extent={{-8,62},{18,90}})));
public
  Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_3(
    redeclare package Medium = Medium,
    zoneName="Perimeter_ZN_3",
    T_start=300.15,
    nPorts=3) "Thermal zone"
    annotation (Placement(transformation(extent={{32,62},{58,90}})));
public
  Buildings.ThermalZones.EnergyPlus.ThermalZone Zn_4(
    redeclare package Medium = Medium,
    zoneName="Perimeter_ZN_4",
    T_start=300.15,
    nPorts=3) "Thermal zone"
    annotation (Placement(transformation(extent={{72,62},{98,90}})));
  VaporCycle.Compressors.Examples.ReciprocatingR134a compressor(
    redeclare package Medium =
        VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
    V_MaxDisplacement(displayUnit="ml") = 16*5/1e6,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    p_start=1700000,
    h_start=420e3 - 142e3,
    eff_min=0.75)
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=90,
        origin={100,-56})));
  VaporCycle.HeatExchangers.TwoPhaseAir.Examples.Evaporator evaporator1(
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    redeclare package WorkingFluid =
        VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
    init_wf(
      m_flow=0.1,
      p_in=1700000,
      p_out=1690000,
      h_in=300e3 - 142e3),
    redeclare package Air = Modelon.Media.PreDefined.CondensingGases.MoistAir,
    init_air(
      m_flow=0.4*0.25,
      T_in=297.15,
      T_out=285.15),
    n=3,
    channelDensity=fill(66*100, 3),
    n_channels=fill(20*100, 3),
    redeclare model HeatTransfer =
        VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.ConstantCoefficient
        (alpha0=4500),
    hx_type=2,
    redeclare model HeatTransfer_air =
        VaporCycle.Pipes.SubComponents.HeatTransfer.SinglePhase.ConstantCoefficient
        (alpha=100),
    condenseMoisture=true)
    annotation (Placement(transformation(extent={{16,-28},{36,-8}})));
  VaporCycle.Valves.SimpleTXV
                   expansionValve(
    redeclare package Medium =
        VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
    Ti=20,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    mflow_start=0.05,
    positiveFlow=false,
    from_dp=false,
    yMax=1,
    yMin(displayUnit="m3/h") = 0.002,
    p_start=1690000,
    h_start=450e3 - 142e3)   annotation (Placement(transformation(
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
    use_phi_in=true)
    annotation (Placement(transformation(extent={{74,-16},{54,4}})));
  VaporCycle.Sources.AirPressureSource pressureSource2(redeclare package Medium
      =        Modelon.Media.PreDefined.CondensingGases.MoistAir,
                                                              N=1)
    annotation (Placement(transformation(extent={{-58,-20},{-38,0}})));
  VaporCycle.Sources.AirFlowSource flowSource2(
    redeclare package Medium =
        Modelon.Media.PreDefined.CondensingGases.MoistAir,
    m_flow=1.86*1.2,
    T=293.15,
    phi=0.5,
    use_flow_in=false,
    use_T_in=true,
    use_phi_in=true)
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
  VaporCycle.Sources.AirPressureSource pressureSource3(redeclare package Medium
      =        Modelon.Media.PreDefined.CondensingGases.MoistAir, N=1)
    annotation (Placement(transformation(extent={{78,-104},{58,-84}})));
  VaporCycle.Sensors.SuperHeatSensor superHeatSensor(redeclare package Medium
      = VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure)
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
    use_X_in=true,
    m_flow=0.409*1.2,
    nPorts=1) annotation (Placement(transformation(extent={{-68,36},{-54,50}})));
  Modelica.Fluid.Sources.MassFlowSource_T Zn1_RF(
    redeclare package Medium = Medium,
    use_m_flow_in=true,
    m_flow=-1,
    nPorts=1)
    annotation (Placement(transformation(extent={{-4,118},{-18,132}})));
  Modelica.Blocks.Math.Gain gain(k=-1)
    annotation (Placement(transformation(extent={{-108,116},{-96,128}})));
  Modelica.Blocks.Math.Gain gain1(k=-1)
    annotation (Placement(transformation(extent={{-56,120},{-44,132}})));
  Buildings.Controls.SetPoints.Table Zn_1_Control(table=[23 + 273.15,0.409*
        0.2*1.2; 23.45 + 273.15,1.2*0.409*0.2; 24.5 + 273.15,1.2*0.409*0.5;
        25.0 + 273.15,1.2*0.409*0.5; 25.05 + 273.15,1.2*0.409*1])
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
        0.5*1.2; 24.5 + 273.15,1.2*0.261*1; 25.5 + 273.15,1.2*0.261*1.2])
    annotation (Placement(transformation(extent={{-218,44},{-204,58}})));
  Buildings.Controls.SetPoints.Table Zn_2_Control(table=[23.5 + 273.15,0.239*
        0.5*1.2; 24.5 + 273.15,1.2*0.239*1; 25.5 + 273.15,1.2*0.239*1.2])
    annotation (Placement(transformation(extent={{-218,-16},{-204,-2}})));
  Modelica.Blocks.Math.Gain gain2(k=-1)
    annotation (Placement(transformation(extent={{-2,140},{10,152}})));
  Buildings.Controls.SetPoints.Table Zn_3_Control(table=[23.5 + 273.15,0.163*
        0.5*1.2; 24.5 + 273.15,1.2*0.163*1; 25.5 + 273.15,1.2*0.163*1.2])
    annotation (Placement(transformation(extent={{120,90},{134,104}})));
  Modelica.Blocks.Math.Gain gain3(k=-1)
    annotation (Placement(transformation(extent={{80,146},{68,158}})));
  Buildings.Controls.SetPoints.Table Zn_4_Control(table=[23.5 + 273.15,0.258*
        0.5*1.2; 24.5 + 273.15,1.2*0.258*1; 25.5 + 273.15,1.2*0.258*1.2])
    annotation (Placement(transformation(extent={{120,66},{134,80}})));
  Modelica.Blocks.Math.Gain gain4(k=-1)
    annotation (Placement(transformation(extent={{132,134},{120,146}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=if time <=
        1.728e+07 + 3600 then 0.5 else Zn_1.phi)
    annotation (Placement(transformation(extent={{-210,-60},{-190,-40}})));
  Buildings.Fluid.Sensors.RelativeHumidityTwoPort senRelHum(
    redeclare package Medium = Buildings.Media.Air,
    allowFlowReversal=true,
    m_flow_nominal=1,
    m_flow_small=0.001) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={-134,-70})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=if time <=
        1.728e+07 + 3600 then 0.01 else evaporator1.portB_sec.X_outflow[1])
    annotation (Placement(transformation(extent={{-202,-88},{-182,-68}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=1 -
        realExpression3.y)
    annotation (Placement(transformation(extent={{-194,-110},{-174,-90}})));
  inner
    Modelon.ThermoFluid.AggregateProperties.Aggregates.AggregateTwoPhaseProperties
    aggregateTwoPhaseProperties
    annotation (Placement(transformation(extent={{-276,34},{-256,54}})));
  VaporCycle.Pipes.PipeAdiabatic pipe(
    redeclare package Medium =
        VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
    n=3,
    n_channels=1,
    L=1,
    D=5/8*0.0254,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    p_in_start=1000000,
    p_out_start=995000,
    CF_PressureLoss=1,
    includeStaticHead=false)
    annotation (Placement(transformation(extent={{42,-50},{62,-30}})));
  Buildings.ThermalZones.EnergyPlus.OutputVariable out(name=
        "Zone Electric Equipment Electricity Rate", key="Perimeter_ZN_1")
    annotation (Placement(transformation(extent={{-262,104},{-242,124}})));
  Modelica.Blocks.Sources.Pulse pulse(
    amplitude=15,
    width=5/7*100,
    period(displayUnit="d") = 604800)
    annotation (Placement(transformation(extent={{26,176},{42,192}})));
  Buildings.ThermalZones.EnergyPlus.Actuator act(
    variableName="Perimeter_ZN_1_MiscPlug_Equip",
    componentType="ElectricEquipment",
    controlType="Electricity Rate",
    unit=Buildings.ThermalZones.EnergyPlus.Types.Units.Power)
    annotation (Placement(transformation(extent={{70,174},{90,194}})));
  Buildings.Fluid.Sources.MassFlowSource_WeatherData Core_Fresh(
    redeclare package Medium = Buildings.Media.Air,
    use_m_flow_in=true,
    m_flow=150*0.3/1000 + 150/18.58*2.5/1000,
    nPorts=1)
    annotation (Placement(transformation(extent={{-140,72},{-124,88}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=-Core_Fresh.m_flow_in)
    annotation (Placement(transformation(extent={{-178,164},{-158,184}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-122,176},{-102,196}})));
  Buildings.Fluid.Sources.MassFlowSource_WeatherData Zn1_Fresh(
    redeclare package Medium = Buildings.Media.Air,
    m_flow=0.1*(113*0.3/1000 + 113/18.58*2.5/1000),
    nPorts=1)
    annotation (Placement(transformation(extent={{-44,204},{-28,220}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=-Zn1_Fresh.m_flow)
    annotation (Placement(transformation(extent={{-118,230},{-98,250}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{-58,254},{-38,274}})));
  Buildings.Fluid.Sources.MassFlowSource_WeatherData Zn2_Fresh(
    redeclare package Medium = Buildings.Media.Air,
    m_flow=67*113*0.3/1000 + 67/18.58*2.5/1000,
    nPorts=1)
    annotation (Placement(transformation(extent={{34,236},{50,252}})));
  Modelica.Blocks.Math.Add add2
    annotation (Placement(transformation(extent={{58,274},{78,294}})));
  Modelica.Blocks.Sources.RealExpression realExpression7(y=-Zn2_Fresh.m_flow)
    annotation (Placement(transformation(extent={{10,264},{30,284}})));
  Buildings.Fluid.Sources.MassFlowSource_WeatherData Zn3_Fresh(
    redeclare package Medium = Buildings.Media.Air,
    m_flow=67*113*0.3/1000 + 67/18.58*2.5/1000,
    nPorts=1)
    annotation (Placement(transformation(extent={{104,238},{120,254}})));
  Modelica.Blocks.Sources.RealExpression realExpression8(y=-Zn3_Fresh.m_flow)
    annotation (Placement(transformation(extent={{124,272},{144,292}})));
  Modelica.Blocks.Math.Add add3
    annotation (Placement(transformation(extent={{168,262},{188,282}})));
  Buildings.Fluid.Sources.MassFlowSource_WeatherData Zn4_Fresh(
    redeclare package Medium = Buildings.Media.Air,
    m_flow=67*113*0.3/1000 + 67/18.58*2.5/1000,
    nPorts=1)
    annotation (Placement(transformation(extent={{210,218},{226,234}})));
  Modelica.Blocks.Sources.RealExpression realExpression9(y=-Zn4_Fresh.m_flow)
    annotation (Placement(transformation(extent={{208,254},{228,274}})));
  Modelica.Blocks.Math.Add add4
    annotation (Placement(transformation(extent={{244,242},{264,262}})));
  Modelica.Blocks.Sources.RealExpression realExpression10(y=150*0.3/1000 +
        150/18.58*2.5/1000)
    annotation (Placement(transformation(extent={{-220,166},{-200,186}})));
  Modelica.Blocks.Math.Product product
    annotation (Placement(transformation(extent={{-184,188},{-164,208}})));
  Modelica.Blocks.Sources.Pulse pulse1(
    amplitude=1,
    width=13/24*100,
    period(displayUnit="h") = 86400,
    offset=0.01,
    startTime(displayUnit="h") = 21600)
    annotation (Placement(transformation(extent={{-228,212},{-212,228}})));
  Modelica.Blocks.Sources.Pulse pulse2(
    amplitude=0,
    width=13/24*100,
    period(displayUnit="h") = 86400,
    offset=1,
    startTime(displayUnit="h") = 21600)
    annotation (Placement(transformation(extent={{-192,-176},{-176,-160}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{-146,-162},{-126,-142}})));
  control_test control_test1(
    tWai(displayUnit="s") = 5,
    TSet=23 + 273.15,
    deaBan=0.5,
    comSpe(fixed=true, start=0.1))
    annotation (Placement(transformation(extent={{92,-2},{112,18}})));
  FDD_RTU.Fan_control_v4 fan_control_v4_1
    annotation (Placement(transformation(extent={{170,4},{190,24}})));
  Modelica.Blocks.Math.Gain gain5(k=0.45)
    annotation (Placement(transformation(extent={{34,-128},{54,-108}})));
  VaporCycle.HeatExchangers.TwoPhaseAir.Examples.Evaporator
                                                        condenser(
    init_air(
      dp(displayUnit="Pa") = 50,
      T_in=298.15,
      m_flow=1,
      phi_in=0.5,
      phi_outlet=0.2),
    init_wf(
      p_in=1700000,
      p_out=1690000,
      h_in=450e3 - 142e3,
      h_out=350e3 - 142e3,
      m_flow=0.1),
    redeclare package WorkingFluid =
        VaporCycle.Media.Hydrofluorocarbons.R410aPseudoPure,
    n=3,
    channelDensity=fill(80*100, 3),
    n_channels=fill(20*100, 3),
    L=fill(2/12, 3),
    redeclare model HeatTransfer =
        VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.ConstantCoefficient
        (alpha0=2000),
    CF_Friction=1,
    include_in_aggregate=true,
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
    redeclare package Air = Modelon.Media.PreDefined.CondensingGases.MoistAir) annotation (Placement(transformation(extent={{56,-76},
            {36,-96}})));
protected
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant qGai_flow[3](k={0,25,
        0})
    "Internal heat gain (computed already in EnergyPlus"
    annotation (Placement(transformation(extent={{-112,-56},{-92,-36}})));
protected
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant qGai_flow1
                                                                  [3](k={0,20,
        5})
    "Internal heat gain (computed already in EnergyPlus"
    annotation (Placement(transformation(extent={{-88,-56},{-68,-36}})));
equation
  connect(qGai_flow.y,Core. qGai_flow) annotation (Line(points={{-90,-46},{
          -90,83},{-89.3,83}},
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
  connect(expansionValve.portB, evaporator1.portA_prim)
    annotation (Line(points={{-2,-46},{-2,-24},{16,-24}}, color={0,190,0}));
  connect(flowSource1.port, evaporator1.portA_sec) annotation (Line(points={{56,-6},
          {44,-6},{44,-12},{36,-12}},     color={85,170,255}));
  connect(building.weaBus, bou.weaBus) annotation (Line(
      points={{-154,70},{-154,32.16},{-166,32.16}},
      color={255,204,51},
      thickness=0.5));
  connect(bou.ports[1], senTem.port_a)
    annotation (Line(points={{-150,32},{-150,22},{-134,22},{-134,6}},
                                                  color={0,127,255}));
  connect(compressor.flange, rps.flange)
    annotation (Line(points={{109,-56},{118,-56}}, color={0,0,0}));
  connect(rps.inPort, speed.y)
    annotation (Line(points={{140,-56},{163.4,-56}}, color={0,0,127}));
  connect(Core.ports[1], Core_SF.ports[1]) annotation (Line(points={{-76.7333,
          62.63},{-76.7333,43},{-104,43}},
                                 color={0,127,255}));
  connect(Zn_1.ports[1], Zn1_SF.ports[1]) annotation (Line(points={{-36.7333,
          62.63},{-36.7333,43},{-54,43}},
                                color={0,127,255}));
  connect(Core_RF.ports[1], Core.ports[2]) annotation (Line(points={{-82,117},
          {-98,117},{-98,60},{-75,60},{-75,62.63}},color={0,127,255}));
  connect(airTemperatureSensor.port, evaporator1.portB_sec) annotation (Line(
        points={{-10,-11},{2,-11},{2,-12},{16,-12}}, color={85,170,255}));
  connect(airTemperatureSensor.port, pressureSource2.port[1]) annotation (Line(
        points={{-10,-11},{-26,-11},{-26,-10},{-40,-10}}, color={85,170,255}));
  connect(airTemperatureSensor.outPort, Core_SF.T_in) annotation (Line(points={{
          -15,-6},{-20,-6},{-20,0},{-32,0},{-32,32},{-124,32},{-124,45.8},{-119.4,
          45.8}}, color={0,0,127}));
  connect(airTemperatureSensor.outPort, Zn1_SF.T_in) annotation (Line(points={{-15,
          -6},{-42,-6},{-42,20},{-74,20},{-74,45.8},{-69.4,45.8}}, color={0,0,127}));
  connect(qGai_flow1.y, Zn_1.qGai_flow) annotation (Line(points={{-66,-46},{-64,
          -46},{-64,2},{-60,2},{-60,83},{-49.3,83}}, color={0,0,127}));
  connect(Zn_1.TAir, Zn_1_Control.u) annotation (Line(points={{-21.35,88.6},{
          -21.35,40},{-226,40},{-226,17},{-219.4,17}}, color={0,0,127}));
  connect(Zn_1.ports[2], Zn1_RF.ports[1]) annotation (Line(points={{-35,62.63},
          {-35,60},{-18,60},{-18,125}},                 color={0,127,255}));
  connect(Zn2_SF.ports[1], Zn_2.ports[1]) annotation (Line(points={{4,43},{6,
          43},{6,62.63},{3.26667,62.63}},
                                      color={0,127,255}));
  connect(Zn_2.ports[2], Zn2_RF.ports[1]) annotation (Line(points={{5,62.63},
          {5,58},{24,58},{24,116},{10,116},{10,125},{16,125}},   color={0,127,
          255}));
  connect(Zn3_SF.ports[1], Zn_3.ports[1]) annotation (Line(points={{38,43},{42,
          43},{42,62.63},{43.2667,62.63}}, color={0,127,255}));
  connect(Zn_3.ports[2], Zn3_RF.ports[1]) annotation (Line(points={{45,62.63},
          {45,48},{60,48},{60,116},{44,116},{44,125},{54,125}},
        color={0,127,255}));
  connect(Zn4_SF.ports[1], Zn_4.ports[1]) annotation (Line(points={{84,43},{82,
          43},{82,44},{85,44},{85,62.63},{83.2667,62.63}}, color={0,127,255}));
  connect(Zn_4.ports[2], Zn4_RF.ports[1]) annotation (Line(points={{85,62.63},
          {85,54},{110,54},{110,116},{80,116},{80,127},{94,127}},
        color={0,127,255}));
  connect(Core.TAir, Core_Control.u) annotation (Line(points={{-61.35,88.6},{
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
  connect(Zn_3.TAir, Zn_3_Control.u) annotation (Line(points={{58.65,88.6},{
          64,88.6},{64,97},{118.6,97}}, color={0,0,127}));
  connect(Zn_3_Control.y, Zn3_SF.m_flow_in) annotation (Line(points={{134.7,
          97},{146,97},{146,52},{16,52},{16,48.6},{24,48.6}}, color={0,0,127}));
  connect(Zn_3_Control.y, gain3.u) annotation (Line(points={{134.7,97},{140,
          97},{140,152},{81.2,152}}, color={0,0,127}));
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
  connect(senTem.T, flowSource2.T_in) annotation (Line(points={{-123,-4},{
          -110,-4},{-110,-8},{-26,-8},{-26,-85}}, color={0,0,127}));
  connect(realExpression.y, flowSource1.T_in) annotation (Line(points={{-159,
          -32},{-54,-32},{-54,-26},{64,-26},{64,1}}, color={0,0,127}));
  connect(realExpression2.y, flowSource1.phi_in) annotation (Line(points={{
          -189,-50},{-66,-50},{-66,-1},{59.4,-1}}, color={0,0,127}));
  connect(senTem.port_b, senRelHum.port_a)
    annotation (Line(points={{-134,-14},{-134,-60}}, color={0,127,255}));
  connect(senRelHum.port_b, sin1.ports[1]) annotation (Line(points={{-134,-80},
          {-134,-98},{-128,-98},{-128,-94}}, color={0,127,255}));
  connect(senRelHum.phi, flowSource2.phi_in) annotation (Line(points={{-123,
          -70.1},{-21.4,-70.1},{-21.4,-87}}, color={0,0,127}));
  connect(realExpression3.y, Zn1_SF.X_in[1]) annotation (Line(points={{-181,
          -78},{-166,-78},{-166,-80},{-69.4,-80},{-69.4,40.2}}, color={0,0,
          127}));
  connect(realExpression4.y, Zn1_SF.X_in[2]) annotation (Line(points={{-173,
          -100},{-158,-100},{-158,-94},{-69.4,-94},{-69.4,40.2}}, color={0,0,
          127}));
  connect(superHeatSensor.portB, compressor.portA) annotation (Line(points={{
          92,-24},{96,-24},{96,-46},{100,-46}}, color={0,190,0}));
  connect(superHeatSensor.y, expansionValve.DeltaT_SH) annotation (Line(
        points={{82,-15},{76,-15},{76,12},{-16,12},{-16,-57.5},{-9.5,-57.5}},
        color={0,0,127}));
  connect(evaporator1.portB_prim, pipe.portA) annotation (Line(points={{36,-24},
          {38,-24},{38,-40},{42,-40}},      color={0,190,0}));
  connect(pipe.portB, superHeatSensor.portA) annotation (Line(points={{62,-40},
          {66,-40},{66,-24},{72,-24}}, color={0,190,0}));
  connect(pulse.y, act.u) annotation (Line(points={{42.8,184},{68,184}},
                          color={0,0,127}));
  connect(building.weaBus, Core_Fresh.weaBus) annotation (Line(
      points={{-154,70},{-148,70},{-148,80.16},{-140,80.16}},
      color={255,204,51},
      thickness=0.5));
  connect(Core_Fresh.ports[1], Core.ports[3]) annotation (Line(points={{-124,80},
          {-116,80},{-116,58},{-73.2667,58},{-73.2667,62.63}},     color={0,
          127,255}));
  connect(realExpression5.y, add.u1) annotation (Line(points={{-157,174},{
          -142,174},{-142,192},{-124,192}}, color={0,0,127}));
  connect(gain.y, add.u2) annotation (Line(points={{-95.4,122},{-114,122},{
          -114,174},{-128,174},{-128,180},{-124,180}}, color={0,0,127}));
  connect(add.y, Core_RF.m_flow_in) annotation (Line(points={{-101,186},{-88,
          186},{-88,124},{-68,124},{-68,122.6}}, color={0,0,127}));
  connect(building.weaBus, Zn1_Fresh.weaBus) annotation (Line(
      points={{-154,70},{-152,70},{-152,228},{-44,228},{-44,212.16}},
      color={255,204,51},
      thickness=0.5));
  connect(Zn1_Fresh.ports[1], Zn_1.ports[3]) annotation (Line(points={{-28,212},
          {-28,52},{-33.2667,52},{-33.2667,62.63}},      color={0,127,255}));
  connect(realExpression6.y, add1.u1) annotation (Line(points={{-97,240},{-80,
          240},{-80,270},{-60,270}}, color={0,0,127}));
  connect(gain1.y, add1.u2) annotation (Line(points={{-43.4,126},{-56,126},{
          -56,250},{-60,250},{-60,258}}, color={0,0,127}));
  connect(add1.y, Zn1_RF.m_flow_in) annotation (Line(points={{-37,264},{-32,
          264},{-32,262},{-20,262},{-20,130.6},{-4,130.6}}, color={0,0,127}));
  connect(building.weaBus, Zn2_Fresh.weaBus) annotation (Line(
      points={{-154,70},{-152,70},{-152,256},{34,256},{34,244.16}},
      color={255,204,51},
      thickness=0.5));
  connect(Zn2_Fresh.ports[1], Zn_2.ports[3]) annotation (Line(points={{50,244},
          {56,244},{56,48},{6.73333,48},{6.73333,62.63}}, color={0,127,255}));
  connect(realExpression7.y, add2.u1) annotation (Line(points={{31,274},{44,
          274},{44,290},{56,290}}, color={0,0,127}));
  connect(gain2.y, add2.u2) annotation (Line(points={{10.6,146},{32,146},{32,
          278},{56,278}}, color={0,0,127}));
  connect(add2.y, Zn2_RF.m_flow_in) annotation (Line(points={{79,284},{80,284},
          {80,130.6},{30,130.6}}, color={0,0,127}));
  connect(building.weaBus, Zn3_Fresh.weaBus) annotation (Line(
      points={{-154,70},{-26,70},{-26,246.16},{104,246.16}},
      color={255,204,51},
      thickness=0.5));
  connect(Zn3_Fresh.ports[1], Zn_3.ports[3]) annotation (Line(points={{120,246},
          {130,246},{130,44},{50,44},{50,62.63},{46.7333,62.63}},      color=
          {0,127,255}));
  connect(realExpression8.y, add3.u1) annotation (Line(points={{145,282},{156,
          282},{156,278},{166,278}}, color={0,0,127}));
  connect(gain3.y, add3.u2) annotation (Line(points={{67.4,152},{62,152},{62,
          166},{158,166},{158,266},{166,266}}, color={0,0,127}));
  connect(add3.y, Zn3_RF.m_flow_in) annotation (Line(points={{189,272},{196,
          272},{196,130.6},{68,130.6}}, color={0,0,127}));
  connect(building.weaBus, Zn4_Fresh.weaBus) annotation (Line(
      points={{-154,70},{28,70},{28,226.16},{210,226.16}},
      color={255,204,51},
      thickness=0.5));
  connect(Zn4_Fresh.ports[1], Zn_4.ports[3]) annotation (Line(points={{226,226},
          {232,226},{232,32},{90,32},{90,62.63},{86.7333,62.63}},      color=
          {0,127,255}));
  connect(realExpression9.y, add4.u1) annotation (Line(points={{229,264},{234,
          264},{234,258},{242,258}}, color={0,0,127}));
  connect(gain4.y, add4.u2) annotation (Line(points={{119.4,140},{122,140},{
          122,220},{242,220},{242,246}}, color={0,0,127}));
  connect(add4.y, Zn4_RF.m_flow_in) annotation (Line(points={{265,252},{266,
          252},{266,132.6},{108,132.6}}, color={0,0,127}));
  connect(realExpression10.y, product.u2) annotation (Line(points={{-199,176},
          {-194,176},{-194,192},{-186,192}}, color={0,0,127}));
  connect(product.y, Core_Fresh.m_flow_in) annotation (Line(points={{-163,198},
          {-152,198},{-152,86.4},{-140,86.4}}, color={0,0,127}));
  connect(pulse1.y, product.u1) annotation (Line(points={{-211.2,220},{-200,
          220},{-200,204},{-186,204}}, color={0,0,127}));
  connect(pulse2.y, product1.u2) annotation (Line(points={{-175.2,-168},{-162,
          -168},{-162,-158},{-148,-158}}, color={0,0,127}));
  connect(flowSource1.m_flow_in, product1.y) annotation (Line(points={{69,-1},
          {69,4},{-98,4},{-98,-152},{-125,-152}}, color={0,0,127}));
  connect(Zn_1.TAir, control_test1.TAct) annotation (Line(points={{-21.35,
          88.6},{72,88.6},{72,8},{91,8}}, color={0,0,127}));
  connect(control_test1.comSpe, compressor.relativeDisplacement) annotation (
      Line(points={{113,8},{124,8},{124,10},{134,10},{134,-46},{106,-46}},
        color={0,0,127}));
  connect(control_test1.comSpe, fan_control_v4_1.u) annotation (Line(points={
          {113,8},{140,8},{140,14.4},{168.4,14.4}}, color={0,0,127}));
  connect(fan_control_v4_1.y, product1.u1) annotation (Line(points={{190.6,
          14.6},{198,14.6},{198,-130},{-154,-130},{-154,-146},{-148,-146}},
        color={0,0,127}));
  connect(fan_control_v4_1.y, gain5.u) annotation (Line(points={{190.6,14.6},
          {88,14.6},{88,-104},{12,-104},{12,-118},{32,-118}}, color={0,0,127}));
  connect(gain5.y, Zn1_SF.m_flow_in) annotation (Line(points={{55,-118},{62,
          -118},{62,-116},{-82,-116},{-82,48.6},{-68,48.6}}, color={0,0,127}));
  connect(gain5.y, gain1.u) annotation (Line(points={{55,-118},{-60,-118},{
          -60,126},{-57.2,126}}, color={0,0,127}));
  connect(compressor.portB, condenser.portA_prim) annotation (Line(points={{
          100,-66},{78,-66},{78,-80},{56,-80}}, color={0,190,0}));
  connect(condenser.portB_prim, expansionValve.portA) annotation (Line(points=
         {{36,-80},{16,-80},{16,-62},{-2,-62}}, color={0,190,0}));
  connect(flowSource2.port, condenser.portA_sec) annotation (Line(points={{
          -18,-92},{10,-92},{10,-92},{36,-92}}, color={85,170,255}));
  connect(pressureSource3.port[1], condenser.portB_sec) annotation (Line(
        points={{60,-94},{58,-94},{58,-92},{56,-92}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=23328000,
      StopTime=23760000,
      Interval=59.9999616,
      Tolerance=1e-06,
      __Dymola_Algorithm="Cvode"),
    uses(
      Buildings(version="9.0.0"),
      Modelica(version="4.0.0"),
      VaporCycle(version="2.8"),
      Modelon(version="4.0")));
end Five_Zone_RTU_chargestudy_simpleTXVv2_v4;
