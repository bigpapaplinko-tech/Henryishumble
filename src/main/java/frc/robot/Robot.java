// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.NetworkTable;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimesliceRobot documentation. If you change the name of this class or the package after
 * creating this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimesliceRobot {
  /** Robot constructor. */
  public Robot() {
    // Run robot periodic() functions for 5 ms, and run controllers every 10 ms
    super(0.005, 0.01);

    // LiveWindow causes drastic overruns in robot periodic functions that will
    // interfere with controllers
    LiveWindow.disableAllTelemetry();

    // Start DataLog for AdvantageScope
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Initialize odometry and simulation
    odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0, new Pose2d());

    // Create drivetrain simulation (Kitbot with CIM motors, standard gearing, 6" wheels)
    driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide,
      KitbotGearing.k10p71,
      KitbotWheelSize.kSixInch,
      null
    );

    // Send field to SmartDashboard for AdvantageScope
    SmartDashboard.putData("Field", field);

    // Initialize NetworkTables publishers for AdvantageScope
    NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drive");
    leftSpeedPub = driveTable.getDoubleTopic("LeftSpeed").publish();
    rightSpeedPub = driveTable.getDoubleTopic("RightSpeed").publish();

    NetworkTable joystickTable = NetworkTableInstance.getDefault().getTable("Joystick");
    joystickYPub = joystickTable.getDoubleTopic("Y-Axis").publish();
    joystickXPub = joystickTable.getDoubleTopic("X-Axis").publish();

    // Publish pose using modern struct format for AdvantageScope 3D field
    posePub = NetworkTableInstance.getDefault()
      .getStructTopic("SmartDashboard/Field/Robot", Pose2d.struct).publish();

    // Runs for 2 ms after robot periodic functions
    schedule(() -> {}, 0.002);

    // Runs for 2 ms after first controller function
    schedule(() -> {}, 0.002);

    // Total usage:
    // 5 ms (robot) + 2 ms (controller 1) + 2 ms (controller 2) = 9 ms
    // 9 ms / 10 ms -> 90% allocated
  }
private Spark leftmotor1 = new Spark(0);
private Spark leftmotor2 = new Spark(1);
private Spark rightmotor1 = new Spark(2);
private Spark rightmotor2 = new Spark(3);

private Joystick joy1 = new Joystick(0);

// Field and odometry for 3D visualization
private Field2d field = new Field2d();
private DifferentialDriveOdometry odometry;
private DifferentialDrivetrainSim driveSim;
private Pose2d pose = new Pose2d();
private double leftDistance = 0.0;
private double rightDistance = 0.0;

// NetworkTables publishers for AdvantageScope
private DoublePublisher leftSpeedPub;
private DoublePublisher rightSpeedPub;
private DoublePublisher joystickYPub;
private DoublePublisher joystickXPub;
private StructPublisher<Pose2d> posePub;

  @Override
  public void robotPeriodic() {
    // Update simulation (converts motor voltages to position changes)
    driveSim.setInputs(
      leftmotor1.get() * 12.0,  // Convert -1 to 1 range to voltage
      rightmotor1.get() * 12.0
    );
    driveSim.update(0.02);  // 20ms update period

    // Get simulated wheel positions
    leftDistance = driveSim.getLeftPositionMeters();
    rightDistance = driveSim.getRightPositionMeters();

    // Update odometry with simulated heading and wheel positions
    pose = odometry.update(
      driveSim.getHeading(),
      leftDistance,
      rightDistance
    );

    // Update field widget
    field.setRobotPose(pose);

    // Publish pose to NetworkTables for AdvantageScope 3D field (modern struct format)
    posePub.set(pose);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

@Override
public void teleopInit() {}

@Override
public void teleopPeriodic() {
  double speed = -joy1.getRawAxis(1) * 0.6;
  double turn = joy1.getRawAxis(4) * 0.3;

  double left = speed + turn;
  double right = speed - turn;

  leftmotor1.set(left);
  leftmotor2.set(left);
  rightmotor1.set(right);
  rightmotor2.set(right);

  // Publish data to NetworkTables for AdvantageScope
  leftSpeedPub.set(left);
  rightSpeedPub.set(right);
  joystickYPub.set(joy1.getRawAxis(1));
  joystickXPub.set(joy1.getRawAxis(4));

  // Also publish to SmartDashboard for easy viewing
  SmartDashboard.putNumber("Left Speed", left);
  SmartDashboard.putNumber("Right Speed", right);
  SmartDashboard.putNumber("Joystick Y", joy1.getRawAxis(1));
  SmartDashboard.putNumber("Joystick X", joy1.getRawAxis(4));
  SmartDashboard.putNumber("Pose X", pose.getX());
  SmartDashboard.putNumber("Pose Y", pose.getY());
  SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
}

@Override
public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
