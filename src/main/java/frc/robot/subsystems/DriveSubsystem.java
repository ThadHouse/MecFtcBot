// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.GoBildaPinpoint;
import frc.robot.Constants.DriveConstants;
import frc.robot.GoBildaPinpoint.GoBildaOdometryPods;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class DriveSubsystem extends SubsystemBase {
  private final Spark m_frontLeft = new Spark(DriveConstants.kFrontLeftMotorPort);
  private final Spark m_rearLeft = new Spark(DriveConstants.kRearLeftMotorPort);
  private final Spark m_frontRight = new Spark(DriveConstants.kFrontRightMotorPort);
  private final Spark m_rearRight = new Spark(DriveConstants.kRearRightMotorPort);

  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set,
      m_rearRight::set);

  // The front-left-side drive encoder
  private final Encoder m_frontLeftEncoder = new Encoder(
      DriveConstants.kFrontLeftEncoderPorts[0],
      DriveConstants.kFrontLeftEncoderPorts[1],
      DriveConstants.kFrontLeftEncoderReversed);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder = new Encoder(
      DriveConstants.kRearLeftEncoderPorts[0],
      DriveConstants.kRearLeftEncoderPorts[1],
      DriveConstants.kRearLeftEncoderReversed);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder = new Encoder(
      DriveConstants.kFrontRightEncoderPorts[0],
      DriveConstants.kFrontRightEncoderPorts[1],
      DriveConstants.kFrontRightEncoderReversed);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder = new Encoder(
      DriveConstants.kRearRightEncoderPorts[0],
      DriveConstants.kRearRightEncoderPorts[1],
      DriveConstants.kRearRightEncoderReversed);

  @NotLogged
  private final GoBildaPinpoint m_pinpoint = new GoBildaPinpoint(Port.kOnboard);

  // Configure Network Tables topics (oculus/...) to communicate with the Quest
  // HMD
  @NotLogged
  private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  @NotLogged
  private final NetworkTable nt4Table = nt4Instance.getTable("oculus");
  @NotLogged
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  @NotLogged
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables oculus data topics
  @NotLogged
  private IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
  @NotLogged
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  @NotLogged
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
      @NotLogged
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
      @NotLogged
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });

  public float[] getQuestQuaternion() {
    return questQuaternion.get();
  }

  public float[] getQuestPosition() {
    return questPosition.get();
  }

  public float[] getQuestEulerAngles() {
    return questEulerAngles.get();
  }

  public double getQuestTimestamp() {
    return questTimestamp.get();
  }

  public long getQuestFrameCount() {
    return questFrameCount.get();
  }

  // Local heading helper variables
  private float yaw_offset = 0.0f;

  // Odometry class for tracking robot pose
  @NotLogged
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
      DriveConstants.kDriveKinematics,
      getOculusRotation2d(),
      new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // Sets the distance per pulse for the encoders
    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    m_pinpoint.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
    m_pinpoint.setPosition(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(getOculusRotation2d(), getCurrentWheelDistances());
    m_pinpoint.update();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getOculusRotation2d(), getCurrentWheelDistances(), pose);
    m_pinpoint.setPosition(pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, getOculusRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_rearLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_rearRightEncoder.getRate());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a
   *         MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_rearLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_rearRightEncoder.getDistance());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  // Zero the realative robot heading
  public void zeroHeading() {
    System.out.println("Resetting Heading");
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the
  // quest logo)
  public void zeroPosition() {
    System.out.println("Resetting Position");
    resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  // Clean up oculus subroutine messages after processing on the headset
  public void cleanUpOculusMessages() {
    //System.out.println("Cleaning up msgs");
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Return the robot heading in degrees, between -180 and 180 degrees
  public double getHeading() {
    return Rotation2d.fromDegrees(getOculusYaw()).getDegrees();
  }

  // Get the rotation rate of the robot
  public double getTurnRate() {
    return getOculusYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private Rotation2d getOculusRotation2d() {
    return Rotation2d.fromDegrees(getOculusYaw());
  }

  // Get the yaw Euler angle of the headset
  @Logged
  public float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    return -(eulerAngles[1] - yaw_offset);
  }

  @Logged
  public Translation2d getOculusPosition() {
    float[] oculusPosition = questPosition.get();
    return new Translation2d(oculusPosition[2], -oculusPosition[0]);
  }

  public Pose2d getOculusPose() {
    return new Pose2d(getOculusPosition(), Rotation2d.fromDegrees(getOculusYaw()));
  }

  public Pose2d getPinpointPose() {
    return m_pinpoint.getPosition();
  }

  public double getPinpointEncoderX() {
    return m_pinpoint.getEncoderX();
  }

  public double getPinpointEncoderY() {
    return m_pinpoint.getEncoderY();
  }
}
