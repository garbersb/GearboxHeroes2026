// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
      // 1. Initialize the Pigeon (use your actual CAN ID)
    private final Pigeon2 m_gyro = new Pigeon2(13); 

    //This is needed for path following and is used to convert from chassis speeds (m/s and rad/s) to wheel speeds (m/s)
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0));
    private final DifferentialDriveOdometry m_odometry;

    
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

    private  SparkClosedLoopController m_leftClosedLoopController;
    private  SparkClosedLoopController m_rightClosedLoopController;

  public CANDriveSubsystem() {

    //output the yaw position of the robot
    SmartDashboard.putNumber("Robot Yaw", m_gyro.getYaw().getValueAsDouble());

    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // Add these:
    m_leftClosedLoopController = leftLeader.getClosedLoopController();
    m_rightClosedLoopController = rightLeader.getClosedLoopController();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // 1. Create a "Base" configuration for settings shared by ALL motors
    SparkMaxConfig baseConfig = new SparkMaxConfig();
    baseConfig.voltageCompensation(12);
    baseConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    
    // Set conversion factors for all motors so they all "speak" meters
    double wheelDiameterMeters = edu.wpi.first.math.util.Units.inchesToMeters(6.0);

    //double gearRatio = 10.71; // Update this to your actual ratio!
    double gearRatio = 1;
    double positionFactor = (Math.PI * wheelDiameterMeters) / gearRatio;
    
    baseConfig.encoder.positionConversionFactor(positionFactor);
    baseConfig.encoder.velocityConversionFactor(positionFactor / 60.0);

    // 2. Create Leader-Specific Configuration
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.apply(baseConfig); // Copy the base settings
    
    // Add PID for PathPlanner to the leader config
    leaderConfig.closedLoop.p(0.1); // Tune these!
    leaderConfig.closedLoop.feedForward.kV(0.25);
  
    // 3. Create Follower-Specific Configuration
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.apply(baseConfig); // Copy base settings
    leftFollowerConfig.follow(leftLeader); // Follow the left leader

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.apply(baseConfig); // Copy base settings
    rightFollowerConfig.follow(rightLeader); // Follow the right leader

    // 4. Apply configurations to the specific motors
    // Note: We apply "inverted" to the Left Leader only
    leaderConfig.inverted(true);
    leftLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    // Revert "inverted" for the Right side
    leaderConfig.inverted(false);
    rightLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


/////////////////////////////////////////////////////////////////////////////
    //     // Reset encoders to zero before starting odometry
    // leftLeader.getEncoder().setPosition(0);
    // rightLeader.getEncoder().setPosition(0);
    // m_gyro.setYaw(0);

    // Initialize the odometry object
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), 
        leftLeader.getEncoder().getPosition(), 
        rightLeader.getEncoder().getPosition()
    );


    // try{
    //   robotCfg = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //   // Handle exception as needed
    //   e.printStackTrace();
    // }

    // // Configure AutoBuilder last
    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
    //         robotCfg, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );

  }



  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }


      public Rotation2d getRotation2d() {
        // 2. PathPlanner and WPILib prefer Rotation2d
        return m_gyro.getRotation2d();
    }

    public void zeroHeading() {
        // 3. Set the current heading to 0
        m_gyro.setYaw(0);
    }

// public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
//     // 1. Convert speeds to individual wheel targets (m/s)
//     DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

//     // 2. Extract linear forces (Newtons) from PathPlanner
//     // Index 0 = Left, Index 1 = Right for Differential Drive
//     double leftFeedforwardVolts = feedforwards.linearForcesNewtons()[0];
//     double rightFeedforwardVolts = feedforwards.linearForcesNewtons()[1];

//     // 3. Apply to Spark Controllers
//     // We use the 'ArbFeedforward' parameter (the 4th argument)
//     m_leftClosedLoopController.setReference(
//         wheelSpeeds.leftMetersPerSecond, 
//         SparkMax.ControlType.kVelocity,
//         ClosedLoopSlot.kSlot0, 
//         leftFeedforwardVolts
//     );

//     m_rightClosedLoopController.setReference(
//         wheelSpeeds.rightMetersPerSecond, 
//         SparkMax.ControlType.kVelocity,
//         ClosedLoopSlot.kSlot0,
//         rightFeedforwardVolts
//     );
// }
//     public void driveRobotRelative(ChassisSpeeds speeds) {
//     // 1. Use Kinematics to convert ChassisSpeeds (vx, omega) 
//     // to individual wheel speeds (left and right m/s)
//     DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

//     // 2. Extract the target velocity for each side
//     // double leftVelocity = wheelSpeeds.leftMetersPerSecond;
//     // double rightVelocity = wheelSpeeds.rightMetersPerSecond;

//     // 3. Command your SparkMax controllers to reach these velocities
//     // Note: It is highly recommended to use PID velocity control 
//     // rather than simple percent output for accurate path following.
//     // m_leftPIDController.setReference(leftVelocity, CANSparkMax.ControlType.kVelocity);
//     // m_rightPIDController.setReference(rightVelocity, CANSparkMax.ControlType.kVelocity);

//         // Use the closedLoop controller specifically
//     m_leftClosedLoopController.setSetpoint(
//         wheelSpeeds.leftMetersPerSecond, 
//         SparkMax.ControlType.kVelocity
//     );
//     m_rightClosedLoopController.setSetpoint(
//         wheelSpeeds.rightMetersPerSecond, 
//         SparkMax.ControlType.kVelocity
//     );
// }


      /**
       * Returns a command that turns the robot to a specific angle.
       * @param targetAngle The target angle in degrees (e.g., 90 for right, -90 for left)
       */
      public Command turnToAngle(double targetAngle) {
        return this.run(() -> {
            double currentAngle = m_gyro.getYaw().getValueAsDouble();
            
            // Determine direction: if target is greater than current, turn right (positive)
            if (currentAngle < targetAngle) {
                drive.arcadeDrive(0, 0.5); // Turn Right
            } else {
                drive.arcadeDrive(0, -0.5); // Turn Left
            }
        })
        // This condition stops the command when the angle is reached (within a 2-degree buffer)
        .until(() -> Math.abs(m_gyro.getYaw().getValueAsDouble() - targetAngle) < 2.0)
        // Ensure motors stop when the command ends
        .finallyDo(() -> drive.stopMotor());
      }


      // will turn the robot 90 degrees
      public Command turn90Right() {
        return turnToAngle(90.0);
      }

      // will turn the robot -90 degrees
      public Command turn90Left() {
        return turnToAngle(-80.0);
      }

      public Command turn180Left() {
        return turnToAngle(-180); 
      }

      public Command turn180Right() {
        return turnToAngle(180);
      }




    @Override
    public void periodic() {


      // Monitor Left Side
      SmartDashboard.putNumber("Left Drive/Left Target", m_leftClosedLoopController.getSetpoint());
      SmartDashboard.putNumber("Left Drive/Left Actual", leftLeader.getEncoder().getVelocity());
      SmartDashboard.putNumber("Left Position Actual", leftLeader.getEncoder().getPosition());

      // Monitor Right Side
      SmartDashboard.putNumber("Right Drive/Right Target", m_rightClosedLoopController.getSetpoint());
      SmartDashboard.putNumber("Right Drive/Right Actual", rightLeader.getEncoder().getVelocity());
      SmartDashboard.putNumber("Right Position Actual", rightLeader.getEncoder().getPosition());


        // 4. Use the Pigeon to update your Odometry
        m_odometry.update(
            getRotation2d(), 
            leftLeader.getEncoder().getPosition(), 
            rightLeader.getEncoder().getPosition());
    }

    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }


    public void resetPose(Pose2d pose) {
  
        // 1. Manually set encoder positions to 0
        leftLeader.getEncoder().setPosition(0);
        rightLeader.getEncoder().setPosition(0);    

        // 2. Reset the Pigeon gyro (if you want the robot's current heading to be 0)
        m_gyro.setYaw(0);

        // 3. Reset the odometry to the specific position and rotation requested
        // Note: Use getRotation2d() from your Pigeon
        m_odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);

}


public ChassisSpeeds getRobotRelativeSpeeds() {
    // Get speeds from your encoders in meters per second
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        leftLeader.getEncoder().getVelocity(),
        rightLeader.getEncoder().getVelocity()  
    );

    System.out.println("wheelSpeeds" + wheelSpeeds);

    // Convert to ChassisSpeeds (vx and omega)
    return m_kinematics.toChassisSpeeds(wheelSpeeds);
    }

}
