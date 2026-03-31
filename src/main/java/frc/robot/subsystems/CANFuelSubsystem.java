// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;


//CanFuelSubystem
public class CANFuelSubsystem extends SubsystemBase {
  //private final SparkMax feederRoller;
  //private final SparkMax intakeLauncherRoller;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0); 

  //this will be used to call within the period method only so often so there
  //are not tons of request occurring on the CANBUS.
  private int loopCounter = 0;
  private final int UPDATE_INTERVAL = 5; // Change to 10 for even less frequent updates
    

  private TalonFX krakenFeederRoller; 
  private TalonFX krakenIntakeLauncherRoller;

  public double INTAKING_FEEDER_RPS_SETPOINT = -40;
  public double INTAKING_INTAKE_RPS_SETPOINT = 30; 

  public double LAUNCHING_FEEDER_RPS_SETPOINT = 40;
  public double LAUNCHING_LAUNCHER_RPS_SETPOINT = 56;

  public double SPIN_UP_FEEDER_RPS_SETPOINT = -40;



  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    //intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    //feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
    
    krakenIntakeLauncherRoller = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID);
    krakenFeederRoller = new TalonFX(FEEDER_MOTOR_ID);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
     //SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_RPS_SETPOINT);
     //SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_RPS_SETPOINT);


    // SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    // SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    // SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    // SparkMaxConfig feederConfig = new SparkMaxConfig();
    // feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    // feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

          // 1. Create the configuration object
      TalonFXConfiguration krakenConfigFeederRoller = new TalonFXConfiguration();

      // 2. Set the Current Limit (Supply current is what the battery sees)
      // In FRC, 40A is a standard "safe" starting point for rollers
      krakenConfigFeederRoller.CurrentLimits.SupplyCurrentLimit = 40.0; 
      krakenConfigFeederRoller.CurrentLimits.SupplyCurrentLimitEnable = true;

      // These are "starting point" gains for a typical shooter
      krakenConfigFeederRoller.Slot0.kP = 0.11; // Proportional gain
      krakenConfigFeederRoller.Slot0.kV = 0.12; // Feedforward (Volts per RPS)
              


      // 3. Set Motor Inversion (Equivalent to launcherConfig.inverted(true))
      // Options: Clockwise_Positive or CounterClockwise_Positive
      krakenConfigFeederRoller.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // 4. Apply the config to your motor
      // This replaces the .configure(...) call in the Spark Max API
      StatusCode status = krakenFeederRoller.getConfigurator().apply(krakenConfigFeederRoller);

      // It is good practice to check if the config applied successfully
      if (!status.isOK()) {
          System.out.println("Could not apply config to Kraken: " + status.toString());
      } 


    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
  //   SparkMaxConfig launcherConfig = new SparkMaxConfig();
  //   launcherConfig.inverted(true);
  //   launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
  //   intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // 1. Create the configuration object
        TalonFXConfiguration krakenConfigIntakeLauncherRoller = new TalonFXConfiguration();

        // 2. Set the Current Limit (Supply current is what the battery sees)
        // In FRC, 40A is a standard "safe" starting point for rollers
        krakenConfigIntakeLauncherRoller.CurrentLimits.SupplyCurrentLimit = 40.0; 
        krakenConfigIntakeLauncherRoller.CurrentLimits.SupplyCurrentLimitEnable = true;
  
      // These are "starting point" gains for a typical shooter
      krakenConfigIntakeLauncherRoller.Slot0.kP = 0.11; // Proportional gain
      krakenConfigIntakeLauncherRoller.Slot0.kV = 0.12; // Feedforward (Volts per RPS)


        // 3. Set Motor Inversion (Equivalent to launcherConfig.inverted(true))
        // Options: Clockwise_Positive or CounterClockwise_Positive
        krakenConfigIntakeLauncherRoller.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  
        // 4. Apply the config to your motor
        // This replaces the .configure(...) call in the Spark Max API
        StatusCode statusKrakenIntakeLauncherRoller = krakenIntakeLauncherRoller.getConfigurator().apply(krakenConfigIntakeLauncherRoller);
  
        // It is good practice to check if the config applied successfully
        if (!status.isOK()) {
            System.out.println("Could not apply config to Kraken: " + statusKrakenIntakeLauncherRoller.toString());
        } 
   }


  //This method will set the velocity using the RPM as an input and an RPS as an output
   public double getVelocityRPS(double targetRPM) {
    // Convert RPM to RPS (Rotations Per Second)
    double targetRPS = targetRPM / 60.0;
    
   return targetRPS;
}


  // A method to set the rollers to values for intaking
  public void intake() {

    //we will set the feeder roller and launher rollers velocity.
    krakenFeederRoller.setControl(velocityRequest.withVelocity(INTAKING_FEEDER_RPS_SETPOINT));
    SmartDashboard.putNumber("Intaking feeder roller SETPOINT", INTAKING_FEEDER_RPS_SETPOINT);

    krakenIntakeLauncherRoller.setControl(velocityRequest.withVelocity(INTAKING_INTAKE_RPS_SETPOINT));
    SmartDashboard.putNumber("Intaking laumcher roller SETPOINT", INTAKING_INTAKE_RPS_SETPOINT);

  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    // krakenFeederRoller
    //     .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    // krakenIntakeLauncherRoller
    //     .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  
        krakenFeederRoller.setControl(velocityRequest.withVelocity(-1 * INTAKING_FEEDER_RPS_SETPOINT));
        SmartDashboard.putNumber("Intaking feeder roller SETPOINT", -1 * INTAKING_FEEDER_RPS_SETPOINT);

                krakenIntakeLauncherRoller.setControl(velocityRequest.withVelocity(-1 * INTAKING_INTAKE_RPS_SETPOINT));
        SmartDashboard.putNumber("Intaking intake roller SETPOINT", -1 * INTAKING_INTAKE_RPS_SETPOINT);

      }

  // A method to set the rollers to values for launching.
  public void launch() {

        //we will set the feeder roller and launher rollers velocity.
        krakenFeederRoller.setControl(velocityRequest.withVelocity(LAUNCHING_FEEDER_RPS_SETPOINT));
        SmartDashboard.putNumber("Intaking feeder roller SETPOINT", LAUNCHING_FEEDER_RPS_SETPOINT);
    
        krakenIntakeLauncherRoller.setControl(velocityRequest.withVelocity(LAUNCHING_LAUNCHER_RPS_SETPOINT));
        SmartDashboard.putNumber("Intaking intake roller SETPOINT", LAUNCHING_LAUNCHER_RPS_SETPOINT);

    // krakenFeederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    // krakenIntakeLauncherRoller
    //     .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A method to stop the rollers
  public void stop() {
    krakenFeederRoller.set(0);
    krakenIntakeLauncherRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    krakenFeederRoller.setControl(velocityRequest.withVelocity(SPIN_UP_FEEDER_RPS_SETPOINT));
   
    krakenIntakeLauncherRoller.setControl(velocityRequest.withVelocity(LAUNCHING_LAUNCHER_RPS_SETPOINT));
    SmartDashboard.putNumber("Launching launcher roller SETPOINT", LAUNCHING_LAUNCHER_RPS_SETPOINT);
     
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  // A command factory that stops all rollers and finishes immediately
public Command stopCommand() {
  return this.runOnce(() -> this.stop());
}

  // A command factory that will star the intake
  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  @Override
  public void periodic() {

     // 1. Always increment the counter
     loopCounter++;

     // 2. Only run this code every 5 loops
   if (loopCounter % UPDATE_INTERVAL == 0) {
    //We will print out the velocity of the two motors and the temperature in the SmartDashboard
    //where we can plot a graph to see how this is working.
    SmartDashboard.putNumber("Intaking FEEDER roller ACTUAL RPS VELOCITY", krakenFeederRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intaking INTAKE roller ACTUAL RPS VELOCITY", krakenIntakeLauncherRoller.getVelocity().getValueAsDouble());
      
    SmartDashboard.putNumber("Intaking FEEDER roller ACTUAL TEMP", krakenFeederRoller.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Intaking INTAKE roller ACTUAL TEMP", krakenIntakeLauncherRoller.getDeviceTemp().getValueAsDouble());
    

    /// Optional: Reset counter to prevent overflow (though ints take years to overflow)
       if (loopCounter > 100) loopCounter = 0; 
  }
} //end of periodic

  //Will increase the intake feeders by 5
  public void increaseFeederRPS() {

    INTAKING_FEEDER_RPS_SETPOINT = INTAKING_FEEDER_RPS_SETPOINT - 5;
    INTAKING_INTAKE_RPS_SETPOINT = INTAKING_INTAKE_RPS_SETPOINT + 5; 

  }


  //Will decrease the feeder RPS by 5
  public void decreaseFeederRPS() {
    INTAKING_FEEDER_RPS_SETPOINT = INTAKING_FEEDER_RPS_SETPOINT + 5;
    INTAKING_INTAKE_RPS_SETPOINT = INTAKING_INTAKE_RPS_SETPOINT - 5; 
  }


    //Will increase the intake feeders by 5
    public void increaseLauncherRPS() {
      System.out.println(" increasing LauncherRPD");

      LAUNCHING_FEEDER_RPS_SETPOINT = LAUNCHING_FEEDER_RPS_SETPOINT +  5;
      LAUNCHING_LAUNCHER_RPS_SETPOINT = LAUNCHING_LAUNCHER_RPS_SETPOINT + 5; 
  
    }
  
  
    //Will decrease the feeder RPS by 5
    public void decreaseLauncherRPS() {
      System.out.println(" decreasing LauncherRPD");
      LAUNCHING_FEEDER_RPS_SETPOINT = LAUNCHING_FEEDER_RPS_SETPOINT - 5;
      LAUNCHING_LAUNCHER_RPS_SETPOINT = LAUNCHING_LAUNCHER_RPS_SETPOINT - 5; 
    }



}
