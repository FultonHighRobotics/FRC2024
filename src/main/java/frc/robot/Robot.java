// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  RobotContainer m_robotContainer;

  private final CANSparkMax intakeMotor1 = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax intakeMotor2 = new CANSparkMax(12, MotorType.kBrushless);

  private final CANSparkMax climberMotorL = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax climberMotorR = new CANSparkMax(9, MotorType.kBrushless);

  private final CANSparkMax ampIntake = new CANSparkMax(13, MotorType.kBrushless);

  private Timer disabledTimer;

  public final float intakeSpeed = 0.2f;
  public final float launchSpeed = 1f;
  public final float launchPrimeTime = 0.4f;

  public final double climbSpeed = 0.5;
  public final double climbDownSpeed = -0.2;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    intakeMotor1.setSmartCurrentLimit(90, 40, 6001);
    intakeMotor2.setSmartCurrentLimit(90, 40, 6001);

    ampIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
  }

  public void setMotorBrake(boolean brake)
  {
    //intakeMotor1.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    //intakeMotor2.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);


    
    isPrimed = false;
  }

  private void setClimberMotors(double speed)
  {
    climberMotorL.set(-speed);
    climberMotorR.set(speed);
  }

  private void intakeNote(){
    intakeMotor1.set(-intakeSpeed);
    intakeMotor2.set(-intakeSpeed);

    ampIntake.set(0.3);
  }

  public boolean isPrimed;
  public double primeTimestamp;
  public void primeLauncher()
  {
    intakeMotor1.set(launchSpeed);
    // launches it when primed

  }

  private void launchNote()
  {
    ampIntake.set(-1);
    intakeMotor1.set(launchSpeed);
    intakeMotor2.set(launchSpeed);
      
  }


  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();

    climberMotorL.setIdleMode(CANSparkMax.IdleMode.kCoast);
    climberMotorR.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    climberMotorL.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climberMotorR.setIdleMode(CANSparkMax.IdleMode.kBrake);

    climberMotorL.setSmartCurrentLimit(45);
    climberMotorR.setSmartCurrentLimit(45);
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);

    
  }

  float launchCountdown = 0;

  boolean alter = false;
  public void handleInputs(){
    double leftTrigger = m_robotContainer.driverXbox.getLeftTriggerAxis(), 
    rightTrigger = m_robotContainer.driverXbox.getRightTriggerAxis();
    
    
    if (m_robotContainer.driverXbox.getLeftBumper())
    {
      intakeNote();
    }
    else{
      intakeMotor1.set(0);
      intakeMotor2.set(0);
      ampIntake.set(0);
    }

    if (leftTrigger > 0.15){
      // primes the launcher while the trigger is held, then launches when released
      launchCountdown = 1f;
      primeLauncher();
    }
    else if (launchCountdown > 0){
      launchCountdown -= 0.05;
      launchNote();
    }
    
    if (rightTrigger > 0.1){
      m_robotContainer.swerveSpeed = 0.73f;
    }
    else{
      m_robotContainer.swerveSpeed = 1.3f;
    }

    if (m_robotContainer.driverXbox.getLeftStickButton()){
      setClimberMotors(0.4);
    }

    else if (m_robotContainer.driverXbox.getRightStickButton()){
      setClimberMotors(-0.4);
    }
    else{
      setClimberMotors(0);
    }

    if (m_robotContainer.driverXbox.getBackButtonPressed())
    {
      alter = !alter;
    }
  }


  @Override
  public void teleopPeriodic()
  {
    handleInputs();
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }
}
