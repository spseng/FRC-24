// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// REV imports

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
 class Swerve {
    private final RelativeEncoder encoder;
    private final int steerPort;
    private final int drivePort;
    private final double ang=0; 
    private final double offset; // radians
    private final CANSparkMax steerMotor;
    private final  CANSparkMax driveMotor;
    public void main ( int steerArg, int driveArg, double off){
        steerPort = steerArg;
        drivePort = driveArg;
        offset = off;
        steerMotor = new CANSparkMax(steerPort,MotorType.kBrushless);
        driveMotor = new CANSparkMax(drivePort,MotorType.kBrushless);
    }
    private final double kp = 2.;
    private final double ki=0.1;
    private final double kd = 0.1;
    private final PIDController PID = new PIDController(kp, ki, kd);
   
    public void idle() {
        steerMotor.set(PID.calculate(encoder.getPosition(), offset/Math.PI));
        
    }
    public void steer(double refAng){
        if (Math.abs(refAng-ang)>Math.abs(refAng-ang-2*Math.PI)){
            steerMotor.set(PID.calculate(steerMotor.getEncoder().getPosition(), (ang+(ang-refAng-2*Math.PI))/Math.PI));
            ang = steerMotor.getEncoder().getPosition()*Math.PI;
        } else {
            steerMotor.set(PID.calculate(br_steer_steerMotor.getEncoder().getPosition(), refAng/Math.PI));
            ang = steerMotor.getEncoder().getPosition()*Math.PI;

        }
        
    }

}

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private XboxController m_stick;
  
  private CANSparkMax bl_steer;
  private CANSparkMax fl_steer;
  private CANSparkMax fr_steer;
  private CANSparkMax br_steer;
  
  private RelativeEncoder bl_steer_enc;
  private RelativeEncoder fl_steer_enc;
  private RelativeEncoder fr_steer_enc;
  private RelativeEncoder br_steer_enc;

  private double kp= 2;
  private double ki= 0.1;
  private double kd= 0.1;


  private PIDController fr_pid = new PIDController(kp, ki, kd);
  private PIDController fl_pid = new PIDController(kp, ki, kd);
  private PIDController bl_pid = new PIDController(kp, ki, kd);
  private PIDController br_pid = new PIDController(kp, ki, kd);
  // br_pid.setTolerance(0.2, 0.10);

  private CANSparkMax bl_motor;
  private CANSparkMax fl_motor;
  private CANSparkMax fr_motor;
  private CANSparkMax br_motor;
  private RelativeEncoder m_encoder;
  private RobotContainer m_robotContainer;
  private Swerve BR;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.d
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //eventually create motor class for each thing
    br_steer=new CANSparkMax(1,MotorType.kBrushless);
    fr_steer=new CANSparkMax(2,MotorType.kBrushless);
    fl_steer=new CANSparkMax(3,MotorType.kBrushless);
    bl_steer=new CANSparkMax(4,MotorType.kBrushless);
    br_steer_enc=br_steer.getEncoder();
    fr_steer_enc=fr_steer.getEncoder();
    fl_steer_enc=fl_steer.getEncoder();
    bl_steer_enc=bl_steer.getEncoder();
    // br_steer_enc.setPosition(0);
    // fr_steer_enc.setPosition(0);
    // fl_steer_enc.setPosition(0);
    // bl_steer_enc.setPosition(0);
    // br_steer_enc=bl_steer.getEncoder();
    // fr_steer_enc=fl_steer.getEncoder();
    // fl_steer_enc=fr_steer.getEncoder();
    // bl_steer_enc=br_steer.getEncoder();

    br_motor=new CANSparkMax(5,MotorType.kBrushless);
    fr_motor=new CANSparkMax(6,MotorType.kBrushless);
    fl_motor=new CANSparkMax(7,MotorType.kBrushless);
    bl_motor=new CANSparkMax(8,MotorType.kBrushless);
    // m_motor.restoreFactoryDefaults(0);

    m_stick = new XboxController(0);
    bl_pid.setTolerance(0.5, 0.5);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


  }

  @Override
  public void teleopInit() {
    BR = new Swerve(1,5);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  private double prevtheta=0;
  private double theta = 0;
  /** This function is called periodically during operator control. */
  private double closest(double a, double b){
    double dir = (b%2) - (a%2);
    if( Math.abs(dir)>1){
       dir = -Math.signum(dir)*2 + dir;
    }
    return dir;
  }
  @Override
  public void teleopPeriodic() {
    double x = m_stick.getLeftX();
    double y = m_stick.getLeftY();
    double r = 0.5*Math.sqrt( Math.pow(x,2) + Math.pow(y,2) );
    theta = Math.atan2(y,-x);
    BR.steer(theta);
    // bl_motor.set(-r);
    // fl_motor.set(r);
    // fr_motor.set(r);
    // br_motor.set(r);
    // if(Math.abs(r)>0.3){
    // br_steer.set(br_pid.calculate(br_steer_enc.getPosition(), closest(theta+0+.5, prevtheta+0+.5)));
    // fr_steer.set(fr_pid.calculate(fr_steer_enc.getPosition(), closest(theta+.238+.5, prevtheta+.238+.5)));
    // bl_steer.set(bl_pid.calculate(bl_steer_enc.getPosition(), closest(theta-.34+.5, prevtheta-.34+.5)));
    // fl_steer.set(fl_pid.calculate(fl_steer_enc.getPosition(), closest(theta-.107+.5, prevtheta-.107+.5)));
    // } else {
    //   br_steer.set(0);
    //   fr_steer.set(0);
    //   bl_steer.set(0);
    //   fl_steer.set(0);
    // }
    // if(m_stick.getAButton()){


    //   br_steer.set(br_pid.calculate(br_steer_enc.getPosition(), 0));
    //   fr_steer.set(fr_pid.calculate(fr_steer_enc.getPosition(), .238));
    //   bl_steer.set(bl_pid.calculate(bl_steer_enc.getPosition(),-.340));
    //   fl_steer.set(fl_pid.calculate(fl_steer_enc.getPosition(), -.107));
    // }

    // SmartDashboard.putNumber("BR Position", br_steer_enc.getPosition());
    // SmartDashboard.putNumber("BL Position", bl_steer_enc.getPosition());
    // SmartDashboard.putNumber("FR Position", fr_steer_enc.getPosition());
    // SmartDashboard.putNumber("FL Position", fl_steer_enc.getPosition());
    // SmartDashboard.putNumber("Theta", theta);
    // SmartDashboard.putNumber("R", r);
    // SmartDashboard.putNumber("X", x);
    // SmartDashboard.putNumber("Y", y);

    // SmartDashboard.putNumber("Encoder Velocity", br_steer_enc.getVelocity());
    // prevtheta=theta;
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
