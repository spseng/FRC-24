// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// REV imports

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.VisionSystem;

import static frc.robot.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private XboxController m_stick;
    private XboxController m_stick_2;

    private AutonomousController autonomousController;
    private TeleopController teleopController;
    private VisionSystem visionSystem;
    private Drivetrain drivetrain;
    private ShooterSystem shooterSystem;

    // private CameraSe/rver camera;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.d
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        CameraServer.startAutomaticCapture();

        m_stick = new XboxController(0);
        m_stick_2 = new XboxController(1);

        teleopController = new TeleopController(false);
        visionSystem = new VisionSystem();

        drivetrain = new Drivetrain();
        shooterSystem = new ShooterSystem(
                INTAKE_MOTOR_CAN,
                LOADING_MOTOR_CAN,
                SHOOTER_MOTOR_CAN,
                ANGLE_ALIGNMENT_MOTOR_CAN,
                ANGLE_ALIGNMENT_ENCODER_CAN,
                SHOOTER_IS_LOADED_BUTTON_ID,
                IS_LOWEST_ANGLE_BUTTON_ID,
                IS_HIGHEST_ANGLE_BUTTON_ID
        );
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

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousController.init();
        shooterSystem.calibrate();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        visionSystem.periodic();
        drivetrain.periodic();
        visionSystem.updateDrivetrainPosition(drivetrain);

        shooterSystem.periodic(kDefaultPeriod);
        autonomousController.periodic(drivetrain, shooterSystem, visionSystem);
    }

    @Override
    public void teleopInit() {
        teleopController = new TeleopController(true);
        teleopController.init(drivetrain);
        shooterSystem.calibrate();
    }

    @Override
    public void teleopPeriodic() {
        visionSystem.periodic();
        drivetrain.periodic();
        visionSystem.updateDrivetrainPosition(drivetrain);

        shooterSystem.periodic(kDefaultPeriod);
        teleopController.periodic(m_stick, drivetrain, shooterSystem, visionSystem);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        teleopController = new TeleopController(false);
        teleopController.init(drivetrain);

        shooterSystem.calibrate();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        visionSystem.periodic();
        drivetrain.periodic();
        visionSystem.updateDrivetrainPosition(drivetrain);

        shooterSystem.periodic(kDefaultPeriod);
        teleopController.periodic(m_stick_2, drivetrain, shooterSystem, visionSystem);
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
