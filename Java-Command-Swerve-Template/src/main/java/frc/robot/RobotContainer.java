// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final XboxController mController = new XboxController(Constants.kControllerPort);
    private final Swerve mSwerve = new Swerve();

    private final ManualDrive mManualDriveCommand = new ManualDrive(mSwerve, mController);

    // Auton
    private PIDController mXController = new PIDController(
        SwerveConstants.kPathingX_kP, 
        SwerveConstants.kPathingX_kI, 
        SwerveConstants.kPathingX_kD
    );
    private PIDController mYController = new PIDController(
        SwerveConstants.kPathingY_kP, 
        SwerveConstants.kPathingY_kI, 
        SwerveConstants.kPathingY_kD
    );
    private PIDController mThetaController = new PIDController(
        SwerveConstants.kPathingTheta_kP, 
        SwerveConstants.kPathingTheta_kI, 
        SwerveConstants.kPathingTheta_kD
    );

    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath(
        "New Path", 
        SwerveConstants.kMaxVelocityMetersPerSecond, 
        SwerveConstants.kMaxAccelerationMetersPerSecond
    );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        mSwerve.setDefaultCommand(mManualDriveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            mTrajectory, 
            mSwerve::getPose, 
            SwerveConstants.kSwerveKinematics, 
            mXController, 
            mYController, 
            mThetaController, 
            mSwerve::setModuleStates, 
            mSwerve
        );
        return command.andThen(() -> mSwerve.drive(0, 0, 0, false));
    }
}
