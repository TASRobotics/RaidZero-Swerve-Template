package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ManualDrive extends CommandBase {
    private final Swerve mSwerve;
    private final XboxController mController;
    
    public ManualDrive(Swerve drive, XboxController controller) {
        mSwerve = drive;
        mController = controller;

        // Adds the Swerve subsystem as a requirement to the command
        addRequirements(mSwerve);
    }

    @Override
    public void execute() {
        mSwerve.drive(mController.getLeftY(), mController.getLeftX(), mController.getRightX(), true);
    }
}
