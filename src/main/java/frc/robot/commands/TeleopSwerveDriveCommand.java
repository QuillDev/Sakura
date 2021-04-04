package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopSwerveDriveCommand extends CommandBase {

    private static final double DEADBAND = 0.05;
    private static final DriveSubsystem DRIVE = RobotContainer.SWERVE_DRIVE;
    private static final Joystick CONTROLS = RobotContainer.CONTROLS;

    public TeleopSwerveDriveCommand() {
        addRequirements(DRIVE);
    }

    @Override
    public void execute() {
        double fwd = deadband(CONTROLS.getRawAxis(1));
        double str = deadband(-CONTROLS.getRawAxis(0));
        double yaw = deadband(-CONTROLS.getRawAxis(4));

        DRIVE.drive(fwd, str, yaw);
    }

    @Override
    public void end(boolean interrupted) {
        DRIVE.drive(0., 0., 0.);
    }

    private double deadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0.;
        }
        return value;
    }
}
