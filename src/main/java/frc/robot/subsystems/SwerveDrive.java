package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final double DEADZONE = .05; // Configure the deadzone for input when driving

    // Creation of swerve modules
    private final SwerveModule fl = new SwerveModule("Font Left", 2, 7, false, false, true);
    private final SwerveModule fr = new SwerveModule("Front Right", 3, 6, false, false, true);
    private final SwerveModule bl = new SwerveModule("Back Left", 8, 5, false, false, true);
    private final SwerveModule br = new SwerveModule("Back Right", 1, 4, false, false, true);
    private final SwerveModule[] modules = { fl, fr, bl, br };

    // Create the gyroscope
    private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

    // Generate kinematics
    private final double trackWidth = 0.4826;
    private final double wheelBase = 0.5334;
    private final double maxSpeed = 3.5;
    private final double speedLimit = 1;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2), new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2), new Translation2d(-wheelBase / 2, -trackWidth / 2));

    /**
     * Create a new swerve drive subsystem.
     */
    public SwerveDrive() {
        this.ahrs.reset(); // Reset the gyro back to zero
    }

    /**
     * Drive the swerve drive using the Joystick inputs.
     * 
     * @param fwd           speed on Y axis
     * @param str           speed on X axis
     * @param rcw           rotation over origin
     * @param fieldOriented whether to drive using field oriented control mode.
     */
    public void drive(double fwd, double str, double rcw, boolean fieldOriented) {

        // Apply deadzones to the speeds
        if (Math.abs(fwd) <= DEADZONE) {
            fwd = 0;
        }
        if (Math.abs(str) <= DEADZONE) {
            str = 0;
        }
        if (Math.abs(rcw) <= DEADZONE) {
            rcw = 0;
        }

        // Generate the swerve module states depending on whether we're currently using
        // field oriented control
        var states = kinematics.toSwerveModuleStates(
                fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rcw, getGyroAngle())
                        : new ChassisSpeeds(fwd, str, rcw));

        // Normalize the wheel speeds using the speed limit
        SwerveDriveKinematics.normalizeWheelSpeeds(states, maxSpeed * speedLimit);

        // Iterate through all modules and print out their angles
        for (var index = 0; index < modules.length; index++) {
            final var module = modules[index];
            final var angle = module.getAngle();
            final var state = states[index];
            SmartDashboard.putNumber(module.name + " ANGLE", angle.getDegrees());
            
            //Set the module state to an optimized state
            module.setState(SwerveModuleState.optimize(state, module.getAngle()));
        }

        // Put Swerve Joystick values on the dashboard.
        SmartDashboard.putNumber("FWD", fwd);
        SmartDashboard.putNumber("STR", str);
        SmartDashboard.putNumber("RCW", rcw);
        SmartDashboard.putNumber("Gyro Angle", getGyroAngle().getDegrees());
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
    }

    /**
     * Get the gyroscope angle from the Swerve subsystem
     * 
     * @return the gyroscope angle of the swerve
     */
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-Math.IEEEremainder(this.ahrs.getAngle(), 360));
    }
}
