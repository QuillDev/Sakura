package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final double DEADZONE = .05; // Configure the deadzone for input when driving

    // Creation of swerve modules
    private final SwerveModule fl = new SwerveModule("Font Left", 2, 7, false, false, true);
    private final SwerveModule fr = new SwerveModule("Front Right", 3, 6, false, false, false);
    private final SwerveModule bl = new SwerveModule("Back Left", 8, 5, false, false, false);
    private final SwerveModule br = new SwerveModule("Back Right", 1, 4, false, false, false);
    private final SwerveModule[] modules = { fl, fr, bl, br };

    //Create the gyroscope
    private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

    /**
     * Create a new swerve drive subsystem.
     */
    public SwerveDrive(){
        this.ahrs.reset(); //Reset the gyro back to zero
    }

    /**
     * Drive the swerve drive using the Joystick inputs.
     * @param fwd speed on Y axis
     * @param str speed on X axis
     * @param rcw rotation over origin
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

        // Put Swerve Joystick values on the dashboard.
        SmartDashboard.putNumber("FWD", fwd);
        SmartDashboard.putNumber("STR", str);
        SmartDashboard.putNumber("RCW", rcw);
        SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);

        // Iterate through all modules and print out their angles
        for (final var module : modules) {
            final var angle = module.getAngle();
            SmartDashboard.putNumber(module.name + " ANGLE", angle.getDegrees());
        }
    }

    /**
     * Get the gyroscope angle from the Swerve subsystem
     * @return the gyroscope angle of the swerve
     */
    public double getGyroAngle(){
        return this.ahrs.getAngle();
    }
}
