package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import org.strykeforce.thirdcoast.swerve.MotorControllerConfig;
import org.strykeforce.thirdcoast.swerve.MotorControllerWrapper;
import org.strykeforce.thirdcoast.swerve.SparkMaxWrapper;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.TalonSRXWrapper;
import org.strykeforce.thirdcoast.swerve.Wheel;
import org.strykeforce.thirdcoast.swerve.MotorControllerConfig.AzimuthMotorController;
import org.strykeforce.thirdcoast.swerve.MotorControllerConfig.FeedbackSensor;
import org.strykeforce.thirdcoast.swerve.TalonSRXWrapper.*;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private static final double ROBOT_LENGTH = 1.;
    private static final double ROBOT_WIDTH = 1.;
    private static final double DRIVE_SETPOINT_MAX = 5500.;
    private static final int AZIMUTH_TICKS = 4096;

    /*
     * if wheels form an x pattern when only applying yaw (right x stick), change to
     * false: hardware dependent
     */
    private static final boolean INVERT_ERROR = true;

    private final SwerveDrive swerve = getSwerve();

    public DriveSubsystem() {
        swerve.setFieldOriented(true);
        zeroAzimuths();
    }

    /**
     * Generate the swerve drive configuration & return a swerve for it
     * 
     * @return A swerve drive based on the given configuration
     */
    private SwerveDrive getSwerve() {
        final var config = new SwerveDriveConfig();

        // Setup General Robot
        config.gyro = new AHRS(SPI.Port.kMXP);
        config.length = ROBOT_LENGTH;
        config.width = ROBOT_WIDTH;
        config.gyroLoggingEnabled = true;
        config.summarizeTalonErrors = true;

        // Configure our motor types
        config.azimuthConfig = new MotorControllerConfig(AzimuthMotorController.TALON_SRX,
                FeedbackSensor.CTRE_MAG_ENCODER);
        config.driveConfig = new MotorControllerConfig(AzimuthMotorController.SPARK_MAX, FeedbackSensor.CAN_CODER);

        // Drive Configuration
        config.driveConfig.continuousCurrentLimit = 40;
        config.driveConfig.peakCurrentLimit = 0;

        // Azimuth Config
        config.azimuthConfig.continuousCurrentLimit = 10;
        config.azimuthConfig.peakCurrentLimit = 0;
        config.azimuthConfig.slot0.kP = 10.0;
        config.azimuthConfig.slot0.kI = 0.0;
        config.azimuthConfig.slot0.kD = 100.0;
        config.azimuthConfig.slot0.kF = 0.0;
        config.azimuthConfig.slot0.kIZone = 0;
        config.azimuthConfig.slot0.kAllowableError = 0;
        config.azimuthConfig.motionAcceleration = 10_000;
        config.azimuthConfig.motionCruiseVelocity = 800;

        var wheels = new Wheel[4];

        // IDS to use for swerve motors
        final var dID = new int[] { 2, 3, 8, 1 };
        final var aID = new int[] { 7, 6, 5, 4 };

        // Create the swerve wheels
        for (int i = 0; i < 4; i++) {
            final var azimuth = new TalonSRXWrapper(config.azimuthConfig, aID[i]);
            final var drive = new SparkMaxWrapper(config.driveConfig, dID[i]);
            final var wheel = new Wheel(azimuth, drive, DRIVE_SETPOINT_MAX, AZIMUTH_TICKS, INVERT_ERROR);
            wheels[i] = wheel;
        }

        // Set the swerve drives wheels
        config.wheels = wheels;

        return new SwerveDrive(config);
    }

    /**
     * Zero the NavX Gyro
     */
    public void zeroGyro() {
        final var gyro = swerve.getGyro();
        gyro.setAngleAdjustment(0);
        var adj = gyro.getAngle() % 360;
        gyro.setAngleAdjustment(-adj);
    }

    /**
     * Drive the swerve by using teh fwd, str, and yaw of the joystick
     * 
     * @param fwd y axis of the swerive
     * @param str x axis of the swerve
     * @param yaw yaw of the swerve
     */
    public void drive(double fwd, double str, double yaw) {
        swerve.drive(fwd, str, yaw);
    }

    /**
     * Zero all of the azmiuths.
     */
    public void zeroAzimuths() {
        swerve.zeroAzimuthEncoders();
    }

    public void saveAzimuthPositions() {
        swerve.saveAzimuthPositions();
    }
}