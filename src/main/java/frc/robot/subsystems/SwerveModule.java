package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Helper.Gains;

public class SwerveModule extends SubsystemBase {

    public final String name;
    private final CANSparkMax drive;
    private final TalonSRX steer;

    // Gains
    final Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);

    SwerveModule(String name, int drivePort, int steerPort, boolean invertDrive, boolean invertSteer,
            boolean invertPhase) {

        this.name = name;
        this.drive = new CANSparkMax(drivePort, MotorType.kBrushless);
        this.steer = new TalonSRX(steerPort);

        this.init(invertDrive, invertSteer, invertPhase);
    }

    public void init(boolean invertDrive, boolean invertSteer, boolean invertPhase) {

        // Setup drive
        drive.restoreFactoryDefaults();
        drive.setInverted(invertDrive);

        // Setup Steer
        steer.configFactoryDefault();

        steer.setInverted(invertSteer);
        steer.setSensorPhase(invertPhase);

        steer.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        steer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        steer.configNominalOutputForward(0, Constants.TIMEOUT);
        steer.configNominalOutputReverse(0, Constants.TIMEOUT);
        steer.configPeakOutputForward(1, Constants.TIMEOUT);
        steer.configPeakOutputReverse(-1, Constants.TIMEOUT);

        // Config allowable error
        steer.configAllowableClosedloopError(Constants.STEER_PID, 0, Constants.TIMEOUT);

        // setup pid values
        steer.config_kF(Constants.STEER_PID, kGains.kF, Constants.TIMEOUT);
        steer.config_kP(Constants.STEER_PID, kGains.kP, Constants.TIMEOUT);
        steer.config_kI(Constants.STEER_PID, kGains.kI, Constants.TIMEOUT);
        steer.config_kD(Constants.STEER_PID, kGains.kD, Constants.TIMEOUT);

        steer.selectProfileSlot(0, Constants.STEER_PID);
        steer.configNominalOutputForward(0, Constants.TIMEOUT);
        steer.configNominalOutputReverse(0, Constants.TIMEOUT);
        steer.configPeakOutputForward(1, Constants.TIMEOUT);
        steer.configPeakOutputReverse(-1, Constants.TIMEOUT);
        steer.configAllowableClosedloopError(0, Constants.STEER_PID, Constants.TIMEOUT);
        steer.configMotionCruiseVelocity(15000, Constants.TIMEOUT);
        steer.configMotionAcceleration(16000, Constants.TIMEOUT); //TODO: Config the motion acceleration etc.
        // reset the given sensor position
        steer.setSelectedSensorPosition(0, Constants.STEER_PID, Constants.TIMEOUT);
    }

    /**
     * Set the state of the swerve module to the given one
     * @param state to set the module to
     */
    public void setState(SwerveModuleState state){
        steer.set(ControlMode.MotionMagic, state.angle.getDegrees() / 360 * 4096);
        //TODO: Implement actual velocity driving
    }

    /**
     * Get the current state of the swerve module
     * @return the state of the module
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(0, getAngle()); //TODO: Add get velocity to this
    }

    /**
     * Get the raw angle of the module
     * @return a rotation2d between -infinity to +infinity degrees
     */
    public Rotation2d getRawAngle() {
        final var unit = steer.getSelectedSensorPosition(0);
        return Rotation2d.fromDegrees((((unit / 4096) * 360)));
    }

    /**
     * Get an angle between 0 and 360 degrees.
     * @return a rotation 2d between 0 and 360 degrees.
     */
    public Rotation2d getAngle360() {
        var raw = getRawAngle().getDegrees();
        return Rotation2d.fromDegrees(((raw % 360) + 360) % 360);
    }

    /**
     * Get an angle between -180 and 180 degrees.
     * @return a rotation 2d between -180 and 180 degrees
     */
    public Rotation2d getAngle() {
        var angle = getAngle360().getDegrees();

        if (angle > 180) {
            angle -= 360;
        }

        return Rotation2d.fromDegrees(angle);
    }
}
