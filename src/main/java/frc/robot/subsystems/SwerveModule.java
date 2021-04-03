package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Helper.Gains;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax drive;
    private final TalonSRX steer;

    // Gains
    final Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);

    SwerveModule(int drivePort, int steerPort) {
        this.drive = new CANSparkMax(drivePort, MotorType.kBrushless);
        this.steer = new TalonSRX(steerPort);
    }

    public void init() {
        drive.restoreFactoryDefaults();
        steer.configFactoryDefault();

        steer.setInverted(false);
        steer.setSensorPhase(false);

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

        // reset the given sensor position
        steer.setSelectedSensorPosition(0, Constants.STEER_PID, Constants.TIMEOUT);
    }

    public Rotation2d getAngle() {
        final var units = steer.getSelectedSensorPosition(0);
        final var angle360 = ((units % 4096) / 4096 * 360);
        final var angle180 = ((angle360 + 180) % 360) - 180;
        return Rotation2d.fromDegrees(angle180);
    }
}