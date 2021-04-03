package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final double DEADZONE = .05; //Configure the deadzone for input when driving

    public void drive(double fwd, double str, double rcw, boolean fieldOriented){

        //Apply deadzones to the speeds
        if(Math.abs(fwd) <= DEADZONE) { fwd = 0; }
        if(Math.abs(str) <= DEADZONE) { str = 0; }
        if(Math.abs(rcw) <= DEADZONE) { rcw = 0; }
        
        System.out.printf("FWD:%s | STR:%s | RCW: %s | FIELD: %s\n", fwd, str, rcw, fieldOriented);
    }
}
