package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Lift extends SubsystemBase {
    public Motor lift;
    public GamepadEx scoreOP;
    float Multiplier = 0.6f;
    public double liftPower = scoreOP.getLeftY();
    public Lift(HardwareMap hardwareMap) {
        this.lift = new Motor(hardwareMap, "lift");
    }
    public void liftMovement() {
        lift.set(liftPower * Multiplier);
    }
    public void UpWithEncoder(int SCORINGPOSITION, float speed) {
        lift.set(speed);
        lift.getCurrentPosition();
        lift.setTargetPosition(SCORINGPOSITION);
    }
    public void DownWithEncoder() {
        lift.set(0.50);
    }
}