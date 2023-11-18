package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class Lift extends SubsystemBase {
    private final DcMotorEx lift;
    float Multiplier = 0.6f;
    public Lift(DcMotorEx lift) {
        this.lift = lift;
        lift.setDirection(DcMotor.Direction.REVERSE);
        }
    public void liftMovement(double liftMotorPower) {
        lift.setPower(liftMotorPower);
    }
    public void UpWithEncoder(int SCORINGPOSITION, float speed) {
        lift.setPower(speed);
        lift.getCurrentPosition();
        lift.setTargetPosition(SCORINGPOSITION);
    }
    public void DownWithEncoder() {
        lift.setPower(0.50);
    }
}