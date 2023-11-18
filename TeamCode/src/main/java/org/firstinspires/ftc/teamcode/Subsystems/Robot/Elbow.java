package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class Elbow extends SubsystemBase {
    public MotorEx elbow;
    public Elbow(MotorEx elbow) {
        this.elbow = elbow;
    }
    public void ElbowMovement(double elbowMotorPower) {
        elbow.set(elbowMotorPower);
    }
}