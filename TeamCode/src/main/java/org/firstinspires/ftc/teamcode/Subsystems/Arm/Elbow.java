package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Elbow extends SubsystemBase {
    public SimpleServo elbow;
    public int scoringPosition = 0; //need to calculate
    public int groundPosition = 1; //need to calculate
    public Elbow(HardwareMap hardwareMap) {
        this.elbow = new SimpleServo(hardwareMap, "elbow", 0.0, 180.0);
    }
    public Command servoScore() {
        elbow.setPosition(scoringPosition);
        return null;
    }
    public Command servoGround() {
        elbow.setPosition(groundPosition);
        return null;
    }
}