package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ftcLibMecanumDrive extends LinearOpMode {
    static final boolean FIELD_CENTRIC = false;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_435)
        );

        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {



            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX(),
                    false
            );
        }
    }
}

