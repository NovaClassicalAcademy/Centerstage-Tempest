package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Optional;

//test
@TeleOp(name = "Tele-Op")
public class teleopMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo intakeServo = hardwareMap.servo.get("intakeServo");
        CRServo hopUpServo = hardwareMap.crservo.get("hopUpServo");
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "myanaloginput");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setAutoClear(true);
        telemetry.update();
        float driveSpeed = 1;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            float intakeMotorPower = gamepad1.left_trigger;
            float outtakeMotorPower = gamepad1.right_trigger;
            double position = analogInput.getVoltage() / 3.3 * 360;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower * driveSpeed);
            backLeft.setPower(backLeftPower * driveSpeed);
            frontRight.setPower(frontRightPower * driveSpeed);
            backRight.setPower(backRightPower * driveSpeed);

            if (gamepad1.left_trigger > 0) {
                intakeMotor.setPower(0.75 * (-intakeMotorPower));
                hopUpServo.setPower(-0.6);

            } else if (gamepad1.right_trigger > 0) {
                intakeMotor.setPower(0.75 * (outtakeMotorPower));
                hopUpServo.setPower(0.6);
            } else {
                intakeMotor.setPower(0);
                hopUpServo.setPower(0);
            }
            if(gamepad1.dpad_up) {
                intakeServo.setPosition(0.45);
            } else if (gamepad1.dpad_down) {
                intakeServo.setPosition(0.53);
            }
        }
    }
}


