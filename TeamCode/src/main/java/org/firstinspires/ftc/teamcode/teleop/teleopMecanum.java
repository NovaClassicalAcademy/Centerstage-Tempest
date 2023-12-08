package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//linear servo port 5
//grip 2 servo port 3
//grip 1 servo port 1
//flip servo port 2
//DO NOT USE PORT 4
@TeleOp(name = "Tele-Op")
public class teleopMecanum extends LinearOpMode {
    boolean flipped = true;
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        DcMotor liftMotor = hardwareMap.dcMotor.get("lift");
        DcMotor elbowMotor = hardwareMap.dcMotor.get("elbow");

        Servo grip1 = hardwareMap.servo.get("grip1");
        Servo grip2 = hardwareMap.servo.get("grip2");
        Servo flip = hardwareMap.servo.get("flip");
        Servo launcher = hardwareMap.servo.get("launcher");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Telemetry.Item liftPosition = telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
        liftPosition.setValue(liftMotor.getCurrentPosition());

        Telemetry.Item elbowPosition = telemetry.addData("Elbow Position", elbowMotor.getCurrentPosition());
        elbowPosition.setValue(elbowMotor.getCurrentPosition());

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.setAutoClear(true);
        telemetry.update();
        float driveSpeed = 0.7f;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double SLOWPOWERMULTIPLIER = 0.35;
            double liftPower = gamepad2.right_stick_y * 0.3;
            double elbowPower = gamepad2.left_stick_y * 0.1;

            elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(gamepad2.right_trigger > 0.01) {
                if(!flipped) {
                    grip1.setPosition(0.55);
                } else if (flipped) {
                    grip2.setPosition(0.35);
                }
            } else if(gamepad2.right_bumper) {
                if (!flipped) {
                    grip1.setPosition(0.27);
                } else if (flipped) {
                    grip2.setPosition(0.6);
                } else {
                    idle();
                }
            }
            if(gamepad2.left_bumper) {
                if (flipped) {
                    grip1.setPosition(0.27);
                } else if (!flipped) {
                    grip2.setPosition(0.6);
                } else {
                    idle();
                }
            }
            if(gamepad2.a) {
                flip.setPosition(0.6);
                flipped = true;
            }
            else if(gamepad2.x) {
                flip.setPosition(0.05);
                flipped = false;
            } else {
                idle();
            }
            if (elbowMotor.isBusy()) {
                liftPosition.setValue(liftMotor.getCurrentPosition());
                telemetry.update();
            } if(gamepad1.right_bumper) {
                launcher.setPosition(0.6);
            } else if(gamepad1.left_bumper) {
                launcher.setPosition(0.3);
            } else {
                idle();
            }
            if(gamepad1.left_trigger > 0.1) {
                backRight.setPower((backRightPower * driveSpeed) * SLOWPOWERMULTIPLIER);
                frontLeft.setPower((frontLeftPower * driveSpeed) * SLOWPOWERMULTIPLIER);
                frontRight.setPower((frontRightPower * driveSpeed) * SLOWPOWERMULTIPLIER);
                backLeft.setPower((backLeftPower * driveSpeed) * SLOWPOWERMULTIPLIER);
            } else {
                backRight.setPower(backRightPower * driveSpeed);
                frontLeft.setPower(frontLeftPower * driveSpeed);
                frontRight.setPower(frontRightPower * driveSpeed);
                backLeft.setPower(backLeftPower * driveSpeed);
            }

            liftMotor.setPower(liftPower);
            elbowMotor.setPower(elbowPower);
            elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Telemetry.Item liftPositioni = telemetry.addData("Lift Position", liftMotor.getCurrentPosition());

            Telemetry.Item elbowPositioni = telemetry.addData("Elbow Position", elbowMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}


