package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//linear servo port 5
//grip 2 servo port 3
//grip 1 servo port 1
//flip servo port 2
//DO NOT USE PORT 4
@TeleOp(name = "Tele-Op")
public class teleopMecanum extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    boolean flipped = true;
    boolean driveToggle = true;
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

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, //adjust this
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double SLOWPOWERMULTIPLIER = 0.35;
            double liftPower = gamepad2.right_stick_y * 0.3;
            double elbowPower = gamepad2.left_stick_y * 0.1;


            elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (currentGamepad1.options && !previousGamepad1.options) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                driveToggle = !driveToggle;
            }

            if (driveToggle) {
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // remove if not needed

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                if(gamepad1.left_trigger > 0.1){
                    frontLeft.setPower(frontLeftPower * SLOWPOWERMULTIPLIER);
                    backLeft.setPower(backLeftPower * SLOWPOWERMULTIPLIER);
                    frontRight.setPower(frontRightPower *SLOWPOWERMULTIPLIER);
                    backRight.setPower(backRightPower * SLOWPOWERMULTIPLIER);
                } else {
                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);
                }
            }
            else {
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                denominator = denominator * 0.1;
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                if(gamepad1.left_trigger > 0.1){
                    frontLeft.setPower(frontLeftPower * SLOWPOWERMULTIPLIER);
                    backLeft.setPower(backLeftPower * SLOWPOWERMULTIPLIER);
                    frontRight.setPower(frontRightPower *SLOWPOWERMULTIPLIER);
                    backRight.setPower(backRightPower * SLOWPOWERMULTIPLIER);
                } else {
                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);
                }
            }

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
            liftMotor.setPower(liftPower);
            elbowMotor.setPower(elbowPower);
            elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.update();
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        }
    }
}


