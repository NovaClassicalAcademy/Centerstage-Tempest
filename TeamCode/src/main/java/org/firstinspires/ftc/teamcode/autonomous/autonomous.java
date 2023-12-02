package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivetrain;

@Autonomous
public class autonomous extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        DcMotor liftMotor = hardwareMap.dcMotor.get("lift");
        DcMotor elbowMotor = hardwareMap.dcMotor.get("elbow");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN, //adjust this
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        Forward(50);

        int frontRightPosition = frontRight.getCurrentPosition();
        int frontLeftPosition = frontLeft.getCurrentPosition();
        int backLeftPosition = backLeft.getCurrentPosition();
        int backRightPosition = backRight.getCurrentPosition();
        int robotPosition = (frontRightPosition+frontLeftPosition+backLeftPosition+backRightPosition)/4;
        int positionError;


        while(opModeIsActive()){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            frontRightPosition = frontRight.getCurrentPosition();
            frontLeftPosition = frontLeft.getCurrentPosition();
            backLeftPosition = backLeft.getCurrentPosition();
            backRightPosition = backRight.getCurrentPosition();
            robotPosition = (frontRightPosition+frontLeftPosition+backLeftPosition+backRightPosition)/4;
            positionError = 3759 - robotPosition;

            telemetry.addData("frontLeftPosition", frontLeftPosition);
            telemetry.addData("frontRightPosition", frontRightPosition);
            telemetry.addData("backLeftPosition", backLeftPosition);
            telemetry.addData("backRightPosition", backRightPosition);
            telemetry.addData("robotPosition", robotPosition);
            telemetry.addData("Error", positionError);
            telemetry.update();
        }
    }
    public void Forward (int distance) {
        distance = (int) (distance*62.65f);
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int frontRightPosition = frontRight.getCurrentPosition();
        int frontLeftPosition = frontLeft.getCurrentPosition();
        int backLeftPosition = backLeft.getCurrentPosition();
        int backRightPosition = backRight.getCurrentPosition();

        int robotPosition = (frontRightPosition+frontLeftPosition+backLeftPosition+backRightPosition)/4;
        int positionError;

        while (distance - robotPosition > 100){
            //751.8ticks per rev
            frontLeft.setPower(0.3);
            backRight.setPower(0.3);
            backLeft.setPower(0.3);
            frontRight.setPower(0.3);

            frontRightPosition = frontRight.getCurrentPosition();
            frontLeftPosition = frontLeft.getCurrentPosition();
            backLeftPosition = backLeft.getCurrentPosition();
            backRightPosition = backRight.getCurrentPosition();
            robotPosition = (frontRightPosition+frontLeftPosition+backLeftPosition+backRightPosition)/4;
            positionError = distance - robotPosition;

            telemetry.addData("frontLeftPosition", frontLeftPosition);
            telemetry.addData("frontRightPosition", frontRightPosition);
            telemetry.addData("backLeftPosition", backLeftPosition);
            telemetry.addData("backRightPosition", backRightPosition);
            telemetry.addData("robotPosition", robotPosition);
            telemetry.addData("Error", positionError);
            telemetry.update();
        }
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        sleep(100);
        while (distance - robotPosition > 50){
            //751.8ticks per rev
            frontLeft.setPower(0.2);
            backRight.setPower(0.2);
            backLeft.setPower(0.2);
            frontRight.setPower(0.2);

            frontRightPosition = frontRight.getCurrentPosition();
            frontLeftPosition = frontLeft.getCurrentPosition();
            backLeftPosition = backLeft.getCurrentPosition();
            backRightPosition = backRight.getCurrentPosition();
            robotPosition = (frontRightPosition+frontLeftPosition+backLeftPosition+backRightPosition)/4;
            positionError = distance - robotPosition;

            telemetry.addData("frontLeftPosition", frontLeftPosition);
            telemetry.addData("frontRightPosition", frontRightPosition);
            telemetry.addData("backLeftPosition", backLeftPosition);
            telemetry.addData("backRightPosition", backRightPosition);
            telemetry.addData("robotPosition", robotPosition);
            telemetry.addData("Error", positionError);
            telemetry.update();
        }
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        sleep(100);
        while (distance - robotPosition > 2){
            //751.8ticks per rev
            frontLeft.setPower(0.05);
            backRight.setPower(0.05);
            backLeft.setPower(0.05);
            frontRight.setPower(0.05);

            frontRightPosition = frontRight.getCurrentPosition();
            frontLeftPosition = frontLeft.getCurrentPosition();
            backLeftPosition = backLeft.getCurrentPosition();
            backRightPosition = backRight.getCurrentPosition();
            robotPosition = (frontRightPosition+frontLeftPosition+backLeftPosition+backRightPosition)/4;
            positionError = distance - robotPosition;

            telemetry.addData("frontLeftPosition", frontLeftPosition);
            telemetry.addData("frontRightPosition", frontRightPosition);
            telemetry.addData("backLeftPosition", backLeftPosition);
            telemetry.addData("backRightPosition", backRightPosition);
            telemetry.addData("robotPosition", robotPosition);
            telemetry.addData("Error", positionError);
            telemetry.update();
        }
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
    }
}