package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class MecanumDriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final RevIMU imu;
    public static int joystickTransformFactor = 30;
    public static double slowFactor = 2.5;
    private ElapsedTime time = new ElapsedTime();

    public MecanumDriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu) {
        this.imu = imu;
        fL.setInverted(true);
        bL.setInverted(true);
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }
    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed, DoubleSupplier gyroAngle) {
        return new RunCommand(
                () -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                        turnSpeed.getAsDouble(), gyroAngle.getAsDouble()),
                this
        );
    }
    public Command robotCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(joystickTransform(strafeSpeed.getAsDouble()), joystickTransform(forwardSpeed.getAsDouble()),
                        joystickTransform(turnSpeed.getAsDouble()), false),
                this
        );
    }
    public Command slowMode(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                            DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafeSpeed.getAsDouble() / slowFactor,
                        forwardSpeed.getAsDouble() / slowFactor,
                        turnSpeed.getAsDouble() / slowFactor),
                this
        );
    }
    public void driveWithMotorPowers(double fL, double fR, double bL, double bR) {
        drive.driveWithMotorPowers(fL, fR, bL, bR);
    }
    public double joystickTransform(double input) {
        return (1.0 / (joystickTransformFactor - 1))
                * Math.signum(input)
                * (Math.pow(joystickTransformFactor, Math.abs(input)) - 1);
    }
    private double getYaw() {
        return imu.getHeading();
    }
}