package org.firstinspires.ftc.teamcode;

import javax.vecmath.Vector2d;
import com.github.dummybotslammer.spacesaintsmppc.DriveControllers.OmniDriveController;

import com.github.dummybotslammer.spacesaintsmppc.Utils.MathUtils;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Vector;

@Autonomous(name="OmniBot Drivebase Autonomous Odometry Test")
public class OmniDriveAutonomousControl_Test extends LinearOpMode {
    //All physical measurements are in SI units, unless stated otherwise, or being converted to motor/encoder ticks.
    //Angles are all in radians unless specified otherwise as well.
    private final int TICKS_PER_HDHEXMOTOR_REV = 560;
    private final int TICKS_PER_COREHEXMOTOR_REV = 288;
    private final double wheelDiameter = 0.09;
    private final double stickMovementVectorScaleFactor = 0.2; //For: heading_velocity = scale * velocity (ms^-1)
    private final double maxAngularVelocity = Math.PI/3;

    private Blinker control_Hub;
    private IMU imu; //BNO055
    private IMU.Parameters imuParameters;
    private DcMotorEx drive1;
    private DcMotorEx drive2;
    private DcMotorEx drive3;
    private OmniDriveController omnidrive;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        drive1 = hardwareMap.get(DcMotorEx.class, "drive1");
        drive2 = hardwareMap.get(DcMotorEx.class, "drive2");
        drive3 = hardwareMap.get(DcMotorEx.class, "drive3");

        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();

        telemetry.addData("Status", "Running");

        //RUNTIME VARIABLES
        double leftStickX;
        double leftStickY;
        double rightStickX;
        double angularVelocity;
        double startElapsed = System.nanoTime();
        double endElapsed = System.nanoTime();
        double elapsedTime = (endElapsed - startElapsed)/(1000000000); //Converted to seconds
        Vector2d stickVector = new Vector2d(0, 0);

        //Object for receiving the IMU angles:
        YawPitchRollAngles robotAngles;
        robotAngles = imu.getRobotYawPitchRollAngles();
        double initialHeading = Math.PI/2;
        double imuYaw = robotAngles.getYaw(AngleUnit.RADIANS) + initialHeading;

        //MOTOR & ENCODER RESETS AND CONFIGS
        drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        omnidrive = new OmniDriveController(drive1, drive2, drive3, TICKS_PER_HDHEXMOTOR_REV, TICKS_PER_HDHEXMOTOR_REV, TICKS_PER_COREHEXMOTOR_REV, wheelDiameter);
        omnidrive.setOdoHeadingAngle(Math.PI/2);
        omnidrive.motionController.setCoefficients(0.4, 0.0, 0.2);
        omnidrive.rotationController.setCoefficients(0.8, 0.0, 0.4);
        waitForStart();

        while(opModeIsActive()) {
            robotAngles = imu.getRobotYawPitchRollAngles();
            imuYaw = robotAngles.getYaw(AngleUnit.RADIANS);
            if(imuYaw < 0) {
                imuYaw = MathUtils.addAngles(((2*Math.PI)+imuYaw), initialHeading);
            }

            else {
                imuYaw = MathUtils.addAngles(imuYaw, initialHeading);
            }

            Vector2d target = new Vector2d(2.5, 0.0);
            double targetAngle = Math.PI/2;
            Vector2d error = new Vector2d(0,0);
            Vector2d correction = new Vector2d(0,0);
            double rotCorrection = 0.0;
            double rotError = 0.0;
            boolean targetReached = false;
            //omnidrive.targetCoordinates(1, 1, 0.1);
            endElapsed = System.nanoTime();
            elapsedTime = (endElapsed - startElapsed)*(Math.pow(10, 9));

            Vector2d deriv = new Vector2d(0,0);

            omnidrive.updateOdometry(elapsedTime);
            omnidrive.setOdoHeadingAngle(imuYaw);
            omnidrive.motionController.setDeltaTime(elapsedTime);
            omnidrive.rotationController.setDeltaTime(elapsedTime);

            rotError = -(targetAngle - omnidrive.getHeadingAngle());
            error.sub(target, omnidrive.getPosition());

            omnidrive.motionController.setCurrentVectorError(error);
            omnidrive.rotationController.setCurrentScalarError(rotError);

            if(Math.toDegrees(Math.abs(rotError)) > 0.25) {
                rotCorrection = omnidrive.rotationController.updateScalarPID();
                omnidrive.setTargetAngularVelocity(rotCorrection);
            }

            else {
                omnidrive.setTargetAngularVelocity(0);
            }

            if(error.length() > 0.02) {
                Vector2d[] temp = omnidrive.motionController.updateVectorPID();
                correction = temp[0];
                deriv = temp[3];
                //correction.scale(Math.min(correction.length(), maxDriveSpeed)/maxDriveSpeed);
                omnidrive.setGlobalTargetLinearVelocity(correction);
            }

            else {
                omnidrive.setGlobalTargetLinearVelocity(new Vector2d(0,0));
                targetReached = true;
            }

            omnidrive.computeVelocitiesThenDrive();

            //DEBUG
            double[] t1 = {omnidrive.getTargetDriveVelocities()[0].x,omnidrive.getTargetDriveVelocities()[0].y};
            double[] t2 = {omnidrive.getTargetDriveVelocities()[1].x,omnidrive.getTargetDriveVelocities()[1].y};
            double[] t3 = {omnidrive.getTargetDriveVelocities()[2].x,omnidrive.getTargetDriveVelocities()[2].y};
            Vector2d robotPosition = omnidrive.getPosition();

            startElapsed = System.nanoTime();

            telemetry.addData("Relative Linear Velocity", String.format("%f, %f", omnidrive.getTargetRelativeLinearVelocity().x, omnidrive.getTargetRelativeLinearVelocity().y));
            telemetry.addData("Drive1 Velocity", String.format("%f, %f", t1[0], t1[1]));
            telemetry.addData("Drive2 Velocity", String.format("%f, %f", t2[0], t2[1]));
            telemetry.addData("Drive3 Velocity", String.format("%f, %f", t3[0], t3[1]));
            telemetry.addData("Drive1 Speed", omnidrive.getTargetDriveSpeeds()[0]);
            telemetry.addData("Drive2 Speed", omnidrive.getTargetDriveSpeeds()[1]);
            telemetry.addData("Drive3 Speed", omnidrive.getTargetDriveSpeeds()[2]);
            telemetry.addData("Drive1 Vector Angle", Math.toDegrees(omnidrive.getTargetDriveVelocities()[0].angle(omnidrive.getUnitVectors()[0])));
            telemetry.addData("Drive2 Vector Angle", Math.toDegrees(omnidrive.getTargetDriveVelocities()[0].angle(omnidrive.getUnitVectors()[0])));
            telemetry.addData("Drive3 Vector Angle", Math.toDegrees(omnidrive.getTargetDriveVelocities()[0].angle(omnidrive.getUnitVectors()[0])));
            telemetry.addData("Robot Position", String.format("%f, %f", robotPosition.x, robotPosition.y));
            telemetry.addData("Robot Heading Angle", Math.toDegrees(imuYaw)); //omnidrive.getHeadingAngle()
            telemetry.addData("Robot Heading Error", rotError);
            telemetry.addData("Robot Angular Velocity", Math.toDegrees(omnidrive.getTargetAngularVelocity()));
            telemetry.addData("Global X and Y Values", String.format("%f, %f", omnidrive.getGlobalTargetLinearVelocity().x, omnidrive.getGlobalTargetLinearVelocity().y));
            telemetry.addData("Robot Correction Vector", String.format("%f, %f", correction.x, correction.y));
            telemetry.addData("Robot Error Vector", String.format("%f, %f", error.x, error.y));
            telemetry.addData("Robot Deriv Vector", String.format("%f, %f", deriv.x, deriv.y));
            telemetry.addData("Reached Target?", targetReached);
            telemetry.update();
        }
    }
}
