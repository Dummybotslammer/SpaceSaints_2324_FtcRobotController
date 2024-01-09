package org.firstinspires.ftc.teamcode;

import com.github.dummybotslammer.spacesaintsmppc.Controllers.DcMotorExController;
import com.github.dummybotslammer.spacesaintsmppc.Controllers.OmniDriveController;
import com.github.dummybotslammer.spacesaintsmppc.Utils.StateMachine;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import javax.vecmath.Vector2d;

@TeleOp(name="Glockenzerstorer2324 - DriverApp")
public class Glockenzerstorer_DriverApp extends LinearOpMode {
    //All physical measurements are in SI units, unless stated otherwise, or being converted to motor/encoder ticks.
    //Angles are all in radians unless specified otherwise as well.
    private final int TICKS_PER_HDHEXMOTOR_REV = (int) (2.0*28.0*18.9); //2 Stage Ultraplanetary Gearboxes: 4:1 & 5:1 (Total Nominal: 20:1)
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

    private DcMotorEx lift;
    private DcMotorSimple intakeFeeder;
    private Servo launcher;
    private Servo outtakePivot;
    private OmniDriveController omnidrive;
    private StateMachine states = new StateMachine();

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        drive1 = hardwareMap.get(DcMotorEx.class, "drive1");
        drive2 = hardwareMap.get(DcMotorEx.class, "drive2");
        drive3 = hardwareMap.get(DcMotorEx.class, "drive3");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intakeFeeder = hardwareMap.get(DcMotorSimple.class, "intakeFeederL");
        launcher = hardwareMap.get(Servo.class, "launcher");
        outtakePivot = hardwareMap.get(Servo.class, "outtakePivot");

        imuParameters = new IMU.Parameters( //Measurements taken from robot chassis.
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                -145,
                                0,
                                180,
                                0  // acquisitionTime, not used
                        )
                )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();

        states.setPrevStates(new boolean[] {
                false, //#0 - Intake Feeder
                false, //#1 - Outtake Pocket Pivot
                false, //#2 - Lift (Up)
                false, //#3 - Lift (Down)
        });

        states.setSequences(new int[] {
                0, //#0 - Lift Stage Sequence
        });

        telemetry.addData("Status", "Running");

        //RUNTIME VARIABLES
        double leftStickX, leftStickY, rightStickX, angularVelocity;
        double startElapsed = System.nanoTime();
        double endElapsed = System.nanoTime();
        double elapsedTime = (endElapsed - startElapsed)*(Math.pow(10, 9)); //Converted to seconds
        Vector2d stickVector = new Vector2d(0, 0);

        //MOTOR & ENCODER RESETS AND CONFIGS
        launcher.setPosition(-1);

        drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setTargetPosition(0);
        lift.setPower(1.0);

        drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakePivot.setPosition(0.5);

        omnidrive = new OmniDriveController(drive1, drive2, drive3, imu, TICKS_PER_HDHEXMOTOR_REV, TICKS_PER_HDHEXMOTOR_REV, TICKS_PER_HDHEXMOTOR_REV, wheelDiameter);
        omnidrive.setInitialHeading(Math.PI/2);
        omnidrive.setTargetHeading(omnidrive.getInitialHeading());

        omnidrive.translationController.setCoefficients(0.4, 0.0, 0.2);
        omnidrive.rotationController.setCoefficients(1.4, 0.0, 1.3);

        waitForStart();

        while(opModeIsActive()) {
            //Input Processing
            leftStickX = gamepad1.left_stick_x;
            leftStickY = -gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;
            angularVelocity = maxAngularVelocity*rightStickX;

            stickVector.set(new double[]{leftStickX, leftStickY});
            stickVector.scale(stickMovementVectorScaleFactor);

            endElapsed = System.nanoTime();
            elapsedTime = (endElapsed - startElapsed)*(Math.pow(10, 9));

            //Launcher
            if(gamepad1.left_bumper && gamepad1.right_bumper) { launcher.setPosition(1); }

            //Linear Slide
            if (states.highToggle(gamepad1.dpad_up, 2)) { DcMotorExController.togglePosition(lift, 0, -DcMotorExController.metersToTicks(0.25, 0.05496, TICKS_PER_HDHEXMOTOR_REV)); }
            else if ( states.highToggle(gamepad1.dpad_down, 3)) {}

            //Intake Feeder
            if (states.highToggle(gamepad1.y, 0)) { DcMotorExController.togglePower(intakeFeeder, 1.0, 0.0); }

            //Outtake Pocket Pivot
            if (states.highToggle(gamepad1.x, 1)) { DcMotorExController.toggleServoPosition(outtakePivot, 0.5, 0.95); }

            //Odometry Loop
            omnidrive.updateOdometry(elapsedTime, true);

            //Heading Controls & Drift Correction
            if (Math.abs(angularVelocity) > 0) {
                omnidrive.setTargetAngularVelocity(angularVelocity);
                omnidrive.setTargetHeading(omnidrive.getHeadingAngle());
            }
            else { omnidrive.applyRotationalCorrection(elapsedTime); /*Drift Correction*/ }

            //Drive Controls
            omnidrive.setRelativeTargetLinearVelocity(stickVector);
            omnidrive.computeVelocitiesThenDrive();

            startElapsed = System.nanoTime();

            //DEBUG//
            double[] t1 = {omnidrive.getTargetDriveVelocities()[0].x,omnidrive.getTargetDriveVelocities()[0].y};
            double[] t2 = {omnidrive.getTargetDriveVelocities()[1].x,omnidrive.getTargetDriveVelocities()[1].y};
            double[] t3 = {omnidrive.getTargetDriveVelocities()[2].x,omnidrive.getTargetDriveVelocities()[2].y};
            Vector2d robotPosition = omnidrive.getPosition();

            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Left Stick Y", leftStickY);
            telemetry.addData("Right Stick X", rightStickX);
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
            telemetry.addData("Robot Heading Angle (Encoders)", Math.toDegrees(omnidrive.getHeadingAngle()));
            telemetry.addData("Robot Heading Angle (IMU)", Math.toDegrees(omnidrive.getHeadingAngle()));
            telemetry.addData("Global X and Y Values", String.format("%f, %f", omnidrive.getGlobalTargetLinearVelocity().x, omnidrive.getGlobalTargetLinearVelocity().y));
            telemetry.update();
        }
    }
}
