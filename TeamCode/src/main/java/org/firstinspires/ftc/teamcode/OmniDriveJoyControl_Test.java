package org.firstinspires.ftc.teamcode;

import javax.vecmath.Vector2d;
import org.firstinspires.ftc.teamcode.spacesaints_mppc.DriveControllers.OmniDriveController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp(name="OmniBot Drivebase Joystick Control Test")
public class OmniDriveJoyControl_Test extends LinearOpMode {
    //All physical measurements are in SI units, unless stated otherwise, or being converted to motor/encoder ticks.
    //Angles are all in radians unless specified otherwise as well.
    private final int TICKS_PER_HDHEXMOTOR_REV = 560;
    private final int TICKS_PER_COREHEXMOTOR_REV = 288;
    private final double wheelDiameter = 0.09;
    private final double stickMovementVectorScaleFactor = 0.2; //For: heading_velocity = scale * velocity (ms^-1)
    private final double maxAngularVelocity = Math.PI/3;

    private Blinker control_Hub;
    private Gyroscope imu;
    private DcMotorEx drive1;
    private DcMotorEx drive2;
    private DcMotorEx drive3;
    private OmniDriveController omnidrive;

    @Override
    public void runOpMode() {
        drive1 = hardwareMap.get(DcMotorEx.class, "drive1");
        drive2 = hardwareMap.get(DcMotorEx.class, "drive2");
        drive3 = hardwareMap.get(DcMotorEx.class, "drive3");

        telemetry.addData("Status", "Running");

        //RUNTIME VARIABLES
        double leftStickX;
        double leftStickY;
        double rightStickX;
        double angularVelocity;
        double startElapsed = System.nanoTime();
        double endElapsed = System.nanoTime();
        double elapsedTime = endElapsed - startElapsed;
        Vector2d stickVector = new Vector2d(0, 0);
        Vector2d heading_velocity;

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
        waitForStart();

        while(opModeIsActive()) {
            leftStickX = gamepad1.left_stick_x;
            leftStickY = -1*gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;
            angularVelocity = maxAngularVelocity*rightStickX;

            stickVector.set(new double[]{leftStickX, leftStickY});
            stickVector.scale(stickMovementVectorScaleFactor);
            omnidrive.setAngularVelocity(angularVelocity);
            omnidrive.setLinearVelocity(stickVector);

            omnidrive.computeVelocitiesThenDrive();
            omnidrive.updateOdometry(elapsedTime*(Math.pow(10, 9)));

            //DEBUG
            double[] t1 = {omnidrive.getDriveVelocities()[0].x,omnidrive.getDriveVelocities()[0].y};
            double[] t2 = {omnidrive.getDriveVelocities()[1].x,omnidrive.getDriveVelocities()[1].y};
            double[] t3 = {omnidrive.getDriveVelocities()[2].x,omnidrive.getDriveVelocities()[2].y};
            Vector2d robotPosition = omnidrive.getPosition();
            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Left Stick Y", leftStickY);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("Drive1 Velocity", String.format("%f, %f", t1[0], t1[1]));
            telemetry.addData("Drive2 Velocity", String.format("%f, %f", t2[0], t2[1]));
            telemetry.addData("Drive3 Velocity", String.format("%f, %f", t3[0], t3[1]));
            telemetry.addData("Drive1 Speed", omnidrive.getDriveSpeeds()[0]);
            telemetry.addData("Drive2 Speed", omnidrive.getDriveSpeeds()[1]);
            telemetry.addData("Drive3 Speed", omnidrive.getDriveSpeeds()[2]);
            telemetry.addData("Drive1 Vector Angle", Math.toDegrees(omnidrive.getDriveVelocities()[0].angle(omnidrive.getUnitVectors()[0])));
            telemetry.addData("Drive2 Vector Angle", Math.toDegrees(omnidrive.getDriveVelocities()[0].angle(omnidrive.getUnitVectors()[0])));
            telemetry.addData("Drive3 Vector Angle", Math.toDegrees(omnidrive.getDriveVelocities()[0].angle(omnidrive.getUnitVectors()[0])));
            telemetry.addData("Robot Position", String.format("%f, %f", robotPosition.x, robotPosition.y));
            telemetry.addData("Robot Heading Angle", Math.toDegrees(omnidrive.getHeadingAngle()));
            telemetry.update();
        }
    }
}
