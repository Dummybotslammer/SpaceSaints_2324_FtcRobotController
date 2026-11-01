package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.AUTONOMOUS_HEADING_TOLERANCE_LOOSE;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.AUTONOMOUS_HEADING_TOLERANCE_TIGHT;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.DISTANCE_SENSOR_CENTRE_OFFSET_LEFT;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.DISTANCE_SENSOR_CENTRE_OFFSET_RIGHT;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.DRIVETRAIN_RADIUS;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.LIFT_EXTENSION_POSITIONS;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.OUTTAKE_POCKET_DEPOSIT_POSITION;
import static org.firstinspires.ftc.teamcode.Glockenzerstorer_RobotProfile.OUTTAKE_POCKET_REST_POSITION;

import com.github.dummybotslammer.spacesaintsmppc.Controllers.OmniDriveController;
import com.github.dummybotslammer.spacesaintsmppc.Utils.MotionPath;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;

import javax.vecmath.Vector2d;

@Autonomous(name="Glockenzerstorer2324 - Calculated Side Auto Blue - Right Side V1")
public class Glockenzerstorer_CalcAutoSide_BLUERight extends LinearOpMode {
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

    //Sensors
    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;
    private OpenCvCamera camera;
    private CentrestageTeamPropDetectionPipeline pipeline;

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
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");;

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

        telemetry.addData("Status", "Running");

        //RUNTIME VARIABLES
        int pathID = 0;
        double startElapsed = System.nanoTime();
        double endElapsed = System.nanoTime();
        double elapsedTime = (endElapsed - startElapsed)/(1000000000); //Converted to seconds
        Vector2d stickVector = new Vector2d(0, 0);

        Vector2d target = new Vector2d(2.5, 1.0);
        double targetAngle = Math.PI/2;

        MotionPath paths[] = {new MotionPath(), new MotionPath(), new MotionPath(), new MotionPath()};
        for (int i = 0; i < paths.length; i++) {
            paths[i].setExMotors( new DcMotorEx[] {
                    lift,
            } );
            paths[i].setSimpleMotors( new DcMotorSimple[] {
                    intakeFeeder,
            } );
            paths[i].setServos( new Servo[] {
                    outtakePivot,
            } );
        }

        //START PATH
        paths[0].setInstructions( new double[][]{
                {'r', 0.5, 0.0, 0.3, 0, 0},
                { 'p', 0.9144, 2.6416, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                {'s', 0, 0, 0, 0, 0}, //Useful for implementing delays.
        } );

        //MIDDLE (2) PATH
        paths[2].setInstructions( new double[][] {
                { 'p', 0.9144, 2.6416, 1.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                {'r', 1.8, 0.0, 1.0, 0, 0},
                { 'p', 0.9144, 2.8448, 1.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.508, 2.8448, 1.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.508, 2.8448, 0.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.508, 1.8288, 0.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE, AUTONOMOUS_HEADING_TOLERANCE_LOOSE,  },
                { 'p', 0.508, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE, AUTONOMOUS_HEADING_TOLERANCE_LOOSE,  },
                { 'p', 2.9464, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE, AUTONOMOUS_HEADING_TOLERANCE_LOOSE,  },
                { 'p', 2.9464, 1.8288, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE, AUTONOMOUS_HEADING_TOLERANCE_LOOSE,  },
                { 'p', 3.048, 2.7432, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 3.1496, 2.7432, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },

                {'e', 0, LIFT_EXTENSION_POSITIONS[2], 500, 1.0, 2000},
                {'v', 0, OUTTAKE_POCKET_DEPOSIT_POSITION, 1500, 0, 0},
                {'v', 0, OUTTAKE_POCKET_REST_POSITION, 500, 0, 0},
                {'e', 0, LIFT_EXTENSION_POSITIONS[0], 500, 1.0, 2000},
        } );

        //LEFT (1) PATH
        paths[1].setInstructions( new double[][] {
                { 'p', 0.9144, 2.6416, 0.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                {'r', 1.8, 0.0, 1.0, 0, 0},
                { 'p', 1.016, 2.6416, 0.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9144, 2.6416, 0.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9144, 1.8288, 0.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9144, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 2.9464, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 3.048, 2.8956, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 3.15, 2.8956, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },

                {'e', 0, LIFT_EXTENSION_POSITIONS[2], 500, 1.0, 2000},
                {'v', 0, OUTTAKE_POCKET_DEPOSIT_POSITION, 1500, 0, 0},
                {'v', 0, OUTTAKE_POCKET_REST_POSITION, 500, 0, 0},
                {'e', 0, LIFT_EXTENSION_POSITIONS[0], 500, 1.0, 2000},
        } );

        //RIGHT (3) PATH
        paths[3].setInstructions( new double[][] {
                { 'p', 0.9144, 2.6416, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                {'r', 1.8, 0.0, 1.0, 0, 0},
                { 'p', 0.8128, 2.6416, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9144, 2.6416, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9144, 1.8288, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9144, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 2.9464, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 2.9464, 1.8288, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 3.048, 2.5908, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 3.1496, 2.5908, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },

                {'r', 0.5, 0.0, 0.3, 0, 0},
                { 'p', 0.9144, 2.6416, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                {'r', 1.8, 0.0, 1.0, 0, 0},
                { 'p', 0.8128, 2.6416, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9652000000000001, 2.6416, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 0.9144, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE, AUTONOMOUS_HEADING_TOLERANCE_LOOSE,  },
                { 'p', 2.9464, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE, AUTONOMOUS_HEADING_TOLERANCE_LOOSE,  },
                { 'p', 2.9464, 1.8288, 0.5*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE, AUTONOMOUS_HEADING_TOLERANCE_LOOSE,  },
                { 'p', 3.048, 2.5908, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },
                { 'p', 3.1496, 2.5908, 1.0*Math.PI, AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT, AUTONOMOUS_HEADING_TOLERANCE_TIGHT,  },

                //Scoring
                {'e', 0, LIFT_EXTENSION_POSITIONS[2], 500, 1.0, 2000},
                {'v', 0, OUTTAKE_POCKET_DEPOSIT_POSITION, 1500, 0, 0},
                {'v', 0, OUTTAKE_POCKET_REST_POSITION, 500, 0, 0},
                {'e', 0, LIFT_EXTENSION_POSITIONS[0], 500, 1.0, 2000},
        } );

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

        outtakePivot.setPosition(OUTTAKE_POCKET_REST_POSITION);

        omnidrive = new OmniDriveController(drive1, drive2, drive3, imu, TICKS_PER_HDHEXMOTOR_REV, TICKS_PER_HDHEXMOTOR_REV, TICKS_PER_HDHEXMOTOR_REV, wheelDiameter);
        omnidrive.setInitialHeading(2.0*Math.PI/4.0*3.0);
        omnidrive.translationController.setCoefficients(0.3, 0.0, 0.15);
        omnidrive.rotationController.setCoefficients(1.8, 0.0, 1.0);  //(1.0, 0.0, 0.6);
        omnidrive.setTranslationControllerTolerance(0.05);
        omnidrive.setRotationControllerTolerance(Math.toRadians(5.0));

        sleep(1000);

        int markerID = 0; //1->LEFT, 2->MIDDLE, 3->RIGHT
        double initx = distanceSensorRight.getDistance(DistanceUnit.METER) + DISTANCE_SENSOR_CENTRE_OFFSET_LEFT;
        double inity = 3.6576-DRIVETRAIN_RADIUS;
        omnidrive.setPosition(new Vector2d(initx, inity));

        waitForStart();

        while(opModeIsActive()) {
            endElapsed = System.nanoTime();
            elapsedTime = (endElapsed - startElapsed)*(Math.pow(10, 9));

            if (markerID == 0 && pathID == 0 && omnidrive.getPosition().y < 3.6576-0.8) {
                double obj_l = distanceSensorLeft.getDistance(DistanceUnit.METER)+DISTANCE_SENSOR_CENTRE_OFFSET_LEFT;
                double obj_r = distanceSensorRight.getDistance(DistanceUnit.METER)+DISTANCE_SENSOR_CENTRE_OFFSET_RIGHT;

                if (obj_l <= 0.4) { markerID=1; }
                if (obj_r <= 0.4) { markerID=3; }
                pathID = markerID;
            }

            if (omnidrive.trackMotionPath(paths[pathID], elapsedTime) && markerID == 0) { markerID=2; pathID=2; }

            //DEBUG
            double[] t1 = {omnidrive.getTargetDriveVelocities()[0].x,omnidrive.getTargetDriveVelocities()[0].y};
            double[] t2 = {omnidrive.getTargetDriveVelocities()[1].x,omnidrive.getTargetDriveVelocities()[1].y};
            double[] t3 = {omnidrive.getTargetDriveVelocities()[2].x,omnidrive.getTargetDriveVelocities()[2].y};
            Vector2d robotPosition = omnidrive.getPosition();

            startElapsed = System.nanoTime();

            telemetry.addData("Autonomous Path ID", pathID);
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
            telemetry.addData("Robot Heading Angle", omnidrive.getHeadingAngle()); //omnidrive.getHeadingAngle()
            //telemetry.addData("Robot Heading Error", rotError);
            telemetry.addData("Robot Angular Velocity", Math.toDegrees(omnidrive.getTargetAngularVelocity()));
            telemetry.addData("Global X and Y Values", String.format("%f, %f", omnidrive.getGlobalTargetLinearVelocity().x, omnidrive.getGlobalTargetLinearVelocity().y));
            telemetry.addData("Robot Correction Vector", String.format("%f, %f", omnidrive.translationController.getVectorCorrection().x, omnidrive.translationController.getVectorCorrection().y));
            //telemetry.addData("Robot Error Vector", String.format("%f, %f", error.x, error.y));
            //telemetry.addData("Reached Target?", targetReached);
            telemetry.update();
        }
    }
}
