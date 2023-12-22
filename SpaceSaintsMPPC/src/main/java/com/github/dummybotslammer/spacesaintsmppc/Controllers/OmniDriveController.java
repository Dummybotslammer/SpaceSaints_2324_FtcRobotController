package com.github.dummybotslammer.spacesaintsmppc.Controllers;

import com.github.dummybotslammer.spacesaintsmppc.Utils.MathUtils;
import com.github.dummybotslammer.spacesaintsmppc.Utils.PIDController;
import com.github.dummybotslammer.spacesaintsmppc.Utils.MotionPath;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import javax.vecmath.Vector2d;

public class OmniDriveController {
    /*
    TODO: SEPARATE VARIABLES INTO THREE TYPES:
    1. CONSTANTS & HARDWARE
    2. DRIVEBASE CHARACTERISTIC VARIABLES
    3. TARGET & CONTROL VARIABLES:
    SET BY THE USER IN ORDER TO CONTROL THE ROBOT, TARGET POSITIONS, AND COMPUTE WHICH DIRECTION IS INTENDED TO BE TRAVELLED
    4. ODOMETRY/TRACKING VARIABLES:
    USED INTERNALLY BY THE CLASS INSTANCE TO COMPUTE <ACTUAL> POSITIONING, LINEAR VELOCITY, AND DISPLACEMENT, ETC...
    MEANT TO TRACK AND STORE <CURRENT>/<ACTUAL> VALUES OF THE SYSTEM, AND COMPARE THEM TO THE TARGETS FOR PID CONTROL,
    MOTION CONTROL AND MAPPING, AS WELL AS DETECTING FAIL SAFES OR JAMS BASED ON ANY INCONSISTENCIES OR SPIKES IN TRACKING
    VARIABLES.
    The target variables and odometry/tracking variables are the same, but just counterparts of one another.
     */
    //The OmniDrive runs on 3 different motors.
    //The motors need to be set to DcMotor.RunMode.RUN_USING_ENCODER mode.
    //All physical quantities are in SI units, which are then converted to the encoder counts for the purpose of the motors.
    //The forward heading of the robot is (0, 1)

    //1-- CONSTANTS & HARDWARE--
    private DcMotorEx drive1;
    private DcMotorEx drive2;
    private DcMotorEx drive3;
    private IMU imu;

    //2-- DRIVEBASE CHARACTERISTIC VARIABLES --
    private double WHEEL_DIAMETER = 0.09;
    private double INSCRIBED_BASE_RADIUS = Math.sqrt(14699.905641)/10/100; //mm -> cm -> m
    private double DRIVE1_TRACKWIDTH = Math.sqrt(132299.878225)/10/100; //mm -> cm -> m
    private double DRIVE2_TRACKWIDTH = Math.sqrt(132299.878225)/10/100; //mm -> cm -> m
    private double DRIVE3_TRACKWIDTH = 363.731/10/100; //mm -> cm -> m

    //2 Stage Ultraplanetary Gearboxes: 4:1 & 5:1 (Total Nominal: 20:1)
    private int TICKS_PER_DRIVE1_REV = (int) (2.0*28.0*18.9);
    private int TICKS_PER_DRIVE2_REV = (int) (2.0*28.0*18.9);
    private int TICKS_PER_DRIVE3_REV = (int) (2.0*28.0*18.9);
    //Unit Vectors with length 1.
    private Vector2d DRIVE1_UNIT_VECTOR = new Vector2d(0.5, Math.sqrt(0.75));
    private Vector2d DRIVE2_UNIT_VECTOR = new Vector2d(0.5, -Math.sqrt(0.75));
    private Vector2d DRIVE3_UNIT_VECTOR = new Vector2d(-1.0, 0.0);

    //3-- TARGET & CONTROL VARIABLES --
    public PIDController translationController;
    public PIDController rotationController;
    private double translationTolerance = 0.0;
    private double rotationTolerance = 0.0;
    private Vector2d target_position = new Vector2d(0, 0);
    private double target_heading = 0.0;
    private Vector2d target_linear_velocity = new Vector2d(0,0); //Relative
    private Vector2d target_drive1_velocity = new Vector2d(0,0);
    private Vector2d target_drive2_velocity = new Vector2d(0,0);
    private Vector2d target_drive3_velocity = new Vector2d(0,0);
    private double target_drive1_speed = 0.0;
    private double target_drive2_speed = 0.0;
    private double target_drive3_speed = 0.0;
    private double target_angular_velocity = 0.0;

    //4-- ODOMETRY & TRACKING VARIABLES--
    //All of these variables are global vectors, and not relative to the drivebase.
    //Linear Motion
    private Vector2d odo_position = new Vector2d(0,0);
    private Vector2d odo_prev_position = new Vector2d(0,0);
    private Vector2d odo_velocity = new Vector2d(0,0);
    private Vector2d odo_prev_velocity = new Vector2d(0,0);
    private Vector2d odo_acceleration = new Vector2d(0, 0);
    private Vector2d odo_prev_acceleration = new Vector2d(0,0);

    //Rotational Motion
    private double odo_intial_heading = 0.0;
    private double odo_heading_angle = 0.0;
    private double odo_prev_heading_angle = 0.0;
    private double odo_angular_vel = 0.0;
    private double odo_prev_angular_vel = 0.0;
    private double odo_angular_accel = 0.0;
    private double odo_prev_angular_accel = 0.0;

    //Motor Positions
    private int drive1_position = 0;
    private int drive2_position = 0;
    private int drive3_position = 0;
    private int drive1_prevPosition = 0;
    private int drive2_prevPosition = 0;
    private int drive3_prevPosition = 0;
    //Timing
    private double odo_prevtime = 0.0;
    private double odo_time = 0.0;

    //Constructors
    public OmniDriveController() {
        //No argument Constructor
        //TODO: Improve the constructors
    }

    public OmniDriveController(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3) {
        drive1 = motor1;
        drive2 = motor2;
        drive3 = motor3;

        translationController = new PIDController(1.0, 0, 0);
        rotationController = new PIDController(1.0, 0, 0);
    }

    public OmniDriveController(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, IMU imuObj, int TICKS_PER_MOTOR1_REV, int TICKS_PER_MOTOR2_REV, int TICKS_PER_MOTOR3_REV, double wheel_diameter) {
        drive1 = motor1;
        drive2 = motor2;
        drive3 = motor3;
        imu = imuObj;

        TICKS_PER_DRIVE1_REV = TICKS_PER_MOTOR1_REV;
        TICKS_PER_DRIVE2_REV = TICKS_PER_MOTOR2_REV;
        TICKS_PER_DRIVE3_REV = TICKS_PER_MOTOR3_REV;

        WHEEL_DIAMETER = wheel_diameter;

        translationController = new PIDController(1.0, 0, 0);
        rotationController = new PIDController(1.0, 0, 0);
    }

    //Setter/Getter Methods
    //TODO: COMPLETE ALL THE SETTERS AND GETTERS
    public void setRotationControllerTolerance(double t) { rotationTolerance = t; }

    public double getRotationControllerTolerance() { return rotationTolerance; }

    public void setTranslationControllerTolerance(double t) { translationTolerance = t; }

    public double getTranslationControllerTolerance() { return translationTolerance; }

    public void setInitialHeading(double heading) {
        odo_intial_heading = heading;
    }

    public double getInitialHeading() {
        return odo_intial_heading;
    }

    public void setIMU(IMU imuObj) {
        imu = imuObj;
    }

    public IMU getIMU() {
        return imu;
    }

    public void setGlobalTargetLinearVelocity(Vector2d vector) {
        //Compute the translational matrix values
        //First, the unit vectors relative to the robot's heading are computed.
        Vector2d i = new Vector2d(
                Math.cos( MathUtils.subAngles(odo_heading_angle, (Math.PI/2)) ),
                Math.sin( MathUtils.subAngles(odo_heading_angle, (Math.PI/2)) ) );
        Vector2d j = new Vector2d(Math.cos(odo_heading_angle), Math.sin(odo_heading_angle));

        //Hardcoded Vector * Translational Matrix (Values are computed from the relative unit vectors)
        //Translational Matrix: {{i.x, j.x}, {i.y, j.y}}
        Vector2d transVector = new Vector2d(
                (vector.x * i.x + vector.y * i.y),
                (vector.x * j.x + vector.y * j.y));

        target_linear_velocity = transVector;
    }

    public Vector2d getTargetPosition() {
        return target_position;
    }

    public double getTargetHeading() {
        return target_heading;
    }

    public void setTargetPosition(Vector2d v) {
        target_position = v;
    }

    public void setTargetHeading(double h) {
        target_heading = h;
    }

    public Vector2d getGlobalTargetLinearVelocity() {
        return convertRelativeToGlobalVector(target_linear_velocity);
    }

    public void setRelativeTargetLinearVelocity(Vector2d vector) {
        target_linear_velocity = vector;
    }

    public double getTargetAngularVelocity() {
        return target_angular_velocity;
    }

    public void setTargetAngularVelocity(double w) {
        target_angular_velocity = w;
    }

    public void setDriveMotors(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3) {
        drive1 = motor1;
        drive2 = motor2;
        drive3 = motor3;
    }

    public void setOdoHeadingAngle(double r) {
        //Intended to be used to set the initial angle.
        odo_heading_angle = r;
    }

    public Vector2d[] getTargetDriveVelocities() {
        return new Vector2d[]{target_drive1_velocity, target_drive1_velocity, target_drive2_velocity};
    }

    public Vector2d[] getUnitVectors() {
        return new Vector2d[]{DRIVE1_UNIT_VECTOR, DRIVE2_UNIT_VECTOR, DRIVE3_UNIT_VECTOR};
    }

    public double[] getTargetDriveSpeeds() {
        return new double[]{target_drive1_speed, target_drive2_speed, target_drive3_speed};
    }

    public Vector2d getTargetRelativeLinearVelocity() {
        return target_linear_velocity;
    }

    public Vector2d getPosition() {return odo_position;}

    public double getHeadingAngle() {return odo_heading_angle;}

    //Computation Methods
    public Vector2d convertRelativeToGlobalVector(Vector2d vector) {
        //LOGIC:
        //1. Compute the difference in angle, that is robotHeading - PI/2
        //2. That difference is exactly how much the relative vector is off from the global space.
        //3. A CCW Rotation Matrix is applied. + angleDiff = CCW Rotation, - angleDiff = CW Rotation
        double angleDiff = odo_heading_angle - (Math.PI/2);
        //Hardcoded CCW Rotation Matrix
        Vector2d globalVector = new Vector2d(
                ( (vector.x * Math.cos(angleDiff)) + (vector.y * -Math.sin(angleDiff)) ),
                ( (vector.x * Math.sin(angleDiff)) + (vector.y * Math.cos(angleDiff)) ) );
        return globalVector;
    }

    public void computeDriveVelFromHeadingAndAngularVel() {
        Vector2d drive1_turnVel = new Vector2d(DRIVE1_UNIT_VECTOR);
        Vector2d drive2_turnVel = new Vector2d(DRIVE2_UNIT_VECTOR);
        Vector2d drive3_turnVel = new Vector2d(DRIVE3_UNIT_VECTOR);
        target_drive1_velocity = new Vector2d(DRIVE1_UNIT_VECTOR);
        target_drive2_velocity = new Vector2d(DRIVE2_UNIT_VECTOR);
        target_drive3_velocity = new Vector2d(DRIVE3_UNIT_VECTOR);
        double turnSpeed_per_motor = target_angular_velocity*INSCRIBED_BASE_RADIUS;

        drive1_turnVel.scale(turnSpeed_per_motor);
        drive2_turnVel.scale(turnSpeed_per_motor);
        drive3_turnVel.scale(turnSpeed_per_motor);

        target_drive1_velocity.scale(DRIVE1_UNIT_VECTOR.dot(target_linear_velocity));
        target_drive2_velocity.scale(DRIVE2_UNIT_VECTOR.dot(target_linear_velocity));
        target_drive3_velocity.scale(DRIVE3_UNIT_VECTOR.dot(target_linear_velocity));

        target_drive1_velocity.add(drive1_turnVel);
        target_drive2_velocity.add(drive2_turnVel);
        target_drive3_velocity.add(drive3_turnVel);

        //Boolean Logic: (angle > 0)? TRUE=-1:FALSE=1
        //Returns -1 if the angle is positive, and 1 if the angle is 0 or less.
        target_drive1_speed = target_drive1_velocity.length() * ((target_drive1_velocity.angle(DRIVE1_UNIT_VECTOR) > 0)? -1:1);
        target_drive2_speed = target_drive2_velocity.length() * ((target_drive2_velocity.angle(DRIVE2_UNIT_VECTOR) > 0)? -1:1);
        target_drive3_speed = target_drive3_velocity.length() * ((target_drive3_velocity.angle(DRIVE3_UNIT_VECTOR) > 0)? -1:1);
    }

    public void updateOdometry(double currentTime, boolean utilizeIMU) {
        //Update readings
        odo_time = currentTime;
        double deltaTime = odo_time - odo_prevtime;
        drive1_position = drive1.getCurrentPosition();
        drive2_position = drive2.getCurrentPosition();
        drive3_position = drive3.getCurrentPosition();

        //--SECTION 1: COMPUTE DISPLACEMENT AND DELTA ANGLE--
        //Calculate displacement
        Vector2d drive1_displacement = new Vector2d(convertRelativeToGlobalVector(DRIVE1_UNIT_VECTOR));
        Vector2d drive2_displacement = new Vector2d(convertRelativeToGlobalVector(DRIVE2_UNIT_VECTOR));
        Vector2d drive3_displacement = new Vector2d(convertRelativeToGlobalVector(DRIVE3_UNIT_VECTOR));
        Vector2d net_displacement = new Vector2d(0,0);

        //Scales the unit vectors to represent the actual displacement.
        //The scale factor is calculated by:
        //1. Find difference between the currentPosition and prevPosition.
        //2. Dividing the result by the TICKS_PER_DRIVE#_REV to calculate the number of rotations.
        //3. Multiplying by the wheelDiameter to find the actual magnitude of displacement (in metres).
        double drive1_distance = ((double)(drive1_position-drive1_prevPosition)/TICKS_PER_DRIVE1_REV)*(Math.PI*WHEEL_DIAMETER);
        double drive2_distance = ((double)(drive2_position-drive2_prevPosition)/TICKS_PER_DRIVE2_REV)*(Math.PI*WHEEL_DIAMETER);
        double drive3_distance = ((double)(drive3_position-drive3_prevPosition)/TICKS_PER_DRIVE3_REV)*(Math.PI*WHEEL_DIAMETER);

        drive1_displacement.scale(drive1_distance);
        drive2_displacement.scale(drive2_distance);
        drive3_displacement.scale(drive3_distance);

        //Compute vector sum of displacements.
        net_displacement.add(drive1_displacement, drive2_displacement);
        net_displacement.add(drive3_displacement);
        odo_position.add(net_displacement);

        //Calculate current heading angle. (Either using drive controllers or the IMU)
        if (utilizeIMU) {
            YawPitchRollAngles robotAngles = imu.getRobotYawPitchRollAngles();
            double imuYaw = robotAngles.getYaw(AngleUnit.RADIANS);
            if(imuYaw < 0) {
                imuYaw = MathUtils.addAngles(((2*Math.PI)+imuYaw), odo_intial_heading);
            }

            else {
                imuYaw = MathUtils.addAngles(imuYaw, odo_intial_heading);
            }

            odo_heading_angle = imuYaw;
        }

        else {
            //When robot turns right (negative change in angle), the motor distance increases (clockwise motor rotation)
            //When robot turns left (positive change in angle), the motor distance decreases (anticlockwise motor rotation)
            //Thus the distances must be negated.
            double drive1_deltaAngle = -drive1_distance/DRIVE1_TRACKWIDTH;
            double drive2_deltaAngle = -drive2_distance/DRIVE2_TRACKWIDTH;
            double drive3_deltaAngle = -drive3_distance/DRIVE3_TRACKWIDTH;
            //ANGLE SUM / (# OF MOTORS / 2)
            double net_deltaAngle = (drive1_deltaAngle+drive2_deltaAngle+drive3_deltaAngle)/1.5;

            odo_heading_angle = MathUtils.addAngles(odo_heading_angle, net_deltaAngle);
        }

        //TODO:--SECTION 2: COMPUTE ACTUAL LINEAR VEL. + ACCEL. & ACTUAL ANGULAR VEL. + ACCEL.
        //Rotational Motion

        //--SECTION 3: PREV VARIABLES UPDATED--
        drive1_prevPosition = drive1_position;
        drive2_prevPosition = drive2_position;
        drive3_prevPosition = drive3_position;

        odo_prevtime = currentTime;

        odo_prev_position = new Vector2d(odo_position);
        odo_prev_velocity = new Vector2d(odo_velocity);
        odo_prev_acceleration = new Vector2d(odo_acceleration);

        odo_prev_heading_angle = odo_heading_angle;
        odo_prev_angular_vel = odo_angular_vel;
        odo_prev_angular_accel = odo_angular_accel;
    }

    //Movement Methods
    public void drive() {
        //Sets the robot's driving motor's velocities to the ALREADY computed values
        //Resets the computed drive1_velocity, drive2_velocity, drive3_velocity values
        int drive1_tickspeed = (int)(target_drive1_speed/WHEEL_DIAMETER*TICKS_PER_DRIVE1_REV);
        int drive2_tickspeed = (int)(target_drive2_speed/WHEEL_DIAMETER*TICKS_PER_DRIVE2_REV);
        int drive3_tickspeed = (int)(target_drive3_speed/WHEEL_DIAMETER*TICKS_PER_DRIVE3_REV);

        drive1.setVelocity(drive1_tickspeed);
        drive2.setVelocity(drive2_tickspeed);
        drive3.setVelocity(drive3_tickspeed);
    }

    public void computeVelocitiesThenDrive() {
        //Recomputes the vectors, and then drives the motors.
        computeDriveVelFromHeadingAndAngularVel();
        drive();
    }

    public boolean applyTranslationalCorrection(double elapsedTime) {
        //Returns true once the target has been reached in accordance to the tolerance. Returns false is otherwise.
        Vector2d error = new Vector2d(0,0);
        Vector2d correction;
        boolean status = false;

        //TODO: UPDATE ODOMETRY HERE?
        //TODO: ADD IMU DATA FEED + KALMAN FILTER SYSTEM

        //Assuming the odometry has been updated. For now.
        translationController.setDeltaTime(elapsedTime);

        error.sub(target_position, odo_position);
        translationController.setCurrentVectorError(error);

        if(error.length() > translationTolerance) {
            correction = translationController.updateVectorPID();
            setGlobalTargetLinearVelocity(correction);
        }

        else {
            setGlobalTargetLinearVelocity(new Vector2d(0,0));
            status = true;
        }

        computeVelocitiesThenDrive();
        return status;
    }

    public boolean applyRotationalCorrection(double elapsedTime) {
        //Returns true once the target has been reached in accordance to the tolerance. Returns false is otherwise.
        double error;
        double correction;
        boolean status = false;

        //TODO: UPDATE ODOMETRY HERE?
        //TODO: ADD IMU DATA FEED + KALMAN FILTER SYSTEM

        //Assuming the odometry has been updated. For now.
        rotationController.setDeltaTime(elapsedTime);

        double e1 = -MathUtils.subAngles(target_heading, odo_heading_angle); //Correction in CCW
        double e2 = MathUtils.subAngles(odo_heading_angle, target_heading); //Correction in CW
        //Picks either CCW or CW based on the shortest magnitude. Defaults to CW direction is e1 == e2.
        error = Math.abs(e1)<Math.abs(e2) ? e1:e2;
        rotationController.setCurrentScalarError(error);

        if(Math.abs(error) > rotationTolerance) {
            correction = rotationController.updateScalarPID();
            setTargetAngularVelocity(correction);
        }

        else {
            setTargetAngularVelocity(0);
            status = true;
        }

        //computeVelocitiesThenDrive();
        return status;
    }

    public boolean[] applyMotionCorrection(double elapsedTime) {
        //Returns true for the respective property once the target has been reached in accordance to the tolerance. Returns false is otherwise.
        //status[0] -> translational motion
        //status[1] -> rotational motion
        boolean[] status = new boolean[]{false, false};
        Vector2d transError = new Vector2d(0,0);
        Vector2d transCorrection;
        double rotError;
        double rotCorrection;

        //TODO: UPDATE ODOMETRY HERE?
        //TODO: ADD IMU DATA FEED + KALMAN FILTER SYSTEM

        //Assuming the odometry has been updated. For now.
        translationController.setDeltaTime(elapsedTime);
        rotationController.setDeltaTime(elapsedTime);

        transError.sub(target_position, odo_position);
        translationController.setCurrentVectorError(transError);
        rotError = -(target_heading - odo_heading_angle);
        rotationController.setCurrentScalarError(rotError);

        //Tolerance check for translation
        if(transError.length() > translationTolerance) {
            transCorrection = translationController.updateVectorPID();
            setGlobalTargetLinearVelocity(transCorrection);
        }

        else {
            setGlobalTargetLinearVelocity(new Vector2d(0,0));
            status[0] = true;
        }

        //Tolerance check for rotation
        if(Math.abs(rotError) > rotationTolerance) {
            rotCorrection = rotationController.updateScalarPID();
            setTargetAngularVelocity(rotCorrection);
        }

        else {
            setTargetAngularVelocity(0);
            status[1] = true;
        }

        //computeVelocitiesThenDrive();
        return status;
    }

    public void trackMotionPath(MotionPath path, double elapsedTime) {
        /*translationController.setCoefficients(0.4, 0.0, 0.2);
        rotationController.setCoefficients(0.8, 0.0, 0.4);*/
        updateOdometry(elapsedTime, true);

        setTargetPosition(path.getCurrentPosition());
        setTargetHeading(path.getCurrentHeading());
        boolean rstate = applyRotationalCorrection(elapsedTime);
        boolean tstate = applyTranslationalCorrection(elapsedTime);

        //Applying corrective motion
        computeVelocitiesThenDrive();

        //Indicates that the robot has reached its current target pose.
        if(rstate && tstate) {
            path.advancePose();
            setTargetPosition(path.getCurrentPosition());
            setTargetHeading(path.getCurrentHeading());
        }
    }

}
