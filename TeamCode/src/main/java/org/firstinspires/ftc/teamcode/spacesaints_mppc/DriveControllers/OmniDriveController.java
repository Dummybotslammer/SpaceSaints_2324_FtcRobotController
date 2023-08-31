package org.firstinspires.ftc.teamcode.spacesaints_mppc.DriveControllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import javax.vecmath.Vector2d;

public class OmniDriveController {
    //The OmniDrive runs on 3 different motors.
    //The motors need to be set to DcMotor.RunMode.RUN_USING_ENCODER mode.
    //All physical quantities are in SI units, which are then converted to the encoder counts for the purpose of the motors.
    //The forward heading of the robot is (0, 1)
    private DcMotorEx drive1;
    private DcMotorEx drive2;
    private DcMotorEx drive3;

    //Unit Vectors with length 1.
    private Vector2d drive1_unitVector = new Vector2d(0.5, Math.sqrt(0.75));
    private Vector2d drive2_unitVector = new Vector2d(0.5, -Math.sqrt(0.75));
    private Vector2d drive3_unitVector = new Vector2d(-1.0, 0.0);

    //Physical Measurements
    private Vector2d linear_velocity = new Vector2d(0,0);
    private Vector2d drive1_velocity = new Vector2d(0,0);
    private Vector2d drive2_velocity = new Vector2d(0,0);
    private Vector2d drive3_velocity = new Vector2d(0,0);
    private double drive1_speed = 0.0;
    private double drive2_speed = 0.0;
    private double drive3_speed = 0.0;
    private double angular_velocity = 0.0;

    //Odometric Measurements
    private Vector2d odo_position = new Vector2d(0,0);
    private double odo_heading_angle = 0.0;
    private int drive1_currentPosition = 0;
    private int drive2_currentPosition = 0;
    private int drive3_currentPosition = 0;
    private int drive1_prevPosition = 0;
    private int drive2_prevPosition = 0;
    private int drive3_prevPosition = 0;
    private double odo_deltaTime = 0.0;

    //DriveBase Characteristics
    private double wheelDiameter = 0.09;
    private double inscribedBasedRadius = Math.sqrt(14699.905641)/10/100; //mm -> cm -> m
    private double drive1_trackWidth = Math.sqrt(132299.878225)/10/100; //mm -> cm -> m
    private double drive2_trackWidth = Math.sqrt(132299.878225)/10/100; //mm -> cm -> m
    private double drive3_trackWidth = 363.731/10/100; //mm -> cm -> m
    private int TICKS_PER_DRIVE1_REV = 28;
    private int TICKS_PER_DRIVE2_REV = 28;
    private int TICKS_PER_DRIVE3_REV = 28;

    //Constructors
    public OmniDriveController() {
        //No argument Constructor
        //TODO: Improve the constructors
    }

    public OmniDriveController(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3) {
        drive1 = motor1;
        drive2 = motor2;
        drive3 = motor3;
    }

    public OmniDriveController(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, int TICKS_PER_MOTOR1_REV, int TICKS_PER_MOTOR2_REV, int TICKS_PER_MOTOR3_REV, double wheel_diameter) {
        drive1 = motor1;
        drive2 = motor2;
        drive3 = motor3;

        TICKS_PER_DRIVE1_REV = TICKS_PER_MOTOR1_REV;
        TICKS_PER_DRIVE2_REV = TICKS_PER_MOTOR2_REV;
        TICKS_PER_DRIVE3_REV = TICKS_PER_MOTOR3_REV;

        wheelDiameter = wheel_diameter;
    }

    //Setter/Getter Methods
    public void setLinearVelocity(Vector2d vector) {
        linear_velocity = vector;
    }

    public void setAngularVelocity(double w) {
        angular_velocity = w;
    }

    public void setDriveMotors(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3) {
        drive1 = motor1;
        drive2 = motor2;
        drive3 = motor3;
    }

    public Vector2d[] getDriveVelocities() {
        return new Vector2d[]{drive1_velocity, drive2_velocity, drive3_velocity};
    }

    public Vector2d[] getUnitVectors() {
        return new Vector2d[]{drive1_unitVector, drive2_unitVector, drive3_unitVector};
    }

    public double[] getDriveSpeeds() {
        return new double[]{drive1_speed, drive2_speed, drive3_speed};
    }

    public Vector2d getHeading_velocity() {
        return linear_velocity;
    }

    public Vector2d getPosition() {return odo_position;}

    public double getHeadingAngle() {return odo_heading_angle;}

    //Computation Methods
    public void computeDriveVelFromHeadingAndAngularVel() {
        Vector2d drive1_turnVel = new Vector2d(drive1_unitVector);
        Vector2d drive2_turnVel = new Vector2d(drive2_unitVector);
        Vector2d drive3_turnVel = new Vector2d(drive3_unitVector);
        drive1_velocity = new Vector2d(drive1_unitVector);
        drive2_velocity = new Vector2d(drive2_unitVector);
        drive3_velocity = new Vector2d(drive3_unitVector);
        double turnSpeed_per_motor = angular_velocity*inscribedBasedRadius;

        drive1_turnVel.scale(turnSpeed_per_motor);
        drive2_turnVel.scale(turnSpeed_per_motor);
        drive3_turnVel.scale(turnSpeed_per_motor);

        drive1_velocity.scale(drive1_unitVector.dot(linear_velocity));
        drive2_velocity.scale(drive2_unitVector.dot(linear_velocity));
        drive3_velocity.scale(drive3_unitVector.dot(linear_velocity));

        drive1_velocity.add(drive1_turnVel);
        drive2_velocity.add(drive2_turnVel);
        drive3_velocity.add(drive3_turnVel);

        //Boolean Logic: (angle > 0)? TRUE=-1:FALSE=1
        //Returns -1 if the angle is positive, and 1 if the angle is 0 or less.
        drive1_speed = drive1_velocity.length() * ((drive1_velocity.angle(drive1_unitVector) > 0)? -1:1);
        drive2_speed = drive2_velocity.length() * ((drive2_velocity.angle(drive2_unitVector) > 0)? -1:1);
        drive3_speed = drive3_velocity.length() * ((drive3_velocity.angle(drive3_unitVector) > 0)? -1:1);
    }

    public void updateOdometry(double deltaTime) {
        //Update readingss
        odo_deltaTime = deltaTime;
        drive1_currentPosition = drive1.getCurrentPosition();
        drive2_currentPosition = drive2.getCurrentPosition();
        drive3_currentPosition = drive3.getCurrentPosition();

        //Calculate displacement
        Vector2d drive1_displacement = new Vector2d(drive1_unitVector);
        Vector2d drive2_displacement = new Vector2d(drive2_unitVector);
        Vector2d drive3_displacement = new Vector2d(drive3_unitVector);
        Vector2d net_displacement = new Vector2d(0,0);

        double drive1_distance = ((double)(drive1_currentPosition-drive1_prevPosition)/TICKS_PER_DRIVE1_REV)*(Math.PI*wheelDiameter);
        double drive2_distance = ((double)(drive2_currentPosition-drive2_prevPosition)/TICKS_PER_DRIVE2_REV)*(Math.PI*wheelDiameter);
        double drive3_distance = ((double)(drive3_currentPosition-drive3_prevPosition)/TICKS_PER_DRIVE3_REV)*(Math.PI*wheelDiameter);

        //Scales the unit vectors to represent the actual displacement.
        //The scale factor is calculated by:
        //1. Find difference between the currentPosition and prevPosition.
        //2. Dividing the result by the TICKS_PER_DRIVEX_REV to calculate the number of rotations.
        //3. Multiplying by the wheelDiameter to find the actual magnitude of displacement (in metres).
        drive1_displacement.scale(drive1_distance);
        drive2_displacement.scale(drive2_distance);
        drive3_displacement.scale(drive3_distance);

        //Compute vector sum of displacements.
        net_displacement.add(drive1_displacement, drive2_displacement);
        net_displacement.add(drive3_displacement);
        odo_position.add(net_displacement);

        //Calculate current heading angle.
        double drive1_deltaAngle = drive1_distance/drive1_trackWidth;
        double drive2_deltaAngle = drive2_distance/drive2_trackWidth;
        double drive3_deltaAngle = drive3_distance/drive3_trackWidth;
        double net_deltaAngle = drive1_deltaAngle+drive2_deltaAngle+drive3_deltaAngle;

        odo_heading_angle += net_deltaAngle;

        drive1_prevPosition = drive1_currentPosition;
        drive2_prevPosition = drive2_currentPosition;
        drive3_prevPosition = drive3_currentPosition;
    }

    //Movement Methods
    public void drive() {
        //Sets the robot's driving motor's velocities to the ALREADY computed values
        //Resets the computed drive1_velocity, drive2_velocity, drive3_velocity values
        int drive1_tickspeed = (int)(drive1_speed/wheelDiameter*TICKS_PER_DRIVE1_REV);
        int drive2_tickspeed = (int)(drive2_speed/wheelDiameter*TICKS_PER_DRIVE2_REV);
        int drive3_tickspeed = (int)(drive3_speed/wheelDiameter*TICKS_PER_DRIVE3_REV);

        drive1.setVelocity(drive1_tickspeed);
        drive2.setVelocity(drive2_tickspeed);
        drive3.setVelocity(drive3_tickspeed);
    }

    public void computeVelocitiesThenDrive() {
        //Recomputes the vectors, and then drives the motors.
        computeDriveVelFromHeadingAndAngularVel();
        drive();
    }
}
