package org.firstinspires.ftc.teamcode;

import com.github.dummybotslammer.spacesaintsmppc.Controllers.DcMotorExController;
import com.github.dummybotslammer.spacesaintsmppc.Utils.MathUtils;

/*
Essentially a data / constants class for constants and info for the robot.

Info about the CONTROL SENSITIVITY STATE:
FALSE - FINE
TRUE - ROUGH
*/

public class Glockenzerstorer_RobotProfile {
    public final static int TICKS_PER_HDHEXMOTOR_REV = (int) (2.0*28.0*18.9); //2 Stage Ultraplanetary Gearboxes: 4:1 & 5:1 (Total Nominal: 20:1)
    public final static int TICKS_PER_COREHEXMOTOR_REV = 288;
    public final static double WHEEL_DIAMETER = 0.09;
    public final static double DRIVETRAIN_RADIUS = 0.380/2;
    //public final static double ROTATIONAL_DRIFT_CORRECTION_PID_ROUGH[] = new double[] {0.0, };
    public final static double CONTROL_SENSITIVITY_STATE_INDICATOR_COLORS[][] = new double[][] {{0, 0, 255}, {0, 255, 0}}; //FINE - Blue, ROUGH - Green
    public final static double ROTATIONAL_DRIFT_CORRECTION_PID[] = new double[] {1.4, 0.0, 1.6};
    public final static double MAX_ANGULAR_VELOCITIES[] = new double[] {Math.PI/6, Math.PI/3}; //FINE, ROUGH
    public final static double MOVEMENT_VECTOR_SCALE_FACTORS[] = new double[] {0.08, 0.2}; //FINE, ROUGH
    public final static int LIFT_EXTENSION_POSITIONS[] = {
            //Rest Position, It's meant to force the lift down a little, in order to ensure the outtake bucket is flush with the ground
            MathUtils.metersToTicks(0.01, 0.05496, (int)(2.0*28.0*54.8)),
            //-MathUtils.metersToTicks(0.10, 0.05496, TICKS_PER_HDHEXMOTOR_REV), -
            -MathUtils.metersToTicks(0.18, 0.05496, (int)(2.0*28.0*54.8)),
            //-MathUtils.metersToTicks(0.28, 0.05496, TICKS_PER_HDHEXMOTOR_REV),
            -MathUtils.metersToTicks(0.28+0.02 /*+0.045 */, 0.05496, (int)(2.0*28.0*54.8))
    };

    public final static double OUTTAKE_POCKET_REST_POSITION = 0.575;
    public final static double OUTTAKE_POCKET_DEPOSIT_POSITION = 1.0; //0.89
    public final static double OUTTAKE_POCKET_MID_POSITION = 0.8;
    public final static double AUTONOMOUS_TRANSLATIONAL_TOLERANCE_TIGHT = 0.01;
    public final static double AUTONOMOUS_TRANSLATIONAL_TOLERANCE_LOOSE = 0.2;
    public final static double AUTONOMOUS_HEADING_TOLERANCE_TIGHT = Math.toRadians(2.5);
    public final static double AUTONOMOUS_HEADING_TOLERANCE_LOOSE = Math.toRadians(10.0);
    //0.067024 -> Distance of the C-Channel's edge from the edge of the drivetrain (the location where the distance sensors are mounted).
    //0.01650 -> Additional thickness of the distance sensor (mounted facing the sides)
    public final static double DISTANCE_SENSOR_CENTRE_OFFSET_RIGHT = DRIVETRAIN_RADIUS - 0.067024;
    public final static double DISTANCE_SENSOR_CENTRE_OFFSET_LEFT = DRIVETRAIN_RADIUS - 0.067024;

}
