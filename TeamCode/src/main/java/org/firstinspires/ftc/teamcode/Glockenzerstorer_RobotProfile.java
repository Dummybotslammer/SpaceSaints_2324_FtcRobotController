package org.firstinspires.ftc.teamcode;

import com.github.dummybotslammer.spacesaintsmppc.Controllers.DcMotorExController;
import com.github.dummybotslammer.spacesaintsmppc.Utils.MathUtils;

//Essentially a data / constants class for constants and info for the robot.
public class Glockenzerstorer_RobotProfile {
    public final static int TICKS_PER_HDHEXMOTOR_REV = (int) (2.0*28.0*18.9); //2 Stage Ultraplanetary Gearboxes: 4:1 & 5:1 (Total Nominal: 20:1)
    public final static int TICKS_PER_COREHEXMOTOR_REV = 288;
    public final static double WHEEL_DIAMETER = 0.09;
    public final static int LIFT_EXTENSION_POSITIONS[] = {
            0, // Rest
            -MathUtils.metersToTicks(0.25, 0.05496, TICKS_PER_HDHEXMOTOR_REV) //Yes
    };

    public final static double OUTTAKE_POCKET_REST_POSITION = 0.5;
    public final static double OUTTAKE_POCKET_DEPOSIT_POSITION = 0.95;

}
