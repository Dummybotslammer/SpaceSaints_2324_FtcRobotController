package com.github.dummybotslammer.spacesaintsmppc;

import javax.vecmath.Vector2d;

public final class MathUtils {
    private MathUtils() {
        //Private constructor, thus cannot be instantiated.
    }

    public static final Vector2d GLOBAL_ANGLE_REF_UNIT_VECTOR = new Vector2d(1, 0);

    public static double getGlobalAngle(Vector2d vec) {
        double angle = GLOBAL_ANGLE_REF_UNIT_VECTOR.angle(vec);
        //If the vec.y < GLOBAL_ANGLE_REF_UNIT_VECTOR.y then the angle is negative.
        if (vec.y < GLOBAL_ANGLE_REF_UNIT_VECTOR.y) {
            angle = 2*Math.PI - angle;
        }

        //If the vec.y >= GLOBAL_ANGLE_REF_UNIT_VECTOR.y, then the angle is positive and remains the same.

        if (Double.isNaN(angle)) {
            angle = 0.0;
        }

        return angle;
    }

    public static double addAngles(double r1, double r2) {
        //In radians
        double sum = r1+r2;
        if(sum < 0 ) {
            sum = 2*Math.PI + sum;
        }

        else if(sum > 2*Math.PI) {
            sum = sum - 2*Math.PI;
        }

        return sum;
    }
}
