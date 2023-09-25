package com.github.dummybotslammer.spacesaintsmppc.Utils;

import java.util.Vector;

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

    public static double subAngles(double r1, double r2) {
        //In radians
        double sum = r1-r2;
        if(sum < 0 ) {
            sum = 2*Math.PI + sum;
        }

        else if(sum > 2*Math.PI) {
            sum = sum - 2*Math.PI;
        }

        return sum;
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

    public static double numericalDerivate(double current_y, double previous_y, double dx) {
        //Given a small dx, the current_y - previous_y is essentially dy.
        double d = (current_y-previous_y)/dx;
        return d;
    }

    public static double numericalIntegrate(double current_y, double previous_y, double dx, double previous_integral) {
        /*
        This formula basically takes the average between the two measurements of error to achieve an average,
        which creates more continuous, consistent, and accurate values. The average error is the y.
        The y is then multiplied with dx to get the area (integral), and added with the previous integral
        to get the integral from 0 to x.
        */
        double i = ((dx/2) * (current_y + previous_y)) + previous_integral;
        return i;
    }

    public static double numericalIntegrate(double current_y, double previous_y, double dx) {
        double i = ((dx/2) * (current_y + previous_y));
        return i;
    }

    public static Vector2d vectorDerivate(Vector2d current_vec, Vector2d previous_vec, double dt) {
        //Uses vector-valued function derivatives.
        //r(t) = [x(t), y(t)]
        //r'(t) = (r(t+dt) - r(t)) / dt

        current_vec.sub(previous_vec);
        current_vec.scale(1/dt);
        return current_vec;
    }

    public static double vectorIntegrate(Vector2d field_vec, Vector2d current_vec, Vector2d previous_vec, double dt, double previous_integral) {
        //Computes the integral of a vector in a vector field, which then becomes the coefficient of the 'error' unit vector.
        //The ds variable is a vector that points in the direction of a slight change that occurs in a path.
        //In this case, that path is the robot's movement, and the ds is the vector tangential to current point on the path.
        Vector2d ds = vectorDerivate(current_vec, previous_vec, dt);
        double dot = field_vec.dot(ds);
        dot += previous_integral;
        return dot;
    }

    public static double vectorIntegrate(Vector2d vec, Vector2d ds) {
        //Computes the integral of a vector in a vector field, which then becomes the coefficient of the 'error' unit vector.
        double dot = vec.dot(ds);
        return dot;
    }
}
