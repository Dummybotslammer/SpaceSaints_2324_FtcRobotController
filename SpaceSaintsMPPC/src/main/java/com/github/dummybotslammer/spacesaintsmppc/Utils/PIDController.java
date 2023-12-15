package com.github.dummybotslammer.spacesaintsmppc.Utils;

import javax.vecmath.Vector2d;

public class PIDController {
    //P.I.D.F. Controller using numerical analysis for the calculus.
    //TODO: Implement the Feedforward (open-loop) at a later date. Implement the PID first.
    //TODO: Implement vector calc.
    //PID Coefficients
    private double Kp;
    private double Ki;
    private double Kd;

    //PID Values
    private double P;
    private double current_I;
    private double previous_I;
    private double D;

    private Vector2d vec_P;
    private Vector2d vec_I;
    private Vector2d vec_D;

    //Variables:
    private double deltaTime;
    private double current_error;
    private double previous_error;
    private double correction;

    private Vector2d current_vec_error;
    private Vector2d previous_vec_error;
    private Vector2d vec_correction;

    //Constructors
    public PIDController() {
        //Empty Constructor
        Kp = 0.0;
        Ki = 0.0;
        Kd = 0.0;

        P = 0.0;
        current_I = 0.0;
        previous_I = 0.0;
        D = 0.0;
        correction= 0;

        vec_P = new Vector2d(0,0);
        vec_I = new Vector2d(0,0);
        vec_D = new Vector2d(0,0);
        vec_correction = new Vector2d(0,0);

        deltaTime = 0.0;
        current_error = 0.0;
        previous_error = 0.0;

        current_vec_error = new Vector2d(0,0);
        previous_vec_error = new Vector2d(0,0);
    }

    public PIDController(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        P = 0.0;
        current_I = 0.0;
        previous_I = 0.0;
        D = 0.0;
        correction= 0;

        vec_P = new Vector2d(0,0);
        vec_I = new Vector2d(0,0);
        vec_D = new Vector2d(0,0);
        vec_correction = new Vector2d(0,0);

        deltaTime = 0.0;
        current_error = 0.0;
        previous_error = 0.0;

        current_vec_error = new Vector2d(0,0);
        previous_vec_error = new Vector2d(0,0);
    }

    //Getters
    public double[] getCoefficients() {
        return new double[]{Kp, Ki, Kd};
    }

    public double[] getScalarPIDValues() {
        return new double[]{P, current_I, previous_I, D};
    }

    public Vector2d[] getVectorPIDValues() {
        return new Vector2d[]{vec_P, vec_I, vec_D};
    }

    public double getCurrentScalarError(){
        return current_error;
    }

    public double getPreviousScalarError() {
        return previous_error;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    public Vector2d getCurrentVectorError() {
        return current_vec_error;
    }

    public Vector2d getPreviousVectorError() {
        return previous_vec_error;
    }

    public double getScalarCorrection() {
        return correction;
    }

    public Vector2d getVectorCorrection() {
        return vec_correction;
    }

    //Setters
    public void setCoefficients(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public void setScalarPIDValues(double p, double current_i, double previous_i, double d) {
        P = p;
        current_I = current_i;
        previous_I = previous_i;
        D = d;
    }

    public void setVectorPIDValues(Vector2d p, Vector2d i, Vector2d d) {
        vec_P = p;
        vec_I = i;
        vec_D = d;
    }

    public void setCurrentScalarError(double error){
        current_error = error;
    }

    public void setPreviousScalarError(double error) {
        previous_error = error;
    }

    public void setDeltaTime(double dt) {
        deltaTime = dt;
    }

    public void setCurrentVectorError(Vector2d error) {
        current_vec_error = error;
    }

    public void setPreviousVectorError(Vector2d error) {
        previous_vec_error = error;
    }

    //Computation Methods
    public double updateScalarPID() {
        P = Kp * current_error;
        current_I = Ki * MathUtils.numericalIntegrate(current_error, previous_error, deltaTime, previous_I);
        D = Kd * MathUtils.numericalDerivate(current_error, previous_error, deltaTime);

        //Correction: Sum of the PID values.
        correction = P + current_I + D;

        //The current values are now assigned to the 'previous' variables
        previous_I = current_I;
        previous_error = current_error;

        return correction;
    }

    public Vector2d updateVectorPID() {
        vec_P = new Vector2d(current_vec_error);
        vec_P.scale(Kp);

        vec_I.normalize(current_vec_error);
        current_I = MathUtils.vectorIntegrate(current_vec_error, current_vec_error, previous_vec_error, deltaTime, previous_I);
        vec_I.scale(current_I);
        vec_I.scale(Ki);

        vec_D = MathUtils.vectorDerivate(current_vec_error, previous_vec_error, deltaTime);
        vec_D.scale(Kd);

        //Correction Vector: Sum of the PID vectors.
        vec_correction.add(vec_P, vec_I);
        vec_correction.add(vec_D);

        //The current values are now assigned to the 'previous' variables
        previous_I = current_I;
        previous_vec_error = new Vector2d(current_vec_error);

        return vec_correction;
    }
}
