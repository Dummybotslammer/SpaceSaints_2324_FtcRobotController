package com.github.dummybotslammer.spacesaintsmppc.Utils;

import javax.vecmath.Vector2d;

public class MotionPath {
    //Paths are stored as a set of poses, which define a simple path (straight-lines only).
    //Each pose is a pair consisting of a position and heading.
    //TODO: Turn the instruction array into a mnemonic based autonomous programming system. int [instruction code, param1, param2, param3]
    //TODO: Add interrupts (i.e. for camera detection)
    private double instructions[][];
    private Vector2d[] positions;
    private double[] headings;
    private double[] rTolerances;
    private double[] tTolerances;
    private int pointer;

    //Constructors
    public MotionPath() {
        //No argument constructor
        pointer = 0;
    }

    public MotionPath(Vector2d[] pos, double[] heading, double[] tTol, double[] rTol) {
        positions = pos;
        headings = heading;
        tTolerances = tTol;
        rTolerances = rTol;

        pointer = 0;
    }

    //Getters
    public Vector2d getCurrentPosition() {
        return positions[Math.min(pointer, positions.length-1)];
    }

    public double getCurrentHeading() {
        return headings[Math.min(pointer, headings.length-1)];
    }

    public double getCurrentRotationTolerance() {
        return rTolerances[Math.min(pointer, rTolerances.length-1)];
    }

    public double getCurrentTranslationTolerance() {
        return tTolerances[Math.min(pointer, tTolerances.length-1)];
    }

    public int getPosePointer() {
        return  pointer;
    }

    //Setters
    public void setPosePointer(int p) {
        pointer = p;
    }

    //Methods
    public void advancePose() {
        //if (pointer >= positions.length-1 || pointer >= headings.length-1) { return; }
        pointer++;
    }
}
