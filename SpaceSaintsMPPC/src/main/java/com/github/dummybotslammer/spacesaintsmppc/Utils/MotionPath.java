package com.github.dummybotslammer.spacesaintsmppc.Utils;

import javax.vecmath.Vector2d;

public class MotionPath {
    //Paths are stored as a set of poses, which define a simple path (straight-lines only).
    //Each pose is a pair consisting of a position and heading.
    private Vector2d[] positions;
    private double[] headings;
    private int pointer;

    //Constructors
    public MotionPath() {
        //No argument constructor
        pointer = 0;
    }

    public MotionPath(Vector2d[] pos, double[] heading) {
        positions = pos;
        headings = heading;
        pointer = 0;
    }

    //Getters
    public Vector2d getCurrentPosition() {
        return positions[pointer];
    }

    public double getCurrentHeading() {
        return headings[pointer];
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
        if (pointer >= positions.length-1 || pointer >= headings.length-1) { return; }
        pointer++;
    }
}
