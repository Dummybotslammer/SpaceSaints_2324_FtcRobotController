package com.github.dummybotslammer.spacesaintsmppc.Utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import javax.vecmath.Vector2d;

public class MotionPath {
    //Paths are stored as a set of poses, which define a simple path (straight-lines only).
    //Each pose is a pair consisting of a position and heading.
    //TODO: Turn the instruction array into a mnemonic based autonomous programming system. int [instruction code, param1, param2, param3]
    //TODO: Add interrupts (i.e. for camera detection)
    private double instructions[][]; //FORMAT: {INSTRUCTION CODE, PARAM1, PARAM2, PARAM 3, PARAM 4, PARAM 5, PARAM 6}
    private DcMotorEx exmotors[];
    private DcMotorSimple simplemotors[];
    private Servo servos[];
    private int pointer;

    //Constructors
    public MotionPath() {
        //No argument constructor
        pointer = 0;
    }

    public MotionPath(double ins[][], DcMotorEx[] em, DcMotorSimple[] sm) {
        instructions = ins;
        exmotors = em;
        simplemotors = sm;

        pointer = 0;
    }

    //Getters
    public double[] getCurrentInstruction() {
        return instructions[pointer];
    }

    public int getPointer() {
        return  pointer;
    }

    public DcMotorEx[] getExMotors() {
        return exmotors;
    }

    public DcMotorSimple[] getSimpleMotors() {
        return simplemotors;
    }

    public Servo[] getServos() {
        return servos;
    }

    //Setters
    public void setInstructions(double[][] ins) {
        instructions = ins;
    }

    public void setExMotors(DcMotorEx[] m) {
        exmotors = m;
    }

    public void setSimpleMotors(DcMotorSimple[] m) {
        simplemotors = m;
    }

    public void setServos(Servo[] s) {
        servos = s;
    }

    public void setPointer(int p) {
        pointer = p;
    }

    //Methods
    public void advancePointer() {
        if (pointer >= instructions.length-1) { return; }
        pointer++;
    }
}
