package com.github.dummybotslammer.spacesaintsmppc.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class DcMotorExController {
    private DcMotorEx motor;

    //Constructors:
    public DcMotorExController() {
    }

    public DcMotorExController(DcMotorEx motorEx) {
        motor = motorEx;
    }

    //Setters:
    public void setMotor(DcMotorEx motorEx) {
        motor = motorEx;
    }
    public void setMode(DcMotor.RunMode runmode) {
        motor.setMode(runmode);
    }

    //Getters:
    public DcMotorEx getMotor() {
        return motor;
    }

    //Methods:
    public void stopAndResetMotorEncoders() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Static Methods:
    public static void togglePower(DcMotorSimple m, double pow1, double pow2) {
        if (m.getPower() == pow1) {
            m.setPower(pow2);
        }

        else {
            m.setPower(pow1);
        }
    }

    public static void toggleServoPosition(Servo s, double pos1, double pos2) {
        if (s.getPosition() == pos1) {
            s.setPosition(pos2);
        }

        else {
            s.setPosition(pos1);
        }
    }

    public static void togglePosition(DcMotorEx m, int pos1, int pos2) {
        if (m.getTargetPosition() == pos1) {
            m.setTargetPosition(pos2);
        }

        else {
            m.setTargetPosition(pos1);
        }
    }

    public static int metersToTicks(double meters, double wheeldiameter, int ticksPerRev) {
        return (int)(meters/(Math.PI*wheeldiameter) * ticksPerRev);
    }
}
