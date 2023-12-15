package com.github.dummybotslammer.spacesaintsmppc.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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


}
