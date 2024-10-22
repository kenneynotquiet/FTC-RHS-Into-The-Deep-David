package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Encoder Test ")
public class armEncoder extends LinearOpMode {
//    DcMotor armEncoder;
    DcMotor pivotMotor;

    //MY VARIABLES
    double ticks = 8192;
    double newTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        pivotMotor = hardwareMap.get(DcMotor.class, "armEncoder");
        telemetry.addData("Hardware: ", "Initialized");
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x){
                encoder(0.125);
            }
            telemetry.addData("Motor Ticks: ", pivotMotor.getCurrentPosition());

            if(gamepad1.a){ // this sets the pivotMotor back to its original position i think
                tracker();
            }
    }

    }

    public void encoder(double turnage){
        newTarget = ticks/turnage;
        pivotMotor.setTargetPosition((int)newTarget);
        pivotMotor.setPower(0.55);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // for some reason it doesnt stop so i added and if statement
//        if (armEncoder.getCurrentPosition() == newTarget) {
//            pivotMotor.setPower(0);
//        }
    }

    public void tracker(){
        pivotMotor.setTargetPosition(0);
        pivotMotor.setPower(0.8);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
