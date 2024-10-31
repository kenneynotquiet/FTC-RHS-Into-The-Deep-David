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
    DcMotor armPivot;
    //DcMotor armEncoder;

    //MY VARIABLES
    int ticks = 8192/2;
    double newTarget = 2048;

    // Position of the arm when it's lifted
    int armUpPosition = -1920;

    // Position of the arm when it's down
    int armDownPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        armPivot = hardwareMap.dcMotor.get("armEncoder");
        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double CPR = 28 * 5.23 * 5.23 * 2;

            int encoderValue = armPivot.getCurrentPosition();
            double revolutions = encoderValue/CPR;

            double angle = revolutions * 360;
            double angleNormalized = angle % 360;

            // If the X button is pressed, raise the arm.
            if (gamepad1.x && ((-1* encoderValue) <= 1920.0)) {
                if((-1* encoderValue) >= 1820.0){
                    double differencePOS = 1920.0 - (-1* encoderValue);
                    double logval = Math.log10(differencePOS);
                    armPivot.setPower(Math.pow(0.3, logval));
                    telemetry.addData("logval", logval);
                } else {
                    armPivot.setPower(0.6);
                }

            }

            // If the A button is pressed, lower the arm
            else if (gamepad1.a && ((-1* encoderValue) >= 4.0))  {
                armPivot.setPower(-0.6);

            }
            else {
                armPivot.setPower(0);
            }

            // Get the current position of the armPivot
            double position = armPivot.getCurrentPosition();

            // Get the target position of the armPivot
            double desiredPosition = armPivot.getTargetPosition();

            // Show the position of the armPivot on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armPivot on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            //telemetry.update();
            telemetry.addData("encoder Ticks: ", encoderValue);
            telemetry.addData("motor revs: ", revolutions);
            telemetry.addData("arm angle: ", angle);

            telemetry.update();

    }

    }

/**
    public void return_position(){
        armEncoder.setTargetPosition(0);
        armEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armEncoder.setPower(0.1);
    }
**/
}
