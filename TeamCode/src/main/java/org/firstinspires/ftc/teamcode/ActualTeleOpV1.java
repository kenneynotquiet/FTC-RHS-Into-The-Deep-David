package org.firstinspires.ftc.teamcode;

import static android.provider.SyncStateContract.Helpers.update;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drawing;

//importing all the subsystems
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Main_Arm;



import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
@TeleOp
public class ActualTeleOpV1 extends LinearOpMode {

//    specimenIntake specimenIntake;

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }


    @Override
    public void runOpMode() throws InterruptedException {



        //constants and objects
//        Limelight3A limelight;
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        int[] validIDs = {3, 4};
//        //LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
        // Lets see


        final double headingKP = 0.01;

        telemetry.setMsTransmissionInterval(5);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        limelight.start();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Main_Arm myArm = new Main_Arm(hardwareMap);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {


                // Convert the robot's current heading (in radians) to degrees.
                double headingMeasurement = Math.toDegrees(drive.pose.heading.toDouble());

                // Set the desired rotational velocity based on the right stick's x-axis input (clockwise is negative).
                double headingVel = -gamepad1.right_stick_x*.6;
                double headingSetpoint = 0;
                boolean armcl = false;
                boolean closedLoop = gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up;
                if (gamepad1.back) {
                    drive.pose = new Pose2d(drive.pose.position, 0);
                }
                if (gamepad1.dpad_up) {
                    headingSetpoint = 0;
                }
                if (gamepad1.dpad_right) {
                    headingSetpoint = -90;
                }
                if (gamepad1.dpad_left) {
                    headingSetpoint = 90;
                }
                if (gamepad1.dpad_down) {
                    headingSetpoint = 180;
                }

                // If closed-loop control is enabled, calculate the error between the desired and current heading.
                if (closedLoop) {
                    // Define the error bounds within [-180, 180] degrees for the heading.
                    double errorBound = (180.0 - (-180)) / 2.0;
                    double headingError = inputModulus(headingSetpoint - headingMeasurement, -errorBound, errorBound);
                    // Adjust the rotational velocity based on the heading error and a proportional gain (headingKP).
                    headingVel = headingError * headingKP;
                }

                //Field Centric Drive
                double flippedHeading = drive.pose.heading.inverse().toDouble();

                Vector2d driveVectorNonRotated = new Vector2d(
                        -gamepad1.left_stick_y * -gamepad1.left_stick_y * -gamepad1.left_stick_y*.75,
                        -gamepad1.left_stick_x * -gamepad1.left_stick_x * -gamepad1.left_stick_x*.75
                );
//                if (Math.abs(armPositionInches) < .5) {
//                    driveVectorNonRotated = new Vector2d(
//                            -gamepad1.left_stick_y * -gamepad1.left_stick_y * -gamepad1.left_stick_y * 0.25,
//                            -gamepad1.left_stick_x * -gamepad1.left_stick_x * -gamepad1.left_stick_x * 0.25
//                    );
//                }

                Vector2d driveVectorRotated = new Vector2d(
                        driveVectorNonRotated.x * Math.cos(flippedHeading) - driveVectorNonRotated.y * Math.sin(flippedHeading),
                        driveVectorNonRotated.x * Math.sin(flippedHeading) + driveVectorNonRotated.y * Math.cos(flippedHeading)
                );

                drive.setDrivePowers(new PoseVelocity2d(
                        driveVectorRotated,
                        headingVel
                ));
                drive.updatePoseEstimate();
                myArm.update(armcl);
                if (gamepad1.x) {
                    myArm.sampleLongIntake();

                    // Extends Arm into Intake Position



                }



//                limelight.updateRobotOrientation(headingMeasurement);
//                LLResult result = limelight.getLatestResult();
//                if (result != null) {
//                    if (result.isValid()) {
//                        Pose3D botpose = result.getBotpose_MT2();
//                        telemetry.addData("llpose", botpose.toString());
//                    }
//                    else {
//                        telemetry.addData("llpose", "NONE");
//                    }
//                }
                telemetry.addData("state" , myArm.state);
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", headingMeasurement);
//                telemetry.addData("armMotor Encoder Position", armPositionInches); //281
//                telemetry.addData("armMotor Desired Position", 1);
//                telemetry.addData("clawPivot Position", clawPivot.getPosition());
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();

            }
        }
    }
}
//telemetry.addData("State", state.toString());
//telemetry.addData("Pivot Error", armPivotError);
//Put here so can see positions in teleop.

