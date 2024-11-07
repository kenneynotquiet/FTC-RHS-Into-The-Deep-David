package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Base64;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    class Ewma {
        public Ewma(double alpha) {
            this.alpha = alpha;
        }
        double alpha;
        double value;
        double calculate(double v) {
            value = alpha * v + (1 - alpha) * value;
            return value;
        }
    }

    enum ArmState {
        Extending,
        Retracting,
        Pivoting,
        Holding
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor"); //extension
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo clawPivot = hardwareMap.servo.get("clawPivot");
        CRServo intakeGo = hardwareMap.crservo.get("intakeGo");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmState state = ArmState.Holding;
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        DcMotor armPivot = hardwareMap.dcMotor.get("armEncoder");
        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        final double CPD = -1961.0 / 90.0;
        boolean wasCl = false;
        double pivotStartPosDeg = armPivot.getCurrentPosition() / CPD;
        Ewma e = new Ewma(0.25);
        Ewma e2 = new Ewma(0.25);
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            // Get the current position of the armPivot
            double armMotorPosition = armMotor.getCurrentPosition();
            final double ticksPerInchArm = 296;
            double armPositionInches = armMotorPosition / ticksPerInchArm;
            double pivotAngleDeg = armPivot.getCurrentPosition() / CPD - pivotStartPosDeg;
            telemetry.addData("Pivot Angle", pivotAngleDeg);
            // Get the target position of the armPivot
            double armMotorDesiredPosition = 0;
            final double armMotorKp = 0.75;
            boolean armCl = false;

            double armPivotDesiredPosition = 0;
            final double armPivotKp = 1.0/1.5;

            if (gamepad1.b) {
                armMotorDesiredPosition = 3;
                armPivotDesiredPosition = 45;

                armCl = true;
            } else if (gamepad1.y) {
                armMotorDesiredPosition = 0;
                armPivotDesiredPosition = 20;
                armCl = true;
            }

            if (armCl && !wasCl) {
                state = ArmState.Retracting;
            }

            wasCl = armCl;

            telemetry.addData("State", state);
            // if (Math.abs(error) < .1) {
            final double pKf = 0.4;
            if (armCl) {
                switch (state) {
                    case Holding:
                        break;
                    case Retracting:
                        armMotorDesiredPosition = 0;
                        armPivotDesiredPosition = pivotAngleDeg;
                        if (Math.abs(armMotorPosition) < .2) {
                            state = ArmState.Pivoting;
                        }
                        break;
                    case Extending:
                        armPivotDesiredPosition = pivotAngleDeg;
                        if (Math.abs(armMotorPosition - armMotorDesiredPosition) < 5) {
                            state = ArmState.Holding;
                        }
                        break;
                    case Pivoting:
                        armMotorDesiredPosition = 0;
                        if (Math.abs(armPivotDesiredPosition - pivotAngleDeg) < 5) {
                            state = ArmState.Extending;
                        }
                        break;
                }

                double armPivotError = armPivotDesiredPosition - pivotAngleDeg;
                telemetry.addData("Pivot Error", armPivotError);
                armPivotDesiredPosition = e.calculate(armPivotDesiredPosition);

                double pivotFf = pKf * Math.cos(Math.toRadians(armPivotDesiredPosition));

                armMotorDesiredPosition = e2.calculate(armMotorDesiredPosition);
                if (Math.abs(armMotorDesiredPosition) > 0) {
                    pivotFf *= (1.0 + 1.0 / 3.0 * armMotorDesiredPosition);
                }
                double armMotorError = armMotorDesiredPosition - armPositionInches;
                armMotor.setPower(armMotorKp * armMotorError);
                armPivot.setPower(armPivotKp * armPivotError + pivotFf);

            } else {
                armMotor.setPower(0);
                armPivot.setPower(pKf * Math.cos(Math.toRadians(pivotAngleDeg)));
            }
            telemetry.addData("Pivot Desired", armPivotDesiredPosition);

            if (gamepad1.dpad_left) {
                clawPivot.setPosition(.87);
            }
            if (gamepad1.dpad_right) {
                clawPivot.setPosition(.44);
            }
            if (gamepad1.dpad_down) {
                clawPivot.setPosition(.5);
            }
            if (gamepad1.right_bumper) {
                intakeGo.setPower(1);
            }
            if (gamepad1.left_bumper) {
                intakeGo.setPower(-1);
            }
            if (gamepad1.a) {
                intakeGo.setPower(0);
            }



            // Show the position of the armMotor on telemetry
            telemetry.addData("armMotor Encoder Position", armPositionInches); //281

            // Show the target position of the armMotor on telemetry
            telemetry.addData("armMotor Desired Position", 1);

            telemetry.update();


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

//        public void FieldCentricMecanumTeleOp(int turnage){
//            newTarget = ticks/turnage;
//            armEncoder.setTargetPosition((int)newTarget);
//            armEncoder.setPower(0.3);
//            armEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        public void tracker(){
//            armEncoder.setTargetPosition(0);
//            armEncoder.setPower(0.8);
//            armEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }


    }
}
