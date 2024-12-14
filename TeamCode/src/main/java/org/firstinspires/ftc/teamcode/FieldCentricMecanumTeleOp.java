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

import kotlin.math.UMathKt;

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
        Holding,
        Wristing,

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
        Servo climbServo1 = hardwareMap.servo.get("climbServo1");
        Servo climbServo2 = hardwareMap.servo.get("climbServo2");
        DcMotor climber = hardwareMap.dcMotor.get("climber");
        Servo clawRotate = hardwareMap.servo.get("clawRotate");
        Servo clawPivot2 = hardwareMap.servo.get("clawPivot2");
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
        DcMotor armPivot = hardwareMap.dcMotor.get("armPivot");
        DcMotor armPivot2 = hardwareMap.dcMotor.get("armPivot2");
        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        final double CPD = -1961.0 / 90.0;
        boolean wasCl = false;
        double armPivotDesiredPosition = 0;
        double armMotorDesiredPosition = 0;

        boolean wasLBPressed = false;
        double pivotStartPosDeg = armPivot.getCurrentPosition() / CPD;
        Ewma e = new Ewma(0.25);
        Ewma e2 = new Ewma(0.25);
        if (isStopRequested()) return;

        //ClawPivot: Higher is upwards
        //ClawPivot: Lower is Upwards
        //set position to default state
        clawPivot.setPosition(.9);
        clawPivot2.setPosition(.1);
        clawRotate.setPosition(.48);

        double clawPivotTarget = 0.9;
        double clawPivot2Target = 0.1;
        double clawRotateTarget = 0.48;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y*-gamepad1.left_stick_y*-gamepad1.left_stick_y*.6; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x*gamepad1.left_stick_x*gamepad1.left_stick_x*.6;
            double rx = gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x*.75;

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
            final double armMotorKp = 0.75;
            boolean armCl = false;

            final double armPivotKp = 1.0 / 22.0;

            if (gamepad1.dpad_up) {
                climber.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                climber.setPower(-1.0);
            } else {
                climber.setPower(0.0);
            }

//            boolean leftBumperRising = gamepad1.left_bumper && !wasLBPressed;
//            if (leftBumperRising) {
//                climbServo1.setPosition(.62);
//                climbServo2.setPosition(.38);
//            }
//            wasLBPressed = gamepad1.left_bumper;
            if (gamepad1.dpad_left) {
                climbServo1.setPosition((.65));
                climbServo2.setPosition(.35);
            }
            if (gamepad1.dpad_right) {
                climbServo1.setPosition((.05));
                climbServo2.setPosition(.95);
            }
            if (gamepad1.left_bumper) {
                climbServo1.setPosition((.72));
                climbServo2.setPosition(.28);

            }
            if (gamepad1.x){
                armMotorDesiredPosition = 0;
                armPivotDesiredPosition = 15;
                clawPivotTarget = 0.75;
                clawPivot2Target = .82;
                clawRotateTarget = 1;
                armCl = true;
            }
            //Intake Button
             if (gamepad1.right_bumper) {
                armMotorDesiredPosition = 4;
                armPivotDesiredPosition = 10;
//                clawPivot.setPosition(.4);
//                clawPivot2.setPosition(.44);
//                clawRotate.setPosition(.48);
                clawPivotTarget = 0.4;
                clawPivot2Target = .42;
                clawRotateTarget = .48;
                armCl = true;

            }if (gamepad2.y) {
//                armMotorDesiredPosition = 0;
//                armPivotDesiredPosition = 45;
//                clawPivot.setPosition(.5);
                armMotorDesiredPosition = 7.7;
                armPivotDesiredPosition = 86;
//                clawPivot.setPosition(.55);
//                clawPivot2.setPosition(.2);
//                clawRotate.setPosition(.48);
                clawPivotTarget = 0.55;
                clawPivot2Target = 0.7;
                clawRotateTarget = 0.48;
                armCl = true;

            } else if (gamepad1.a) {
                armMotorDesiredPosition = 0;
                armPivotDesiredPosition = 10;
//                clawPivot.setPosition(.9);
//                clawPivot2.setPosition(.1);
//                clawRotate.setPosition(.48);
                clawPivotTarget = 0.9;
                clawPivot2Target = 0.1;
                clawRotateTarget = 0.48;

                armCl = true;

            } else if (gamepad2.b) {
                armMotorDesiredPosition = 3.85;
                armPivotDesiredPosition = 95;
//                clawPivot.setPosition(.8);
//                clawPivot2.setPosition(.3);
//                clawRotate.setPosition(.48);
                clawPivotTarget = 0.8;
                clawPivot2Target = 0.3;
                clawRotateTarget = 0.48;

                armCl = true;

            } else if (gamepad2.x) {
                armMotorDesiredPosition = 0;
                armPivotDesiredPosition = 93;
//                clawPivot.setPosition(.78);
//                clawPivot2.setPosition(.72);
//                clawRotate.setPosition(0);
                clawPivotTarget = 0.78;
                clawPivot2Target = 0.72;
                clawRotateTarget = 0.0;

                armCl = true;

//            }while (gamepad2.right_bumper) {
//                intakeGo.setPower(-.7);
//
//            }while (gamepad2.left_bumper) {
//                intakeGo.setPower(1.1);
            }
            if (gamepad1.y) {
                clawRotate.setPosition(.4);
            }

            if (armCl && !wasCl) {
                state = ArmState.Retracting;
            }

            wasCl = armCl;

            //i have sigma ohio rizz
            // skibidi toilet

            telemetry.addData("State", state);
            // if (Math.abs(error) < .1) {
            final double pKf = 0.0;
            if (armCl || true) {
                switch (state) {
                    case Holding:
                        state = ArmState.Wristing;
                        break;
                    case Retracting:
                        if (Math.abs(armPositionInches) > 0.35) {
                            clawPivot.setPosition(.5);
                            clawPivot2.setPosition(.5);
                            clawRotate.setPosition(.48);
                            armMotorDesiredPosition = 0;
                        }
                        armPivotDesiredPosition = pivotAngleDeg;
                        if (Math.abs(armPositionInches) < .5) {
                            state = ArmState.Pivoting;
                        }
                        break;
                    case Extending:
                        armPivotDesiredPosition = pivotAngleDeg;
                        if (Math.abs(armPositionInches - armMotorDesiredPosition) < 5) {
                            state = ArmState.Holding;
                        }
                        break;
                    case Pivoting:
                        armMotorDesiredPosition = 0;
                        if (Math.abs(armPivotDesiredPosition - pivotAngleDeg) < 5) {
                            state = ArmState.Extending;
                        }
                        break;
                    case Wristing:
                        clawPivot.setPosition(clawPivotTarget);
                        clawPivot2.setPosition(clawPivot2Target);
                        clawRotate.setPosition(clawRotateTarget);
                }
                telemetry.addData("State", state.toString());
                double armPivotError = armPivotDesiredPosition - pivotAngleDeg;
                telemetry.addData("Pivot Error", armPivotError);
                armPivotDesiredPosition = e.calculate(armPivotDesiredPosition);
                double pivotFf = pKf * Math.cos(Math.toRadians(armPivotDesiredPosition));

                armMotorDesiredPosition = e2.calculate(armMotorDesiredPosition);
//                if (Math.abs(armMotorDesiredPosition) > 0) {
//                    pivotFf *= (1.0 + 1.0 / 3.0 * armMotorDesiredPosition);
//                }
//                pivotFf = 0;
                double armMotorError = armMotorDesiredPosition - armPositionInches;
                armMotor.setPower(armMotorKp * armMotorError);
                armPivot.setPower(armPivotKp * armPivotError + pivotFf);
                armPivot2.setPower(armPivotKp * armPivotError + pivotFf);

            } else {
                armMotor.setPower(0);
                armPivot.setPower(pKf * Math.cos(Math.toRadians(pivotAngleDeg)));
                armPivot2.setPower(pKf * Math.cos(Math.toRadians(pivotAngleDeg)));
            }
            telemetry.addData("Pivot Desired", armPivotDesiredPosition);

            if (gamepad1.right_trigger > 0.2) {
                intakeGo.setPower(-1);
            } else if (gamepad1.left_trigger > 0.2) {
                    intakeGo.setPower(1);
            } else {
                intakeGo.setPower(0);
            }



            // Show the position of the armMotor on telemetry
            telemetry.addData("armMotor Encoder Position", armPositionInches); //281

            // Show the target position of the armMotor on telemetry
            telemetry.addData("armMotor Desired Position", 1);

            telemetry.addData("clawPivot Position", clawPivot.getPosition());

            telemetry.update();


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.3;  // Counteract imperfect strafing

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
