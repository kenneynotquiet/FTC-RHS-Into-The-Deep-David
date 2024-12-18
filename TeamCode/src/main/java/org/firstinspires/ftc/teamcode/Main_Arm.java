package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Main_Arm {
//Create motor/servo Objects
    public DcMotor armMotor;
    public DcMotor armPivot;
    public DcMotor armPivot2;
    public Servo clawPivot;
    public Servo clawPivot2;
    public Servo clawRotate;
    public CRServo intakeGo;

//    Initializing Hardware
    public void init (HardwareMap HardwareMap){
        armMotor = HardwareMap.get(DcMotor.class, "armMotor");
        armPivot = HardwareMap.get(DcMotor.class, "armPivot");
        armPivot2 = HardwareMap.get(DcMotor.class, "armPivot2");
        clawPivot = HardwareMap.get(Servo.class, "clawPivot");
        clawPivot2 = HardwareMap.get(Servo.class, "clawPivot2");
        clawRotate = HardwareMap.get(Servo.class, "clawRotate");
        intakeGo = HardwareMap.get(CRServo.class, "intakeGo");
        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
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
        Wrist,

    }
    ArmState state = ArmState.Holding;
    final double CPD = -1961.0 / 90.0;
    boolean wasCl = false;
    double armPivotDesiredPosition = 0;
    double armMotorDesiredPosition = 0;

    boolean wasLBPressed = false;
    double pivotStartPosDeg = armPivot.getCurrentPosition() / CPD;
    Ewma e = new Ewma(0.25);
    Ewma e2 = new Ewma(0.25);

    double clawPivotTarget = 0.9;
    double clawPivot2Target = 0.1;
    double clawRotateTarget = 0.48;
    double armMotorPosition = armMotor.getCurrentPosition();
    final double ticksPerInchArm = 296;
    double armPositionInches = armMotorPosition / ticksPerInchArm;
    double pivotAngleDeg = armPivot.getCurrentPosition() / CPD - pivotStartPosDeg;
    // Get the target position of the armPivot
    final double armMotorKp = 0.75;
    boolean armCl = false;

    final double armPivotKp = 1.0 / 20.0;

            if (gamepad1.dpad_left) {
        climbServo1.setPosition((.65));
        climbServo2.setPosition(.35);
    }
            if (gamepad1.dpad_right) {
        climbServo1.setPosition((.05));
        climbServo2.setPosition(.95);
    }
            if (gamepad1.left_bumper) {
        climbServo1.setPosition((.76));
        climbServo2.setPosition(.24);

    }
            if (gamepad1.x) {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 8;
        clawPivotTarget = 0.85;
        clawPivot2Target = 0.9;
        clawRotateTarget = 1;
        armCl = true;
    }

            if (gamepad1.y) {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 4;
        clawPivotTarget = 0.87;
        clawPivot2Target = 0.75;
        clawRotateTarget = 0.0;

        armCl = true;
    }

            //Intake Button
            if (gamepad1.right_bumper) {
        armMotorDesiredPosition = 4;
        armPivotDesiredPosition = 7;
//                clawPivot.setPosition(.4);
//                clawPivot2.setPosition(.44);
//                clawRotate.setPosition(.48);
        clawPivotTarget = 0.4;
        clawPivot2Target = .35;
        clawRotateTarget = .48;
        armCl = true;

    }
            if (gamepad2.y) {
//                armMotorDesiredPosition = 0;
//                armPivotDesiredPosition = 45;
//                clawPivot.setPosition(.5);
        armMotorDesiredPosition = 7.7;
        armPivotDesiredPosition = 75;
//                clawPivot.setPosition(.55);
//                clawPivot2.setPosition(.2);
//                clawRotate.setPosition(.48);
        clawPivotTarget = 0.55;
        clawPivot2Target = 0.6;
        clawRotateTarget = 0.48;
        armCl = true;

    } else if (gamepad1.a) {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 4;
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
        clawPivotTarget = 0.65;
        clawPivot2Target = 0.3;
        clawRotateTarget = 0.48;

        armCl = true;

    } else if (gamepad2.x) {
//                armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 100;
//                clawPivot.setPosition(.78);
//                clawPivot2.setPosition(.72);
//                clawRotate.setPosition(0);
//                clawPivotTarget = 0.9;
//                clawPivot2Target = .3;
//                clawRotateTarget = 0.0;

        armCl = true;

//            }while (gamepad2.right_bumper) {
//                intakeGo.setPower(-.7);
//
//            }while (gamepad2.left_bumper) {
//                intakeGo.setPower(1.1);
    } else if (gamepad2.right_bumper) {
//                armMotorDesiredPosition = 0;
//                armPivotDesiredPosition = 93;
//                clawPivot.setPosition(.78);
//                clawPivot2.setPosition(.72);
//                clawRotate.setPosition(0);
//                clawPivotTarget = 0.78;
        clawPivot2Target = .1;
//                clawRotateTarget = 0.0;

        armCl = true;
    }

    if (armCl && !wasCl) {
        state = ArmState.Retracting;
    }

    wasCl = armCl;

    // if (Math.abs(error) < .1) {
    final double pKf = 0.0;
             (armCl || true)

    {
        switch (state) {
            case Holding:
                state = ArmState.Wrist;
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
            case Wrist:
                clawPivot.setPosition(clawPivotTarget);
                clawPivot2.setPosition(clawPivot2Target);
                clawRotate.setPosition(clawRotateTarget);
        }
        double armPivotError = armPivotDesiredPosition - pivotAngleDeg;
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




}
