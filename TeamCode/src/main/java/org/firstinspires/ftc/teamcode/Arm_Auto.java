package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.FieldCentricMecanumTeleOp;

public class Arm_Auto {
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

    private ArmState state;
    private double armMotorDesiredPosition;
    private double armPivotDesiredPosition;
    private DcMotorEx armPivot;
    private Servo clawPivot;
    private Servo clawPivot2;
    private Servo clawRotate;
    private Servo clawRotateTarget;

    public Arm_Auto(DcMotorEx armPivot,
                    Servo clawPivot,
                    Servo clawPivot2,
                    Servo clawRotate,
                    Servo clawRotateTarget) {

        this.armPivot = armPivot;
        this.clawPivot = clawPivot;
        this.clawPivot2 = clawPivot2;
        this.clawRotate = clawRotate;
        this.clawRotateTarget = clawRotateTarget;
    }


    public void GP1setToPositionX () {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 15;
        clawPivot.setPosition(.75);
        clawPivot2.setPosition(.82);
        clawRotateTarget.setPosition(1);
    }
    public void GP1setToPositionRightBumper () {
        armMotorDesiredPosition = 4;
        armPivotDesiredPosition = 10;
        clawPivot.setPosition(.4);
        clawPivot2.setPosition(.44);
        clawRotate.setPosition(.48);
    }

    public void GP2setToPositionY () {
        armMotorDesiredPosition = 7.7;
        armPivotDesiredPosition = 86;
        clawPivot.setPosition(.55);
        clawPivot2.setPosition(.2);
        clawRotate.setPosition(.48);
    }

    public void GP1setPositionA ()  {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 10;
        clawPivot.setPosition(.9);
        clawPivot2.setPosition(.1);
        clawRotate.setPosition(.48);
    }

    public void GP2setToPositionB() {
        armMotorDesiredPosition = 3.85;
        armPivotDesiredPosition = 95;
        clawPivot.setPosition(.8);
        clawPivot2.setPosition(.3);
        clawRotate.setPosition(.48);
    }

    public void GP2setToPositionX() {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 93;
        clawPivot.setPosition(.78);
        clawPivot2.setPosition(.72);
        clawRotate.setPosition(0);
    }

    public void GP1setToPositionY () {
        clawRotate.setPosition(.4);
    }

    public void update() {
        double pivotAngleDeg = armPivot.getCurrentPosition();
        switch (state) {
            case Holding:
                break;
            case Retracting:
                armMotorDesiredPosition = 0;
                armPivotDesiredPosition = pivotAngleDeg;
                if (Math.abs(armPositionInches) < .5) {
                    state = FieldCentricMecanumTeleOp.ArmState.Pivoting;
                }
                break;
            case Extending:
                armPivotDesiredPosition = pivotAngleDeg;
                if (Math.abs(armPositionInches - armMotorDesiredPosition) < 5) {
                    state = FieldCentricMecanumTeleOp.ArmState.Holding;
                }
                break;
            case Pivoting:
                armMotorDesiredPosition = 0;
                if (Math.abs(armPivotDesiredPosition - pivotAngleDeg) < 5) {
                    state = FieldCentricMecanumTeleOp.ArmState.Extending;
                }
                break;
        }

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

    }
}
