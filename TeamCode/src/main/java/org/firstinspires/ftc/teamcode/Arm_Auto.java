package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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

    public Arm_Auto(DcMotorEx armPivot,
                    Servo clawPivot,
                    Servo clawPivot2,
                    Servo clawRotate) {
        this.armPivot = armPivot;
        this.clawPivot = clawPivot;
        this.clawPivot2 = clawPivot2;
        this.clawRotate = clawRotate;
        
    }

    public void setToPositionX() {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 93;
        clawPivot.setPosition(.78);
        clawPivot2.setPosition(.72);
        clawRotate.setPosition(0);
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
