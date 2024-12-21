package org.firstinspires.ftc.teamcode;
import static android.provider.SyncStateContract.Helpers.update;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Base64;

import kotlin.math.UMathKt;


public final class Main_Arm {
    //Create motor/servo Objects
    public DcMotorEx armMotor;
    public DcMotorEx armPivot;
    public DcMotorEx armPivot2;
    public final Servo clawPivot;
    public final Servo clawPivot2;
    public final Servo clawRotate;
    public final CRServo intakeGo;
    double pivotStartPosDeg;
    double armMotorPosition;
    double pivotAngleDeg;

    enum ArmState {
        Extending,
        Retracting,
        Pivoting,
        Holding,
        Wrist,

    }

    ArmState state = ArmState.Holding;

    //    Initializing Hardware
    public Main_Arm(@NonNull HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        this.armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
        this.armPivot2 = hardwareMap.get(DcMotorEx.class, "armPivot2");
        this.clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        this.clawPivot2 = hardwareMap.get(Servo.class, "clawPivot2");
        this.clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        this.intakeGo = hardwareMap.get(CRServo.class, "intakeGo");
        this.armPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.armPivot2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pivotStartPosDeg = armPivot.getCurrentPosition() / CPD;
        armMotorPosition = armMotor.getCurrentPosition();
        pivotAngleDeg = armPivot.getCurrentPosition() / CPD - pivotStartPosDeg;
    }

    public DcMotorEx getArmMotor() {
        return this.armMotor;
    }
    public DcMotorEx getArmPivot() {
        return this.armPivot;
    }
    public DcMotorEx getArmPivot2() {
        return this.armPivot2;
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


    final double CPD = -8192.0 / 360.0;//-1961.0 / 90.0;
    boolean wasCl = false;

    double armPivotDesiredPosition = 0;
    double armMotorDesiredPosition = 0;


    Ewma e = new Ewma(0.25);
    Ewma e2 = new Ewma(0.25);

    double clawPivotTarget = 0.9;
    double clawPivot2Target = 0.1;
    double clawRotateTarget = 0.48;
    final double ticksPerInchArm = 296;
    double armPositionInches = armMotorPosition / ticksPerInchArm;
    // Get the target position of the armPivot
    final double armMotorKp = 0.75;
    boolean armCl = false;

    final double armPivotKp = 1.0 / 10.0;


    //
    public void specimenIntake() {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 8;
        clawPivotTarget = 0.85;
        clawPivot2Target = 0.9;
        clawRotateTarget = 1;
        armCl = true;
        state = ArmState.Retracting;
        // Gamepad 1 LB
    }

    public void specimenPrep() {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 4;
        clawPivotTarget = 0.87;
        clawPivot2Target = 0.75;
        clawRotateTarget = 0.0;

        armCl = true;
        state = ArmState.Retracting;
        // Gamepad 2 LB
    }

    public void sampleLongIntake() {
        armMotorDesiredPosition = 4;
        armPivotDesiredPosition = 6;
        clawPivotTarget = 0.37;
        clawPivot2Target = .35;
        clawRotateTarget = .48;
        armCl = true;
        state = ArmState.Retracting;
        // Gamepad 1 Right Bumper
    }

    public void scoreSampleHigh() {
        armMotorDesiredPosition = 7.7;
        armPivotDesiredPosition = 75;
        clawPivotTarget = 0.55;
        clawPivot2Target = 0.6;
        clawRotateTarget = 0.48;
        armCl = true;
        state = ArmState.Retracting;
        // Gamepad 2 Y
    }

    public void normalStow() {
        armMotorDesiredPosition = 0;
        armPivotDesiredPosition = 4;
        clawPivotTarget = 0.9;
        clawPivot2Target = 0.1;
        clawRotateTarget = 0.48;

        armCl = true;
        state = ArmState.Retracting;
        // Gamepad 1 A
    }

    public void scoreSampleMid() {
        armMotorDesiredPosition = 3.85;
        armPivotDesiredPosition = 95;
        clawPivotTarget = 0.65;
        clawPivot2Target = 0.3;
        clawRotateTarget = 0.48;

        armCl = true;
        state = ArmState.Retracting;
        // Gamepad 2 B
    }

    public void specimenScore() {
        armPivotDesiredPosition = 100;

        armCl = true;
        // Gamepad 2 X
    }

    public void update(boolean armCl, Telemetry telemetry) {

        if (armCl && !wasCl) {
            state = ArmState.Retracting;
        }
        armPositionInches = armMotor.getCurrentPosition() / ticksPerInchArm;
        wasCl = armCl;
        // if (Math.abs(error) < .1) {
        double armDesiredLocal = armMotorDesiredPosition;
        double pivotDesiredLocal = armPivotDesiredPosition;
        pivotAngleDeg = armPivot.getCurrentPosition() / CPD - pivotStartPosDeg;

        final double pKf = 0.0;
        if (armCl || true) {
            switch (state) {
                case Holding:
                    state = ArmState.Wrist;
                    break;
                case Retracting:
                    if (Math.abs(armPositionInches) > 0.35) {
                        clawPivot.setPosition(.5);
                        clawPivot2.setPosition(.5);
                        clawRotate.setPosition(.48);
                        armDesiredLocal = 0;
                    }
                    pivotDesiredLocal = pivotAngleDeg;
                    if (Math.abs(armPositionInches) < .5) {
                        state = ArmState.Pivoting;
                    }
                    break;
                case Extending:
                    if (Math.abs(armPositionInches - armDesiredLocal) < 5) {
                        state = ArmState.Holding;
                    }
                    break;
                case Pivoting:
                    armDesiredLocal = 0;
                    if (Math.abs(pivotDesiredLocal - pivotAngleDeg) < 5) {
                        state = ArmState.Extending;
                    }
                    break;
                case Wrist:
                    clawPivot.setPosition(clawPivotTarget);
                    clawPivot2.setPosition(clawPivot2Target);
                    clawRotate.setPosition(clawRotateTarget);

            }
            double armPivotError = pivotDesiredLocal - pivotAngleDeg;
            pivotDesiredLocal = e.calculate(pivotDesiredLocal);
            double pivotFf = pKf * Math.cos(Math.toRadians(pivotDesiredLocal));

            armDesiredLocal = e2.calculate(armDesiredLocal);
//                if (Math.abs(armMotorDesiredPosition) > 0) {
//                    pivotFf *= (1.0 + 1.0 / 3.0 * armMotorDesiredPosition);
//                }
//                pivotFf = 0;
            double armMotorError = armDesiredLocal - armPositionInches;
            armMotor.setPower(armMotorKp * armMotorError);
            armPivot.setPower(armPivotKp * armPivotError + pivotFf);
            armPivot2.setPower(armPivotKp * armPivotError + pivotFf);

            telemetry.addData("Pivot Error", armPivotError);
            telemetry.addData("armMotor Error", armMotorError);

        } else {
            armMotor.setPower(0);
            armPivot.setPower(pKf * Math.cos(Math.toRadians(pivotAngleDeg)));
            armPivot2.setPower(pKf * Math.cos(Math.toRadians(pivotAngleDeg)));
        }

        telemetry.addData("State", state.toString());
        telemetry.addData("clawPivot Position", clawPivot.getPosition());
        telemetry.addData("clawPivot2 Position", clawPivot2.getPosition());
        telemetry.addData("clawRotate Position", clawRotate.getPosition());
        telemetry.addData("clawPivot Desired Position", clawPivotTarget);
        telemetry.addData("clawPivot2 Desired Position", clawPivot2Target);
        telemetry.addData("clawRotate Desired Position", clawRotateTarget);
        telemetry.addData("armMotor Encoder Position", armPositionInches);
        telemetry.addData("armPivot Encoder Position", pivotAngleDeg);
        telemetry.addData("armMotor Desired Position", armMotorDesiredPosition);
        telemetry.addData("armPivot Desired Position", armPivotDesiredPosition);


    }
//    public void (gamepad1.x){
//        armMotorDesiredPosition = 0;
//        armPivotDesiredPosition = 8;
//        clawPivotTarget = 0.85;
//        clawPivot2Target = 0.9;
//        clawRotateTarget = 1;
//        armCl = true;
}




