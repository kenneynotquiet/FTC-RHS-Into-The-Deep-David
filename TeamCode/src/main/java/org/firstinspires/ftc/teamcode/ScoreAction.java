package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.Main_Arm;
import com.qualcomm.robotcore.hardware.CRServo;


public class ScoreAction implements Action {
    private final Main_Arm arm;
    private boolean hasToldToMove = false;
    CRServo intakeGo = hardwareMap.crservo.get("intakeGo");
    public ScoreAction(Main_Arm arm) {
        this.arm = arm;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hasToldToMove) {
            intakeGo.setPower(1);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intakeGo.setPower(0);
            hasToldToMove = true;
        }

        arm.update(true);

        return arm.getState() != Main_Arm.ArmState.Holding;
    }
}
