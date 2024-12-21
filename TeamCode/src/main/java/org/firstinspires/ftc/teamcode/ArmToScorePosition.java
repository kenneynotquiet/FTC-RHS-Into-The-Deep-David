package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmToScorePosition implements Action {
    private final Main_Arm arm;
    private boolean hasToldToMove = false;
    private Telemetry telemetry;

    public ArmToScorePosition(Main_Arm arm) {
        this.arm = arm;
        this.telemetry = telemetry;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hasToldToMove) {
            arm.scoreSampleHigh();
            hasToldToMove = true;
        }

        arm.update(true, telemetry);

        return arm.getState() != Main_Arm.ArmState.Holding;
    }
}

