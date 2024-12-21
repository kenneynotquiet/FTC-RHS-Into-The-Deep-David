package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class specimenScore implements Action {
    private final Main_Arm arm;
    private boolean hasToldToMove = false;
    public specimenScore(Main_Arm arm) {
        this.arm = arm;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hasToldToMove) {
            arm.specimenScore();
            hasToldToMove = true;
        }

        arm.update(true);

        return arm.getState() != Main_Arm.ArmState.Holding;
    }
}

