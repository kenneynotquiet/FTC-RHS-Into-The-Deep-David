package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class specimenScore implements Action {
    private final Main_Arm arm;
    private boolean hasToldToMove = false;
    private Telemetry telemetry;
    public specimenScore(Main_Arm arm, Telemetry telemetry) {
        this.arm = arm;
        this.telemetry = telemetry;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hasToldToMove) {
            arm.specimenScore();
            hasToldToMove = true;
        }

        arm.update(true, telemetry);

        return arm.getState() != Main_Arm.ArmState.Holding;
    }
}

