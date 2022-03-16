package com.example.teamcode_8087.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.example.teamcode_8087.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.List;

public final class WaitSegment extends SequenceSegment {
    public WaitSegment(Pose2d pose, double seconds, List<TrajectoryMarker> markers) {
        super(seconds, pose, pose, markers);
    }
}
