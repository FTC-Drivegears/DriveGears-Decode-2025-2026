package org.firstinspires.ftc.teamcode.util;

public class Artifact {

    private BallColour colour;

    public enum BallColour {
        NONE,
        PURPLE,
        GREEN
    }

    public Artifact(BallColour colour) {
        this.colour = colour;
    }

    public BallColour getColour() {
        return this.colour;
    }

    public boolean is(BallColour colour) {
        return (this.colour == colour);
    }

    @Override
    public String toString() {
        return colour.name();
    }
}