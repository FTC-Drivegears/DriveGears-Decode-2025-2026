package org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class Artifact {
    private char color;
    private double position;

    public Artifact(char c, double p) {
        this.color = c;
        this.position = p;
    }

    char getColor() { return this.color; }

    double getPosition() { return this.position; }
}
