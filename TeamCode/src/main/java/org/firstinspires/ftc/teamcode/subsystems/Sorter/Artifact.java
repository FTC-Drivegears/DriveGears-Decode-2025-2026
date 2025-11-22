package org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class Artifact {
    private char colour;
    private double position;

    public Artifact(char c, double p) {
        this.colour = c;
        this.position = p;
    }

    char getColor() { return this.colour; }

    double getPosition() { return this.position; }
}
