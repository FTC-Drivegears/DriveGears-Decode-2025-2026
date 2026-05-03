package org.firstinspires.ftc.teamcode.util;

public class Artifact {
    private String colour;
    public Artifact(String colour) {
        this.colour = colour;
    }
    public String getColour(){
        return this.colour;
    }

    @Override
    public String toString() {
        return colour; // Return the string you want to see
    }


}

