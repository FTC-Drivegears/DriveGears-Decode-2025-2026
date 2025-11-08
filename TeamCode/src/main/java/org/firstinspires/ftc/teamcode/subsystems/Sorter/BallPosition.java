//package org.firstinspires.ftc.teamcode.subsystems.Sorter;
//
//import androidx.annotation.NonNull;
//
//class BallPosition {
//    int position;
//    Color color;
//
//    BallPosition(int position, Color color) {
//        this.position = position;
//        this.color = color;
//    }
//
//    @NonNull
//    @Override
//    public String toString() {
//        return "BallPosition [pos=" + position + ", color=" + color + "]";
//    }
//
//    @Override
//    // Overriding equals is necessary for indexOf or object comparison.
//    public boolean equals(Object o) {
//
//        // If the object is compared with itself then return true
//        if (o == this) {
//            return true;
//        }
//
//        // Check if o is an instance of BallPosition or not "null instanceof [type]" also returns false
//        if (!(o instanceof BallPosition)) {
//            return false;
//        }
//
//        // typecast o to BallPosition so that we can compare data members
//        BallPosition otherCasted = (BallPosition) o;
//
//        return otherCasted.position == this.position && otherCasted.color == this.color;
//    }
//}


