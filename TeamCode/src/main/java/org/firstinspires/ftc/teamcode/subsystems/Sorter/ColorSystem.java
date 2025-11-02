//package org.firstinspires.ftc.teamcode.subsystems.Sorter;
//
//import java.util.ArrayList;
//
//class ColorSystem {
//    ArrayList<BallPosition> mockBallPositions = new ArrayList<BallPosition>();
//
//    ColorSystem(String pattern) {
//        int i = 1;
//        for (char c : pattern.toCharArray()) {
//            this.mockBallPositions.add(new BallPosition(i, charToColor(c)));
//            i++;
//        }
//    }
//
//    static Color charToColor(char c) {
//        switch (Character.toLowerCase(c)) {
//            case 'p':
//                return Color.PURPLE;
//            case 'g':
//                return Color.GREEN;
//        }
//
//        return Color.INVALID;
//    }
//
//    ArrayList<BallPosition> getBallPositions() {
//        return mockBallPositions;
//    }
//
//    int findBallIndex(Color c) {
//        // Among the 3 balls on the sorter plate, there can be more than one of the same color. findBall just returns the earliest one.
//        for (int i = 0; i < mockBallPositions.size(); i++) {
//            BallPosition bp = mockBallPositions.get(i);
//            if (bp.color == c) {
//                return i;
//            }
//        }
//
//        return -1;
//    }
//
//    int findBallIndex(int position, Color c) {
//        return mockBallPositions.indexOf(new BallPosition(position, c));
//    }
//}
