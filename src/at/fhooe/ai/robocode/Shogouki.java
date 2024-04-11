package at.fhooe.ai.robocode;

import robocode.AdvancedRobot;

public class Shogouki extends AdvancedRobot {

    @Override
    public void run() {
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        setTurnRadarRight(Double.POSITIVE_INFINITY);

        while (true) {
            ahead(100);
            turnGunLeft(360);
            back(100);
            turnGunRight(360);
        }
    }

    @Override
    public void onScannedRobot(robocode.ScannedRobotEvent e) {
        fire(1);
    }

    private void turnGunLeftWithRadar() {
        turnGunLeft(360);
        turnRadarLeft(360);
    }

    private void turnGunRightWithRadar() {
        turnGunRight(360);
        turnRadarRight(360);
    }
}
