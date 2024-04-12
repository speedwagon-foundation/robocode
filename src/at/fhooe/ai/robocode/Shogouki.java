package at.fhooe.ai.robocode;

import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;

import java.util.HashMap;

public class Shogouki extends AdvancedRobot {
    private static final double MIN_BULLET_POWER = 1.0;

    private final HashMap<String, EnemyInfo> detectedEnemies = new HashMap<>();

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
        updateDetectedEnemies(e);
        /**
         * ideas:
         *
         * don't fire when energy under 30%
         * don't move into corners, i.e. draw a circle from the center and use it as the border for movement
         * move away from opponent if spotted and under 600 ranger or something like that
         *
         * while no enemy spotted: turn radar in the same direction as the gun for maximum turn rate
         * once enemy is spotted: try to track enemy with radar and gun, i.e. lock radar to barrel direction and adjust barrel for bearing the enemy is moving to
         *
         *
         *
         * neutral/searching:
         * - turn radar in the same direction as the gun for maximum turn rate
         * - move in a circle around the center of the battlefield
         *
         * enemy spotted:
         *
         *  randomly stop sometimes
         *  randomly change movement direction
         *  prevent wall collisions
         *
         */


    }

    private void updateDetectedEnemies(ScannedRobotEvent e) {
        var time =  getTime();
        String name = e.getName();
        if(detectedEnemies.containsKey(name)) {
            EnemyInfo enemy = detectedEnemies.get(name);
            var oldState = enemy.copy();
            enemy.update(e, time);
            detectEventOnEnemy(oldState, enemy);
        } else {
            detectedEnemies.put(name, new EnemyInfo(e, time));
        }
    }

    private void detectEventOnEnemy(EnemyInfo oldState, EnemyInfo enemy) {
        var energyDiff = oldState.energy - enemy.energy;
        var velocityDiff = oldState.velocity - enemy.velocity;
        var wallHit = oldState.velocity > enemy.velocity && velocityDiff> 2;
        var enemyFiredShot = oldState.energy > enemy.energy
                && energyDiff >= 0.1 && energyDiff <= 3.0
                && !wallHit;

    }

    private double distanceToWallImpact() {

    }

    private double getBulletPower(ScannedRobotEvent e) {
        double distance = e.getDistance();
        double bulletPower = Math.min(3, e.getEnergy() / 4);
        if(distance > 600)
            bulletPower /= 2;


        return Math.max(bulletPower, MIN_BULLET_POWER);
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
