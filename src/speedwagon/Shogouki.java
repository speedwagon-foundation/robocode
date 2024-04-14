package speedwagon;

import org.w3c.dom.css.Rect;
import robocode.AdvancedRobot;
import robocode.HitByBulletEvent;
import robocode.ScannedRobotEvent;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.HashMap;

public class Shogouki extends AdvancedRobot {
    private static final double MIN_BULLET_POWER = 1.0;
    private static final int WALL_MARGIN = 60;
    private static final double BEST_DISTANCE = 525; // magic number - trial and error
    private static double blindCane = 130; // distance to wall to start wall smoothing
    private static double battlefieldWidth;
    private static double battlefieldHeight;
    private static double circleDir = 1;
    private static double lastReverseTime;
    private static double numBadHits = 0;
    private static boolean flat = true;
    private static EnemyInfo currentTarget;

    private final HashMap<String, EnemyInfo> detectedEnemies = new HashMap<>();

    @Override
    public void run() {
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        battlefieldWidth = getBattleFieldWidth();
        battlefieldHeight = getBattleFieldHeight();

        ahead(Double.MAX_VALUE);
        while (true) {
            turnRadarRightRadians(Double.POSITIVE_INFINITY);
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
        updateDetectedEnemies(e);
        currentTarget = detectedEnemies.get(e.getName());

        double theta;
        double enemyAbsoluteBearing = getHeadingRadians() + e.getBearingRadians();
        double enemyDistance = e.getDistance();

        // get battlefield - bounds (18) - for some reason upper bounds use 18 * 2 buffer
        Rectangle2D.Double battlefield = new Rectangle2D.Double(18, 18, battlefieldWidth - 36, battlefieldHeight - 36);

        // movement

        Point2D.Double newDestination;
        double distDelta = 0.02 + Math.PI / 2 + (enemyDistance > BEST_DISTANCE ? -.1 : .5);

        while (!battlefield.contains(newDestination = projectMotion(getX(), getY(), enemyAbsoluteBearing + circleDir * (distDelta -= 0.02), 170)));

        theta = 0.5952 * (20D - 3D * currentTarget.energy) / enemyDistance;

        // randomly reverse direction after some time
        if ((flat && Math.random() > Math.pow(theta, theta)) || distDelta < Math.PI / 5 || (distDelta < Math.PI / 3.5 && enemyDistance < 400)) {
            circleDir = -circleDir;
            lastReverseTime = getTime();
        }

        theta = absoluteBearing(getX(), getY(), newDestination) - getHeadingRadians();
        setAhead(Math.cos(theta) * 100);
        setTurnRightRadians(Math.tan(theta));
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

    @Override
    public void onHitByBullet(HitByBulletEvent e) {
        // musashi trick
        if ((double)(getTime() - lastReverseTime) > currentTarget.distance/e.getVelocity() && currentTarget.distance > 200 && !flat)
            flat = (++numBadHits/(getRoundNum()+1) > 1.1);
    }

    private void updateDetectedEnemies(ScannedRobotEvent e) {
        var time = getTime();
        String name = e.getName();
        if (detectedEnemies.containsKey(name)) {
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
        var wallHit = oldState.velocity > enemy.velocity && velocityDiff > 2;
        var enemyFiredShot = oldState.energy > enemy.energy
                && energyDiff >= 0.1 && energyDiff <= 3.0 && !wallHit;


    }

    private double getBulletPower(ScannedRobotEvent e) {
        double distance = e.getDistance();
        double bulletPower = Math.min(3, e.getEnergy() / 4);
        if (distance > 600) bulletPower /= 2;

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

    private void resetCane() {
        blindCane = 130;
    }

    /**
     * x/y = current coordinates
     * startAngle = absolute angle that tank starts off moving - this is the angle
     * they will be moving at if there is no wall smoothing taking place.
     * orientation = 1 if orbiting enemy clockwise, -1 if orbiting counter-clockwise
     * NOTE: this method is designed based on an orbital movement system; these
     * last 2 arguments could be simplified in any other movement system.
     */
    public double wallSmoothing(double x, double y, double startAngle, int orientation) {
        resetCane();
        double wallDistanceX = Math.min(x - WALL_MARGIN, battlefieldWidth - x - WALL_MARGIN);
        double wallDistanceY = Math.min(y - WALL_MARGIN, battlefieldHeight - y - WALL_MARGIN);

        if (wallDistanceX > blindCane && wallDistanceY > blindCane) {
            return startAngle;
        }

        double angle = startAngle;
        double testX = x + (Math.sin(angle) * blindCane);
        double testY = y + (Math.cos(angle) * blindCane);
        double testDistanceX = Math.min(testX - WALL_MARGIN, battlefieldWidth - testX - WALL_MARGIN);
        double testDistanceY = Math.min(testY - WALL_MARGIN, battlefieldHeight - testY - WALL_MARGIN);

        double adjacent = 0;
        int g = 0; // shouldn't be needed, but infinite loop sanity check to prevent infinite loop

        while ((testDistanceX < 0 || testDistanceY < 0) && g++ < 25) {
            if (testDistanceY < 0 && testDistanceY < testDistanceX) {
                // North or South wall
                angle = (testY < WALL_MARGIN) ? Math.PI : 0;
                adjacent = wallDistanceY;
            } else if (testDistanceX < 0 && testDistanceX <= testDistanceY) {
                // East or West wall
                angle = (testX < WALL_MARGIN) ? (3 * (Math.PI / 2.0)) : (Math.PI / 2.0);
                adjacent = wallDistanceX;
            }

            if (adjacent < 0) {
                if (-adjacent > blindCane) {
                    blindCane += -adjacent;
                }
                angle += Math.PI - orientation * (Math.abs(Math.acos(-adjacent / blindCane)) - 0.0005);
            } else {
                angle += orientation * (Math.abs(Math.acos(adjacent / blindCane)) + 0.0005);
            }
            testX = x + (Math.sin(angle) * blindCane);
            testY = y + (Math.cos(angle) * blindCane);
            testDistanceX = Math.min(testX - WALL_MARGIN, battlefieldWidth - testX - WALL_MARGIN);
            testDistanceY = Math.min(testY - WALL_MARGIN, battlefieldHeight - testY - WALL_MARGIN);
        }
        resetCane();
        return normalizeBearing(angle);
    }

    private static Point2D.Double projectMotion(double x, double y, double heading, double distance) {
        return new Point2D.Double(x + distance * Math.sin(heading), y + distance * Math.cos(heading));
    }

    private static double absoluteBearing(double x, double y, Point2D.Double target) {
        return Math.atan2(target.x - x, target.y - y);
    }

    // Normalizes a bearing to between +pi and -pi
    private double normalizeBearing(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
