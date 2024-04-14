package speedwagon;

import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.HashMap;
import java.util.Random;

public class Shogouki extends AdvancedRobot {
    private static final double MIN_BULLET_POWER = 1.0;
    private static final int WALL_MARGIN = 60;
    private static final double SCAN_DIST = 36;
    private static final double BEST_DISTANCE = 525; // magic number - trial and error
    private static double blindCane = 130; // distance to wall to start wall smoothing
    private static double battlefieldWidth;
    private static double battlefieldHeight;
    private static double circleDir = 1;
    private static double lastReverseTime;
    private static double numBadHits = 0;
    private static boolean flat = true;
    private static EnemyInfo currentTarget;
    Rectangle2D.Double battlefield;

    private final HashMap<String, EnemyInfo> detectedEnemies = new HashMap<>();

    private static final double DECELERATION_COOLDOWN = 10; // 20 ticks cooldown for deceleration
    private static double remainingCooldown = 10;

    @Override
    public void run() {
        initColors();

        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        battlefieldWidth = getBattleFieldWidth();
        battlefieldHeight = getBattleFieldHeight();

        // get battlefield - bounds (18) - for some reason upper bounds use 18 * 2 buffer
        battlefield = new Rectangle2D.Double(18, 18, battlefieldWidth - 36, battlefieldHeight - 36);

        setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
        ahead(Double.MAX_VALUE);
        while (true) {
            if (getRadarTurnRemaining() == 0.0)
                setTurnRadarRightRadians(Double.POSITIVE_INFINITY);

            execute();
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
        updateDetectedEnemies(e);
        currentTarget = detectedEnemies.get(e.getName());
        performMovement(e);
        updateRadar(e);
        fireAtEnemy(e);

    }

    private void performMovement(ScannedRobotEvent e) {
        double theta;
        double enemyAbsoluteBearing = getHeadingRadians() + e.getBearingRadians();
        double enemyDistance = e.getDistance();

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
        double currentAhead = Math.cos(theta) * 100;
        stutterStep(currentAhead);
        setTurnRightRadians(Math.tan(theta));
    }

    private Random random = new Random();
    private void stutterStep(double currentAhead) {
        Stutter stutter = Stutter.ACCELERATE;
        // 30% chance to deccelerate on movement
        if(random.nextDouble() < 0.3 && remainingCooldown == 0) {
            stutter = Stutter.DECELERATE;

            // reset cooldown
            remainingCooldown = DECELERATION_COOLDOWN;
        }

        if(remainingCooldown > 0) {
            remainingCooldown--;
        }

        switch (stutter) {
            case ACCELERATE:
                setAhead(currentAhead);
                break;
            case DECELERATE:
                setBack(currentAhead / 2.0);
                break;
        }
    }

    private void updateRadar(ScannedRobotEvent e) {
        // Absolute angle towards target
        double angleToEnemy = getHeadingRadians() + e.getBearingRadians();

        // Subtract current radar heading to get the turn required to face the enemy, be sure it is normalized
        double radarTurn = Utils.normalRelativeAngle(angleToEnemy - getRadarHeadingRadians());

        // Distance we want to scan from middle of enemy to either side
        // The 36.0 is how many units from the center of the enemy robot it scans.
        double extraTurn = Math.min(Math.atan(SCAN_DIST / e.getDistance()), Rules.RADAR_TURN_RATE_RADIANS);

        // Adjust the radar turn so it goes that much further in the direction it is going to turn
        // Basically if we were going to turn it left, turn it even more left, if right, turn more right.
        // This allows us to overshoot our enemy so that we get a good sweep that will not slip.
        if (radarTurn < 0)
            radarTurn -= extraTurn;
        else
            radarTurn += extraTurn;

        //Turn the radar
        setTurnRadarRightRadians(radarTurn);
    }

    private void fireAtEnemy(ScannedRobotEvent event) {
        final double FIREPOWER = 2;
        final double ROBOT_WIDTH = 16, ROBOT_HEIGHT = 16;
        // Variables prefixed with e- refer to enemy, b- refer to bullet and r- refer to robot
        final double eAbsBearing = getHeadingRadians() + event.getBearingRadians();
        final double rX = getX(), rY = getY(),
                bV = Rules.getBulletSpeed(FIREPOWER);
        final double eX = rX + event.getDistance() * Math.sin(eAbsBearing),
                eY = rY + event.getDistance() * Math.cos(eAbsBearing),
                eV = event.getVelocity(),
                eHd = event.getHeadingRadians();
        // These constants make calculating the quadratic coefficients below easier
        final double A = (eX - rX) / bV;
        final double B = eV / bV * Math.sin(eHd);
        final double C = (eY - rY) / bV;
        final double D = eV / bV * Math.cos(eHd);
        // Quadratic coefficients: a*(1/t)^2 + b*(1/t) + c = 0
        final double a = A * A + C * C;
        final double b = 2 * (A * B + C * D);
        final double c = (B * B + D * D - 1);
        final double discrim = b * b - 4 * a * c;
        if (discrim >= 0) {
            // Reciprocal of quadratic formula
            final double t1 = 2 * a / (-b - Math.sqrt(discrim));
            final double t2 = 2 * a / (-b + Math.sqrt(discrim));
            final double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);
            // Assume enemy stops at walls
            final double endX = limit(
                    eX + eV * t * Math.sin(eHd),
                    ROBOT_WIDTH / 2, getBattleFieldWidth() - ROBOT_WIDTH / 2);
            final double endY = limit(
                    eY + eV * t * Math.cos(eHd),
                    ROBOT_HEIGHT / 2, getBattleFieldHeight() - ROBOT_HEIGHT / 2);
            setTurnGunRightRadians(robocode.util.Utils.normalRelativeAngle(
                    Math.atan2(endX - rX, endY - rY)
                            - getGunHeadingRadians()));
            setFire(FIREPOWER);
        }
    }

    private double limit(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
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

    @Override
    public void onBulletHit(BulletHitEvent event) {
        if (detectedEnemies.containsKey(event.getName())) {
            var enemy = detectedEnemies.get(event.getName());
            enemy.energy = event.getEnergy();
        }
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

    private static Point2D.Double projectMotion(double x, double y, double heading, double distance) {
        return new Point2D.Double(x + distance * Math.sin(heading), y + distance * Math.cos(heading));
    }

    private static double absoluteBearing(double x, double y, Point2D.Double target) {
        return Math.atan2(target.x - x, target.y - y);
    }

    private void initColors() {
        setBodyColor(new Color(118, 88, 152));
        setGunColor(new Color(82, 208, 83));
        setRadarColor(new Color(252, 119, 1));
        setBulletColor(new Color(0, 0, 0));
        setScanColor(new Color(211, 41, 15));
    }

    private enum Stutter {
        ACCELERATE,
        DECELERATE
    }
}
