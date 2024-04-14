package speedwagon;

import robocode.ScannedRobotEvent;

public class EnemyInfo {
    public String name;
    public double energy;
    public int posX;
    public int posY;
    public long lastSeen;
    public double velocity;
    public double heading;
    public double bearing;
    public double distance;

    public EnemyInfo(String name, double energy, int posX, int posY, long lastSeen, double velocity, double heading, double bearing, double distance) {
        this.name = name;
        this.energy = energy;
        this.posX = posX;
        this.posY = posY;
        this.lastSeen = lastSeen;
        this.velocity = velocity;
        this.heading = heading;
        this.bearing = bearing;
    }
    public EnemyInfo(ScannedRobotEvent e,long currentTime) {
        this.name = e.getName();
        this.energy = e.getEnergy();
        this.posX = (int) Math.round(Math.cos(e.getBearingRadians()) * e.getDistance());
        this.posY = (int) Math.round(Math.sin(e.getBearingRadians()) * e.getDistance());
        this.lastSeen = currentTime;
        this.velocity =  e.getVelocity();
        this.heading = e.getHeading();
        this.bearing = e.getBearing();
        this.distance = e.getDistance();
    }

    public void update(ScannedRobotEvent e, long currentTime) {
        this.energy = e.getEnergy();
        this.posX = (int) Math.round(Math.cos(e.getBearingRadians()) * e.getDistance());
        this.posY = (int) Math.round(Math.sin(e.getBearingRadians()) * e.getDistance());
        this.lastSeen = currentTime;
        this.velocity =  e.getVelocity();
        this.heading = e.getHeading();
        this.bearing = e.getBearing();
        this.distance = e.getDistance();
    }

    public EnemyInfo copy() {
        return new EnemyInfo(name, energy, posX, posY, lastSeen, velocity, heading, bearing, distance);
    }
}
