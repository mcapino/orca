package rvolib;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;

import java.util.ArrayList;


public class RVOTrajectory implements EvaluatedTrajectory {

    private ArrayList<Point> listOfPoints;
    private int cost;
    private float timeStep;
    private Point goal;
	private int maxTime;

    public RVOTrajectory(ArrayList<Point> listOfPoints, float timeStep, Point goal, int maxTime) {
        this.goal = goal;
        this.listOfPoints = listOfPoints;
        this.timeStep = timeStep;
        this.maxTime = maxTime;
        determineCost();
    }

    private void determineCost() {
        int pointsOutsideGoal = 0;
        for (int i = 0; i < listOfPoints.size(); i++) {
            if (!listOfPoints.get(i).equals(goal)) {
                pointsOutsideGoal++;
            }
        }
        this.cost = (int) (pointsOutsideGoal * timeStep);
    }

    public RVOTrajectory() {
        listOfPoints = new ArrayList<Point>();
    }

    @Override
    public double getCost() {
        return cost;
    }

    @Override
    public int getMinTime() {
        return 0;
    }

    @Override
    public int getMaxTime() {
        return maxTime;
    }

    @Override
    public Point get(int t) {
        if ( t >= 0 && t <= maxTime) {
        	int i = (int) ((float)t / timeStep);
        	if (i <= listOfPoints.size()-1) {
        		return listOfPoints.get(i);
        	} else {
        		return getLast();
        	}
        } else {
        	return null;
        }

    }

    public Point getLast() {
        return listOfPoints.get(listOfPoints.size() - 1);
    }

    public ArrayList<Point> getListOfPoints() {
        return listOfPoints;
    }

    public float getTimeStep() {
        return timeStep;
    }

}
