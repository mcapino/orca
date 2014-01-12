package rvolib;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;

import java.util.ArrayList;


public class RVOTrajectory implements EvaluatedTrajectory {

    private ArrayList<Point> listOfPoints;
    private int cost;
    private float timeStep;
    private Point goal;

    public RVOTrajectory(ArrayList<Point> listOfPoints, float timeStep, Point goal) {
        this.goal = goal;
        this.listOfPoints = listOfPoints;
        this.timeStep = timeStep;
        determineCost();
    }

    private void determineCost() {
        int pointsOutsideGoal = 0;
//		Point goal = listOfPoints.get(listOfPoints.size() - 1);
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
        return listOfPoints.size() - 1;
    }

    @Override
    public Point get(int t) {
        if (t >= listOfPoints.size() || t < 0) {
            return null;
        }
        return listOfPoints.get(t);
    }

    public Point getLast() {
        return listOfPoints.get(listOfPoints.size() - 1);
    }

    public ArrayList<Point> getListOfPoints() {
        return listOfPoints;
    }

    public void addAll(RVOTrajectory evaluatedTrajectory) {
        listOfPoints.addAll(evaluatedTrajectory.getListOfPoints());
    }

    public float getTimeStep() {
        return timeStep;
    }

    public void setTimeStep(float timeStep) {
        this.timeStep = timeStep;
    }

}
