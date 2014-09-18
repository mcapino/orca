package rvolib;

import java.util.Collection;

import org.jgrapht.DirectedGraph;

import tt.euclid2d.Vector;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.VisibilityGraph;
import tt.euclid2i.util.Util;
import tt.jointtraj.solver.SearchResult;
import util.DesiredControl;
import util.GraphBasedOptimalPolicyController;

public class RVOSolver {

	/**
	 * Somewhere near goal the agent must adjust the preferred velocity to avoid
	 * overshooting
	 **/
	private float adjustPrefferedVolcityNearGoalEpsilon = 0;
	/**
	 * The agent is considered at goal if the distance between its current
	 * position and the goal is within this tolerance
	 **/
	private float goalReachedToleranceEpsilon = 0; // 0.1f;

	private Point[] goals;
	private DesiredControl[] desiredControls;

	private Collection<Region> moreInflatedObstacles;
	private Collection<Region> lessInflatedObstacles;
	private int iteration;
	private float jointCost = 0;
	private boolean showProgress;

	Simulator simulator;
	private Point[] starts;
	private DirectedGraph<Point, Line> graph;
	private double bestGraphNodeSearchRadius;
	
	public RVOSolver(Point[] starts, Point[] goals, int bodyRadius,
			DirectedGraph<Point, Line> graph, 
			Collection<Region> obstacles,
			float timeStep, float neighborDist,
			int maxNeighbors, float deconflictionTimeHorizonAgents,
			float deconflictionTimeHorizonObstacles,
			float maxSpeed,
			boolean showProgress) {
		this(starts, goals, bodyRadius, graph, Double.MAX_VALUE, obstacles, timeStep, neighborDist, maxNeighbors, deconflictionTimeHorizonAgents, deconflictionTimeHorizonObstacles, maxSpeed, showProgress);
	}

	public RVOSolver(Point[] starts, Point[] goals, int bodyRadius,
			DirectedGraph<Point, Line> graph, 
			double bestGraphNodeSearchRadius, 
			Collection<Region> obstacles,
			float timeStep, float neighborDist,
			int maxNeighbors, float deconflictionTimeHorizonAgents,
			float deconflictionTimeHorizonObstacles,
			float maxSpeed,
			boolean showProgress) {

		this.showProgress = showProgress;
		this.simulator = new Simulator();
		this.simulator.setShowVis(showProgress);

		simulator.setTimeStep(timeStep);

		Vector2 initialVelocity = new Vector2();
		simulator.setAgentDefaults(neighborDist, maxNeighbors,
				deconflictionTimeHorizonAgents,
				deconflictionTimeHorizonObstacles, bodyRadius, maxSpeed,
				initialVelocity);

		simulator.addObstacles(obstacles);

		simulator.processObstacles();

		this.starts = starts;
		this.goals = goals;

		// Add agents
		for (int i = 0; i < starts.length; i++) {
			Vector2 start = new Vector2(starts[i]);
			simulator.addAgent(start);
			simulator.getAgent(i).goal_ = goals[i];
		}
		this.graph = graph;
		this.bestGraphNodeSearchRadius = bestGraphNodeSearchRadius;

		this.lessInflatedObstacles = Util.inflateRegions(obstacles, bodyRadius - 1);
		this.moreInflatedObstacles = Util.inflateRegions(obstacles, bodyRadius);
	}

	public SearchResult solve(int iterationLimit, long interruptAtNs, double maxJointCost) {

		tt.euclid2i.EvaluatedTrajectory[] trajs = new tt.euclid2i.EvaluatedTrajectory[simulator.getNumAgents()];

		if (graph != null) {
			createGraphControllers(graph);
		} else {
			createVisibilityGraphController();
		}

		for (int i = 0; i < simulator.getNumAgents(); i++) {
			simulator.getAgent(i).clearTrajectory();
		}

		iteration = 0;
		do {
			setPrefferedVelocities();

			simulator.doStep();

			// HARDCODED SIMULATION SPEED

			if (showProgress) {
				try {
					Thread.sleep((long) 10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			iteration++;

			calculateJointCost();

			if (System.nanoTime() > interruptAtNs) {
				return new SearchResult(null, false);
			}

		} while (!reachedGoal() && iteration < iterationLimit
				&& jointCost <= maxJointCost);

		for (int i = 0; i < trajs.length; i++) {
			trajs[i] = simulator.getAgent(i).getEvaluatedTrajectory(
					/*simulator.timeStep,*/
					goals[i]
					/*(int) Math.ceil(iterationLimit * simulator.timeStep)*/);
		}
		if (System.nanoTime() < interruptAtNs) {
			if (iteration < iterationLimit && jointCost <= maxJointCost) {
				// Success
				return new SearchResult(trajs, true);
			} else {
				// Fail
				return new SearchResult(null, true);
			}
		} else {
			return new SearchResult(null, false);
		}
 	}

	private void createGraphControllers(DirectedGraph<Point, Line> graph) {
		desiredControls = new DesiredControl[simulator.getNumAgents()];
        for (int i = 0; i < simulator.getNumAgents(); i++) {

            Point start = starts[i];
            Point end = goals[i];

            Util.addVertexAndConnectToNeighbors(graph, start, 4, lessInflatedObstacles);
            Util.addVertexAndConnectToNeighbors(graph, end, 4, lessInflatedObstacles);

        	GraphBasedOptimalPolicyController graphBasedDesiredControl
        		= new GraphBasedOptimalPolicyController(graph, goals[i], lessInflatedObstacles, 1.0, bestGraphNodeSearchRadius, showProgress);
        	desiredControls[i] = graphBasedDesiredControl;
        }
	}

	private void calculateJointCost() {
		for (int i = 0; i < simulator.getNumAgents(); ++i) {
			Point p = simulator.getAgentPosition(i).toPoint2i();
			Point g = goals[i];
			if (!p.equals(g)) {
				jointCost += simulator.getTimeStep();
			}
		}
	}

	private void createVisibilityGraphController() {
		desiredControls = new DesiredControl[simulator.getNumAgents()];
        for (int i = 0; i < simulator.getNumAgents(); i++) {
        	DirectedGraph<Point, Line> visGraph
        		= VisibilityGraph.createVisibilityGraph(starts[i], goals[i], lessInflatedObstacles, moreInflatedObstacles);
        	GraphBasedOptimalPolicyController visibilityGraphBasedDesiredControl
        		= new GraphBasedOptimalPolicyController(visGraph, goals[i], lessInflatedObstacles, 1.0, Double.MAX_VALUE, showProgress);
        	desiredControls[i] = visibilityGraphBasedDesiredControl;
        }
    }

	private void setPrefferedVelocities() {
		for (int i = 0; i < simulator.getNumAgents(); i++) {

			Vector2 currentPosition = simulator.getAgentPosition(i);
			double distanceToGoal = currentPosition.toPoint2i().distance(goals[i]);

			if (currentPosition.toPoint2i().distance(goals[i]) < adjustPrefferedVolcityNearGoalEpsilon) {
				simulator.setAgentPrefVelocity(i, new Vector2(0, 0));
			} else {
				Vector desiredVelocity = desiredControls[i].getDesiredControl(simulator.getAgent(i).position_.toPoint2d());
				double desiredSpeed = desiredVelocity.length();
				Vector desiredDir = new Vector(desiredVelocity);
				desiredDir.normalize();

				//System.out.println("Setting pref. velocity for agent " + i + " to " + desiredVelocity);

				// Adjust if the agent is near the goal
				if (distanceToGoal <= simulator.timeStep * desiredSpeed) {
					// goal will be reached in the next time step
					double speed = distanceToGoal / simulator.timeStep;
					desiredVelocity = desiredDir;
					desiredVelocity.scale(speed);
				}

				simulator.setAgentPrefVelocity(i, new Vector2(desiredVelocity));
			}
		}
	}

	private boolean reachedGoal() {
		/* Check if all agents have reached their goals. */
		for (int i = 0; i < simulator.getNumAgents(); ++i) {
			Point p = simulator.getAgentPosition(i).toPoint2i();
			Point g = goals[i];

			if (p.distance(g) > goalReachedToleranceEpsilon) {
				return false;
			}
		}

		return true;
	}

	public static enum outerLoopControl {
		LAZY_GRID, VISIBILITY_GRAPH
	}

	public int getIterations() {
		return iteration;
	}


}
