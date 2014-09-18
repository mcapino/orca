package util;

import java.awt.Color;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map.Entry;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.alg.DijkstraShortestPaths;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;

import rvolib.VisibilityGraphLayer;
import tt.euclid2d.Vector;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.region.Circle;
import tt.euclid2i.util.Util;
import tt.euclid2i.vis.PointLayer.PointProvider;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import cz.agents.alite.vis.VisManager;

public class GraphBasedOptimalPolicyController implements DesiredControl{
	
	static Logger LOGGER = Logger.getLogger(GraphBasedOptimalPolicyController.class);

	protected Point goal;
	protected DirectedGraph<Point, Line>  graph;
	protected HashMap<Point, Double> nodeValues;
	protected Collection<Region> obstacles;
	protected double vmax;
	protected double bestNodeSearchRadius;
	
	protected tt.euclid2d.Point lastQueryPosition = new tt.euclid2d.Point(0,0);
	protected double lastQueryRadius;
	protected Collection<Point> lastQueryNodes = Collections.synchronizedList(new LinkedList<Point>());
	protected Point lastBestNode;

	public GraphBasedOptimalPolicyController(DirectedGraph<Point, Line> graph,
			Point goal, Collection<Region> obstacles,
			double vmax, double bestNodeSearchRadius, boolean showVis) {
        this.graph = graph;
        this.vmax = vmax;
        this.goal = goal;
        this.nodeValues = evaluateGraph(goal);
        this.obstacles = obstacles;
        this.bestNodeSearchRadius = bestNodeSearchRadius;

        if (showVis) {
            VisManager.registerLayer(VisibilityGraphLayer.create(this));
        }

        VisManager.registerLayer(tt.euclid2i.vis.PointLayer.create(new PointProvider() {
        	
        	@Override
        	public Collection<Point> getPoints() {
        		return new LinkedList<Point>(lastQueryNodes);
        	}
        },Color.RED, 2));
        
        VisManager.registerLayer(tt.euclid2i.vis.PointLayer.create(new PointProvider() {
        	
        	@Override
        	public Collection<Point> getPoints() {
        		return Collections.singleton(lastBestNode);
        	}
        },Color.GREEN, 4));
        
        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {
			
			@Override
			public Collection<? extends Region> getRegions() {
				assert lastQueryPosition != null;
				return Collections.singleton(new Circle(new Point((int)lastQueryPosition.x, (int)lastQueryPosition.y), (int) lastQueryRadius));
			}
		}, Color.RED));
    }

    public HashMap<Point, Double> evaluateGraph(Point goal) {
        DijkstraShortestPaths<Point, Line> dijkstra = new DijkstraShortestPaths<Point, Line>(
                graph, goal, new Goal<Point>() {
            @Override
            public boolean isGoal(Point current) {
                return false;
            }
        }, Double.MAX_VALUE);

        HashMap<Point, Double> lengths = dijkstra.findLengths(100000000000l);
        return lengths;
    }

    public GraphPath<Point, Line> getShortestPath(Point start, final Point goal) {
        AStarShortestPathSimple<Point, Line> astarSimple = new AStarShortestPathSimple<Point, Line>(
                graph, new HeuristicToGoal<Point>() {

            @Override
            public double getCostToGoalEstimate(Point current) {
                double estimate = current.distance(goal);
                return estimate;
            }
        }, start, new Goal<Point>() {
            @Override
            public boolean isGoal(Point current) {
                return current.equals(goal);
            }
        }
        );

        GraphPath<Point, Line> path = astarSimple.findPath(100000000);
        return path;
    }

    public DirectedGraph<Point, Line>  getGraph() {
        return graph;
    }
    
    @Override
    public tt.euclid2d.Vector getDesiredControl(tt.euclid2d.Point currentPosition) {
    	return getDesiredControl(currentPosition, bestNodeSearchRadius);
    }

	public tt.euclid2d.Vector getDesiredControl(tt.euclid2d.Point currentPosition, double bestNodeSearchRadius) {
		
		assert !Double.isNaN(currentPosition.x) && !Double.isNaN(currentPosition.y);
		LOGGER.trace("Finding best direction from node: " + currentPosition);
		lastQueryNodes.clear();
		lastQueryPosition = currentPosition;
		lastQueryRadius = bestNodeSearchRadius;
		
		double minTotalDistanceToGoal = Double.MAX_VALUE;
		tt.euclid2d.Vector bestDirection = null;
		Point bestNode = null;
		
		for (Entry<Point, Double> pointValuePair : nodeValues.entrySet()) {
			Point node = pointValuePair.getKey();
			double value = pointValuePair.getValue();

			Vector vector = new Vector(node.x - currentPosition.x, node.y - currentPosition.y);

			double distToCurrentPosition = vector.length();
			double sumTotalDistToGoal = distToCurrentPosition + value;
			Point currentPos2i = new Point((int)currentPosition.x, (int) currentPosition.y);
			

			if (currentPosition.distance(node.toPoint2d()) > 0.1 &&
				currentPosition.distance(node.toPoint2d()) <= bestNodeSearchRadius
				) {
				lastQueryNodes.add(node);
				
				if (sumTotalDistToGoal < minTotalDistanceToGoal && Util.isVisible(currentPos2i, node, obstacles)) {
					LOGGER.trace("  -- Considering node " + node + " with value=" + value + ". Total-value=" + sumTotalDistToGoal);
					bestDirection = vector;
					bestNode = node;
					minTotalDistanceToGoal = sumTotalDistToGoal;
				}
				
			}
			
			lastBestNode = bestNode;
		}

		if (bestDirection != null && !(bestDirection.x == 0 && bestDirection.y == 0)) {
			LOGGER.trace("Best node: " + bestNode);
			bestDirection.normalize();
			bestDirection.scale(vmax);
			
			if (Double.isNaN(bestDirection.x)) {
				LOGGER.error("NaN");
			}
			
			return bestDirection;
		} else {
			LOGGER.warn("Cannot find path to goal from " + currentPosition + ". bestDirection=" + bestDirection + ". Using straight-line direction to goal");
			tt.euclid2d.Vector vector = new tt.euclid2d.Vector(goal.x - currentPosition.x, goal.y - currentPosition.y);
			vector.normalize();
			vector.scale(vmax);
			return vector;
		}
	}

}
