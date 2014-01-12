package rvolib;

import cz.agents.alite.vis.element.StyledLine;
import cz.agents.alite.vis.element.aggregation.StyledLineElements;
import cz.agents.alite.vis.element.implemetation.StyledLineImpl;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.StyledLineLayer;

import org.jgrapht.WeightedGraph;
import org.jgrapht.alg.VisibilityGraphPlanner;
import org.jgrapht.alg.WeightedLine;
import org.jgrapht.graph.SimpleGraph;

import tt.euclid2i.Line;
import tt.euclid2i.Point;

import javax.vecmath.Point3d;
import java.awt.*;
import java.util.ArrayList;
import java.util.Set;

public class VisibilityGraphLayer {

    public final static int AGENT_FOR_GRID_VIS = -1;

    public static VisLayer create(final VisibilityGraphPlanner planner) {
        GroupLayer group = GroupLayer.create();


/*
		group.addSubLayer(StyledPointLayer.create(new StyledPointElements() {

			@Override
			public Iterable<? extends StyledPoint> getPoints() {
				ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

				Color color = Color.BLUE;

				Point3d position = new Point3d(agent.position_.x_,
						agent.position_.y_, 0);
				points.add(new StyledPointImpl(position, color, 15));
				return points;
			}

		}));*/

        group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

            @Override
            public Iterable<? extends StyledLine> getLines() {
                ArrayList<StyledLine> lines = new ArrayList<StyledLine>();

                Color color = Color.LIGHT_GRAY;

                WeightedGraph<Point, Line> graph = planner.getGraph();
                if (graph != null) {
	                Set<Line> edges = graph.edgeSet();
	                for (Line line : edges) {
	                    Point start = graph.getEdgeSource(line);
	                    Point3d start3d = new Point3d(start.x, start.y, 0);
	                    Point end = graph.getEdgeTarget(line);
	                    Point3d end3d = new Point3d(end.x, end.y, 0);
	                    lines.add(new StyledLineImpl(start3d, end3d, color, 1));
	                }
                }

                return lines;
            }

        }));


        return group;
    }
}
