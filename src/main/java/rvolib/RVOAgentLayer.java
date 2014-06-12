package rvolib;

import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.element.StyledLine;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.StyledLineElements;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.StyledLineImpl;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.StyledLineLayer;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;
import tt.euclid2i.Point;
import tt.util.AgentColors;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

public class RVOAgentLayer {

    public final static int AGENT_FOR_GRID_VIS = -1;
    private static final boolean SHOW_ORCA_LINES = false;
    private static final boolean SHOW_CURRENT_TRAJECTORY = true;

    public static VisLayer create(final RVOAgent agent) {
        GroupLayer group = GroupLayer.create();

        if (agent.showVis) {
            // draw position info
            group.addSubLayer(new CommonLayer() {

                @Override
                public void paint(Graphics2D canvas) {
                    if (agent.showVis) {
                        super.paint(canvas);

                        canvas.setColor(AgentColors.getColorForAgent(agent.id_));
                        canvas.drawOval(
                                Vis.transX(agent.position_.x_ - agent.radius_),
                                Vis.transY(agent.position_.y_ - agent.radius_),
                                Vis.transH(agent.radius_ * 2),
                                Vis.transW(agent.radius_ * 2));
                    }
                }
            });

            // draw center position
            group.addSubLayer(StyledPointLayer
                    .create(new StyledPointElements() {

                        @Override
                        public Iterable<? extends StyledPoint> getPoints() {
                            ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

                            Color color = Color.BLACK;

                            Point3d position = new Point3d(agent.position_.x_,
                                    agent.position_.y_, 0);
                            if (agent.showVis) {
                                points.add(new StyledPointImpl(position, color,
                                        5));
                            }
                            return points;
                        }

                    }));

            // draw goal
            group.addSubLayer(StyledPointLayer
                    .create(new StyledPointElements() {

                        @Override
                        public Iterable<? extends StyledPoint> getPoints() {
                            ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

                            Color color = AgentColors
                                    .getColorForAgent(agent.id_);
                            if (agent.goal_ != null) {
                                Point3d position = new Point3d(agent.goal_.x,
                                        agent.goal_.y, 0);
                                if (agent.showVis) {
                                    points.add(new StyledPointImpl(position,
                                            color, 5));
                                }
                            }
                            return points;
                        }

                    }));

            // draw preferred velocity
            group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

                @Override
                public Iterable<? extends StyledLine> getLines() {
                    ArrayList<StyledLine> lines = new ArrayList<StyledLine>();

                    Color color = Color.BLUE;
                    if (agent.prefVelocity_ != null) {
                        Vector3d arrowTip = new Vector3d(
                                agent.prefVelocity_.x_, agent.prefVelocity_.y_,
                                0);
                        arrowTip.scale(agent.prefVelocity_.getLength());
                        Point3d position = new Point3d(agent.position_.x_,
                                agent.position_.y_, 0);
                        arrowTip.add(position);
                        if (agent.showVis) {
                            lines.add(new StyledLineImpl(position, new Point3d(
                                    arrowTip), color, 2));
                        }
                    }
                    return lines;
                }

            }));

            // draw velocity
            group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

                @Override
                public Iterable<? extends StyledLine> getLines() {
                    ArrayList<StyledLine> lines = new ArrayList<StyledLine>();

                    Color color = Color.RED;
                    if (agent.velocity_ != null) {
                        Vector3d arrowTip = new Vector3d(agent.velocity_.x_,
                                agent.velocity_.y_, 0);
                        arrowTip.scale(agent.velocity_.getLength());
                        Point3d position = new Point3d(agent.position_.x_,
                                agent.position_.y_, 0);
                        arrowTip.add(position);
                        if (agent.showVis) {
                            lines.add(new StyledLineImpl(position, new Point3d(
                                    arrowTip), color, 2));
                        }
                    }
                    return lines;
                }

            }));

            // draw ORCA lines
            if (SHOW_ORCA_LINES) {
                group.addSubLayer(StyledLineLayer
                        .create(new StyledLineElements() {

                            @Override
                            public Iterable<? extends StyledLine> getLines() {
                                ArrayList<StyledLine> lines = new ArrayList<StyledLine>();

                                Color color = AgentColors
                                        .getColorForAgent(agent.id_);
                                if (agent.orcaLines_ != null) {
                                    for (RVOLine orcaLine : agent.orcaLines_) {
                                        if (orcaLine != null
                                                && orcaLine.direction != null) {
                                            Vector3d arrowTip1 = new Vector3d(
                                                    orcaLine.direction.x_,
                                                    orcaLine.direction.y_, 0);
                                            arrowTip1.scale(1000);
                                            Vector3d arrowTip2 = new Vector3d(
                                                    arrowTip1);
                                            arrowTip2.negate();
                                            Point3d position = new Point3d(
                                                    orcaLine.point.x_
                                                            + agent.position_.x_,
                                                    orcaLine.point.y_
                                                            + agent.position_.y_,
                                                    0);
                                            arrowTip1.add(position);
                                            arrowTip2.add(position);
                                            lines.add(new StyledLineImpl(
                                                    position, new Point3d(
                                                    arrowTip1), color,
                                                    2));
                                            lines.add(new StyledLineImpl(
                                                    position, new Point3d(
                                                    arrowTip2), color,
                                                    2));
                                        }
                                    }
                                }
                                return lines;
                            }

                        }));
            }


            // draw current trajectory
            if (SHOW_CURRENT_TRAJECTORY) {
                group.addSubLayer(StyledPointLayer
                        .create(new StyledPointElements() {

                            @Override
                            public Iterable<? extends StyledPoint> getPoints() {
                                ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

                                Color color = AgentColors
                                        .getColorForAgent(agent.id_);
                                if (agent.timePointTrajectory != null) {
                                    for (tt.euclidtime3i.Point trajectoryPoint : agent.timePointTrajectory) {
                                        Point3d p = new Point3d(trajectoryPoint.x, trajectoryPoint.y, 0);
                                        points.add(new StyledPointImpl(p, color, 5));
                                    }

                                }

                                return points;
                            }


                        }));
            }
        }

//        // draw grid values
//        if (AGENT_FOR_GRID_VIS == agent.id_) {
//            group.addSubLayer(new CommonLayer() {
//
//                @Override
//                public void paint(Graphics2D canvas) {
//                    super.paint(canvas);
//
//                    canvas.setColor(Color.RED);
//                    // Color.getHSBColor((float) i / (float) n, 0.85f, 1.0f);
//
//                    HashMap<Point, Double> map = agent.getEvaluatedGraph();
//                    Iterator<Entry<Point, Double>> it = map.entrySet()
//                            .iterator();
//                    while (it.hasNext()) {
//                        Map.Entry pairs = (Map.Entry) it.next();
//                        Point position = (Point) pairs.getKey();
//
//                        String text = "" + pairs.getValue();
//                        canvas.drawString(text, (int) (Vis.transX(position.x)),
//                                (int) (Vis.transY(position.y)));
//                    }
//                }
//            });
//        }
        return group;
    }
}
