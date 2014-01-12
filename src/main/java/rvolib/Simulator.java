package rvolib;

import tt.euclid2i.Region;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;

public class Simulator {

	private ArrayList<RVOAgent> agents_;
    public ArrayList<RVOObstacle> obstacles_;
    private float time_;
    public RVOAgent defaultAgent_;
    private KdTree kdTree_;
    public float timeStep_;

    private Collection<Region> rectangleObstacles = new LinkedList<Region>();

	private boolean showVis = false;

    Simulator() {
        agents_ = new ArrayList<RVOAgent>();
        obstacles_ = new ArrayList<RVOObstacle>();
        time_ = 0;
        defaultAgent_ = null;
        kdTree_ = new KdTree();
        timeStep_ = .1f;
    }

    public float getGlobalTime() {
        return time_;
    }

    public int getNumAgents() {
        return agents_.size();
    }

    public float getTimeStep() {
        return timeStep_;
    }

    public void setAgentPrefVelocity(int i, Vector2 velocity) {
        agents_.get(i).setPrefVelocity(velocity);
    }

    public Vector2 getAgentPosition(int i) {
        return agents_.get(i).position_;
    }

    public Vector2 getAgentPrefVelocity(int i) {
        return agents_.get(i).prefVelocity_;
    }

    public Vector2 getAgentVelocity(int i) {
        return agents_.get(i).velocity_;
    }

    public float getAgentRadius(int i) {
        return agents_.get(i).radius_;
    }

    public ArrayList<RVOLine> getAgentOrcaLines(int i) {
        return agents_.get(i).orcaLines_;
    }

    public int addAgent(Vector2 position) {
        if (defaultAgent_ == null) {
            return -1;
        }

        RVOAgent agent = new RVOAgent();

        agent.position_ = position;
        agent.maxNeighbors_ = defaultAgent_.maxNeighbors_;
        agent.maxSpeed_ = defaultAgent_.maxSpeed_;
        agent.neighborDist_ = defaultAgent_.neighborDist_;
        agent.radius_ = defaultAgent_.radius_;
        agent.timeHorizon_ = defaultAgent_.timeHorizon_;
        agent.timeHorizonObst_ = defaultAgent_.timeHorizonObst_;
        agent.velocity_ = defaultAgent_.velocity_;

        agent.id_ = agents_.size();
        if (showVis )
        	agent.initVisualization();
        agents_.add(agent);

        return agents_.size() - 1;

    }

    public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity) {
        if (defaultAgent_ == null) {
            defaultAgent_ = new RVOAgent();
        }

        defaultAgent_.maxNeighbors_ = maxNeighbors;
        defaultAgent_.maxSpeed_ = maxSpeed;
        defaultAgent_.neighborDist_ = neighborDist; // probably the distance at which the neighboring agents are registered
        defaultAgent_.radius_ = radius; //
        defaultAgent_.timeHorizon_ = timeHorizon;
        defaultAgent_.timeHorizonObst_ = timeHorizonObst;
        defaultAgent_.velocity_ = velocity;
    }

    //TODO paralelize doStep
    public void doStep() {
        kdTree_.buildAgentTree(agents_.toArray(new RVOAgent[0]));

        for (int i = 0; i < (agents_.size()); ++i) {
            agents_.get(i).computeNeighbors(kdTree_);
            agents_.get(i).computeNewVelocity(timeStep_);
        }

        for (int i = 0; i < (agents_.size()); ++i) {
            agents_.get(i).update(timeStep_);
        }

        time_ += timeStep_;
    }

    /*
     * vertices MUST BE ADDED IN CLOCKWISE ORDER
     */
    public int addObstacle(ArrayList<Vector2> vertices) {
        if (vertices.size() < 2) {
            return -1;
        }

        int obstacleNo = obstacles_.size();

        for (int i = 0; i < vertices.size(); ++i) {
            RVOObstacle obstacle = new RVOObstacle();
            obstacle.point_ = vertices.get(i);
            if (i != 0) {
                obstacle.prevObstacle = obstacles_.get(obstacles_.size() - 1);
                obstacle.prevObstacle.nextObstacle = obstacle;
            }
            if (i == vertices.size() - 1) {
                obstacle.nextObstacle = obstacles_.get(obstacleNo);
                obstacle.nextObstacle.prevObstacle = obstacle;
            }
            obstacle.unitDir_ = RVOMath.normalize(Vector2.minus(vertices.get(i == vertices.size() - 1 ? 0 : i + 1), vertices.get(i)));

            if (vertices.size() == 2) {
                obstacle.isConvex_ = true;
            } else {
                obstacle.isConvex_ = (RVOMath.leftOf(vertices.get(i == 0 ? vertices.size() - 1 : i - 1), vertices.get(i), vertices.get(i == vertices.size() - 1 ? 0 : i + 1)) >= 0);
            }

            obstacle.id_ = obstacles_.size();

            obstacles_.add(obstacle);
        }

        return obstacleNo;
    }

    public void processObstacles() {
        kdTree_.buildObstacleTree(obstacles_.toArray(new RVOObstacle[0]), this);
    }

    public boolean queryVisibility(Vector2 point1, Vector2 point2, float radius) {
        return kdTree_.queryVisibility(point1, point2, radius);
    }

    public int getNumObstacleVertices() {
        return obstacles_.size();
    }

    public Vector2 getObstacleVertex(int vertexNo) {
        return obstacles_.get(vertexNo).point_;
    }

    public int getNextObstacleVertexNo(int vertexNo) {
        return obstacles_.get(vertexNo).nextObstacle.id_;
    }

    public int getPrevObstacleVertexNo(int vertexNo) {
        return obstacles_.get(vertexNo).prevObstacle.id_;
    }

    public RVOAgent getAgent(int i) {
        return agents_.get(i);
    }

    public KdTree getKdTree() {
        return kdTree_;
    }

    public void setTimeStep(float f) {
        this.timeStep_ = f;
    }

    public void setRectangleObstacles(
            LinkedList<Region> rectangleObstacles) {
        this.rectangleObstacles = rectangleObstacles;
    }

    public Collection<Region> getRectangleObstacles() {
        return rectangleObstacles;
    }

    public float getAgentMaxSpeed() {
        return defaultAgent_.maxSpeed_;
    }


}
