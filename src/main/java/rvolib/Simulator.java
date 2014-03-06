package rvolib;

import tt.euclid2i.Region;
import tt.euclid2i.region.Polygon;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;

public class Simulator {

	private ArrayList<RVOAgent> agents;
    public ArrayList<RVOObstacle> obstacles;
    private float time;
    public RVOAgent defaultAgent;
    private KdTree kdTree;
    public float timeStep;

    private Collection<Region> rectangleObstacles = new LinkedList<Region>();

	private boolean showVis = false;

    Simulator() {
        agents = new ArrayList<RVOAgent>();
        obstacles = new ArrayList<RVOObstacle>();
        time = 0;
        defaultAgent = null;
        kdTree = new KdTree();
        timeStep = .1f;
    }

    public float getGlobalTime() {
        return time;
    }

    public int getNumAgents() {
        return agents.size();
    }

    public float getTimeStep() {
        return timeStep;
    }

    public void setAgentPrefVelocity(int i, Vector2 velocity) {
        agents.get(i).setPrefVelocity(velocity);
    }

    public Vector2 getAgentPosition(int i) {
        return agents.get(i).position_;
    }

    public Vector2 getAgentPrefVelocity(int i) {
        return agents.get(i).prefVelocity_;
    }

    public Vector2 getAgentVelocity(int i) {
        return agents.get(i).velocity_;
    }

    public float getAgentRadius(int i) {
        return agents.get(i).radius_;
    }

    public ArrayList<RVOLine> getAgentOrcaLines(int i) {
        return agents.get(i).orcaLines_;
    }

    public int addAgent(Vector2 position) {
        if (defaultAgent == null) {
            return -1;
        }

        RVOAgent agent = new RVOAgent();

        agent.position_ = position;
        agent.maxNeighbors_ = defaultAgent.maxNeighbors_;
        agent.maxSpeed_ = defaultAgent.maxSpeed_;
        agent.neighborDist_ = defaultAgent.neighborDist_;
        agent.radius_ = defaultAgent.radius_;
        agent.timeHorizon_ = defaultAgent.timeHorizon_;
        agent.timeHorizonObst_ = defaultAgent.timeHorizonObst_;
        agent.velocity_ = defaultAgent.velocity_;

        agent.id_ = agents.size();

        if (showVis )
        	agent.initVisualization();

        agents.add(agent);

        return agents.size() - 1;
    }

    public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity) {
        if (defaultAgent == null) {
            defaultAgent = new RVOAgent();
        }

        defaultAgent.maxNeighbors_ = maxNeighbors;
        defaultAgent.maxSpeed_ = maxSpeed;
        defaultAgent.neighborDist_ = neighborDist; // probably the distance at which the neighboring agents are registered
        defaultAgent.radius_ = radius; //
        defaultAgent.timeHorizon_ = timeHorizon;
        defaultAgent.timeHorizonObst_ = timeHorizonObst;
        defaultAgent.velocity_ = velocity;
    }

    //TODO paralelize doStep
    public void doStep() {
        kdTree.buildAgentTree(agents.toArray(new RVOAgent[0]));

        for (int i = 0; i < (agents.size()); ++i) {
            agents.get(i).computeNeighbors(kdTree);
            agents.get(i).computeNewVelocity(timeStep);
        }

        for (int i = 0; i < (agents.size()); ++i) {
            agents.get(i).update(timeStep);
        }

        time += timeStep;
    }



    public void processObstacles() {
        kdTree.buildObstacleTree(obstacles.toArray(new RVOObstacle[0]), this.obstacles);
    }

    public boolean queryVisibility(Vector2 point1, Vector2 point2, float radius) {
        return kdTree.queryVisibility(point1, point2, radius);
    }

    public int getNumObstacleVertices() {
        return obstacles.size();
    }

    public Vector2 getObstacleVertex(int vertexNo) {
        return obstacles.get(vertexNo).point_;
    }

    public int getNextObstacleVertexNo(int vertexNo) {
        return obstacles.get(vertexNo).nextObstacle.id_;
    }

    public int getPrevObstacleVertexNo(int vertexNo) {
        return obstacles.get(vertexNo).prevObstacle.id_;
    }

    public RVOAgent getAgent(int i) {
        return agents.get(i);
    }

    public KdTree getKdTree() {
        return kdTree;
    }

    public void setTimeStep(float f) {
        this.timeStep = f;
    }

    public void setRectangleObstacles(
            LinkedList<Region> rectangleObstacles) {
        this.rectangleObstacles = rectangleObstacles;
    }

    public Collection<Region> getRectangleObstacles() {
        return rectangleObstacles;
    }

    public float getAgentMaxSpeed() {
        return defaultAgent.maxSpeed_;
    }

    public void setShowVis(boolean showVis) {
		this.showVis = showVis;
	}

	public void addObstacles(Collection<Region> obstacles2) {
		// Add obstacles
		for (Region region : obstacles2) {
			if (!(region instanceof Polygon)) {
				throw new RuntimeException("Only polygons are supported");
			}
			ArrayList<Vector2> obstacle = RVOUtil.regionToVectorList((Polygon) region);
			RVOUtil.addObstacle(obstacle, obstacles);
		}

	}

}
