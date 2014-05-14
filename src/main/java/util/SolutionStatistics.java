package util;

import rvolib.RVOTrajectory;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.util.Util;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;

import java.util.Collection;
import java.util.Random;

public class SolutionStatistics {

    private EvaluatedTrajectory[] bestTrajectories;
    private EarliestArrivalProblem problem;
    private int iterations;
    private long runTime;
    private int sumMaxTime;
    private double agentMaxSpeed;
    private int singleAgentSumMaxTime;
    private Collection<Region> inflatedObstacles;
    private double quality;
    private String experimentID;
    private String algorithm;
    private int nAgents;
    private int timeOut;
    private double problemObstacleRatio;
    private int seed;
    private int agentRadius;
    private double suboptimalityThreshold;

    public SolutionStatistics(String experimentID, String algorithm,
                              int nAgents, int timeOut, double problemObstacleRatio, int seed,
                              int iterations, EvaluatedTrajectory[] bestTrajectories,
                              EarliestArrivalProblem problem, long startTime,
                              double agentMaxSpeed, double suboptimalityThreshold) {
        this.experimentID = experimentID;
        this.algorithm = algorithm;
        this.nAgents = nAgents;
        this.timeOut = timeOut;
        this.problemObstacleRatio = problemObstacleRatio;
        this.seed = seed;
        this.agentRadius = problem.getBodyRadius(0);

        this.bestTrajectories = bestTrajectories;
        this.problem = problem;
        this.iterations = iterations;
        this.agentMaxSpeed = agentMaxSpeed;
        this.runTime = ((System.nanoTime() - startTime) / 1000000);
        inflatedObstacles = Util.inflateRegions(problem.getObstacles(),
                problem.getBodyRadius(0));
        this.suboptimalityThreshold = suboptimalityThreshold;
        calculateSumMaxTime();
        calculateQuality();
    }

    public SolutionStatistics(String experimentID, String algorithm,
                              int nAgents, int timeOut, double problemObstacleRatio, int seed,
                              int iterations, EvaluatedTrajectory[] bestTrajectories,
                              EarliestArrivalProblem problem, long startTime,
                              double agentMaxSpeed, int sumMaxTime, double suboptimalityThreshold) {

        this.experimentID = experimentID;
        this.algorithm = algorithm;
        this.nAgents = nAgents;
        this.timeOut = timeOut;
        this.problemObstacleRatio = problemObstacleRatio;
        this.seed = seed;
        this.agentRadius = problem.getBodyRadius(0);

        this.bestTrajectories = bestTrajectories;
        this.problem = problem;
        this.iterations = iterations;
        this.agentMaxSpeed = agentMaxSpeed;
        this.runTime = ((System.nanoTime() - startTime) / 1000000);
        inflatedObstacles = Util.inflateRegions(problem.getObstacles(),
                problem.getBodyRadius(0));
        this.sumMaxTime = sumMaxTime;
        this.suboptimalityThreshold = suboptimalityThreshold;
        calculateQuality();

    }

    private void calculateSumMaxTime() {
        sumMaxTime = calculateSumMaxTime(bestTrajectories);
    }

    private void calculateQuality() {

        singleAgentSumMaxTime = OptimalSolutionProvider.getInstance()
                .getOptimalSolutionCost();
        quality = (double) sumMaxTime / singleAgentSumMaxTime;
    }

    public static int calculateSumMaxTime(EvaluatedTrajectory[] solution) {
        int sumMaxTime = 0;
        for (int i = 0; i < solution.length; i++) {
            int j = solution[i].getMaxTime();
            while (j >= 0
                    && solution[i].get(j).equals(
                    solution[i].get(solution[i].getMaxTime()))) {
                j--;
            }
            // System.out.println(j);
            RVOTrajectory tr = (RVOTrajectory) solution[i];
            sumMaxTime += solution[i].getCost();// (j * tr.getTimeStep());
        }
        return sumMaxTime;
    }

    public EvaluatedTrajectory[] getBestTrajectories() {
        return bestTrajectories;
    }

    public EarliestArrivalProblem getProblem() {
        return problem;
    }

    public int getIterations() {
        return iterations;
    }

    public long getRunTime() {
        return runTime;
    }

    public int getSumMaxTime() {
        return sumMaxTime;
    }

    public double getAgentMaxSpeed() {
        return agentMaxSpeed;
    }

    public int getSingleAgentSumMaxTime() {
        return singleAgentSumMaxTime;
    }

    public Collection<Region> getInflatedObstacles() {
        return inflatedObstacles;
    }

    public double getQuality() {
        return quality;
    }

    public void printSolution() {
        System.out.println(experimentID + ";" + algorithm + ";" + timeOut + ";"
                + problemObstacleRatio + ";" + seed + ";" + nAgents + ";"
                + agentRadius + ";" + runTime + ";" + iterations + ";"
                + quality + ";" + suboptimalityThreshold);
    }

    public static void printFinalStats(String experimentID, String algorithm,
                                       long timeOut, int nAgents, int agentRadius,
                                       double problemObstacleRatio, int seed, int iterations,
                                       double suboptimalityThreshold) {
        System.out.println(experimentID + ";" + algorithm + ";" + timeOut + ";"
                + problemObstacleRatio + ";" + seed + ";" + nAgents + ";"
                + agentRadius + ";" + "NA" + ";" + iterations + ";" + "NA"
                + ";" + suboptimalityThreshold);
    }

}
