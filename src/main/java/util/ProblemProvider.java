package util;

import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;

public class ProblemProvider {

    private static ProblemProvider instance = new ProblemProvider();
    private EarliestArrivalProblem problem;

//    public static ProblemProvider getInstance() {
//        return instance;
//    }

    public void init(EarliestArrivalProblem problem) {

        this.problem = problem;
    }

    public EarliestArrivalProblem getProblem() {
        if (problem == null) {
            throw new RuntimeException("ProblemProvider not initialized");
        }
        return problem;
    }

}
