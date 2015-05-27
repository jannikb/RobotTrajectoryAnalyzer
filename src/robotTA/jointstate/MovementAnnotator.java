package robotTA.jointstate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.descriptor.ConfigurationParameter;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.cas.DoubleArray;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.JointState;
import robotTA.types.Movement;

public class MovementAnnotator extends JCasAnnotator_ImplBase {
    
    public static final String MINVARIANCE = "minVariance";
    @ConfigurationParameter(name = MINVARIANCE)
    private double minVariance;
    
    public static final String OBSERVEDJOINTSTATES = "observedJointStates";
    @ConfigurationParameter(name = OBSERVEDJOINTSTATES)
    private int observedJointStates;

    //private double minVariance = 0.000003;
    //private int observedJointStates = 10;

    @Override
    public void process(JCas aJCas) throws AnalysisEngineProcessException {
        FSIndex<Annotation> jsIndex = aJCas.getAnnotationIndex(JointState.type);
        Iterator<Annotation> jsIter = jsIndex.iterator();
        JointState js;
        ArrayList<DoubleArray> prevPositions = new ArrayList<DoubleArray>();
        ArrayList<Movement> moves = null;
        ArrayList<Movement> prevMoves = null;
        ArrayList<Double> variances = null;

        while (jsIter.hasNext()) {
            js = (JointState) jsIter.next();

            if (moves == null) {
                moves = new ArrayList<Movement>(Collections.nCopies(js
                        .getName().size(), null));
                prevMoves = new ArrayList<Movement>(Collections.nCopies(js
                        .getName().size(), null));
            }

            prevPositions.add(js.getJointTrajectoryPoint().getPositions());

            if (prevPositions.size() > observedJointStates)
                prevPositions.remove(0);

            if (prevPositions.size() < observedJointStates)
                continue;

            variances = calcVariances(prevPositions);

            for (int i = 0; i < variances.size(); i++) {
                if (moves.get(i) != null && variances.get(i) >= minVariance) {
                    moves.get(i).setEnd(js.getSeq());
                } else if (moves.get(i) == null
                        && variances.get(i) >= minVariance) {
                    if (prevMoves.get(i) != null
                            && prevMoves.get(i).getEnd() >= (js.getSeq() - 2 * observedJointStates)) {
                        moves.set(i, prevMoves.get(i));
                        moves.get(i).setEnd(js.getSeq());
                    } else {
                        if (prevMoves.get(i) != null) {
                            prevMoves.get(i).addToIndexes();
                            prevMoves.set(i, null);
                        }
                        moves.set(i, createAnnotation(aJCas, js, i));
                    }
                } else if (moves.get(i) != null
                        && variances.get(i) < minVariance) {
                    prevMoves.set(i, moves.get(i));
                    moves.set(i, null);
                }
            }
        }

        if (moves != null)
            for (Movement move : moves) {
                if (move != null)
                    move.addToIndexes();
            }

        if (prevMoves != null)
            for (Movement move : prevMoves) {
                if (move != null)
                    move.addToIndexes();
            }
    }

    private ArrayList<Double> calcVariances(ArrayList<DoubleArray> prevPositions) {
        ArrayList<Double> variances = new ArrayList<Double>(
                Collections.nCopies(prevPositions.get(0).size(), 0.0));
        ArrayList<Double> means = new ArrayList<Double>(Collections.nCopies(
                prevPositions.get(0).size(), 0.0));

        // Calculate mean
        for (DoubleArray positions : prevPositions) {
            for (int i = 0; i < positions.size(); i++) {
                means.set(i, means.get(i) + positions.get(i));
            }
        }
        for (int i = 0; i < means.size(); i++) {
            means.set(i, means.get(i) / observedJointStates);
        }

        // Calculate variance
        for (DoubleArray positions : prevPositions) {
            for (int i = 0; i < positions.size(); i++) {
                variances.set(
                        i,
                        (variances.get(i) + Math.pow(
                                ((means.get(i) - positions.get(i)) * 1), 2.0)));
            }
        }
        for (int i = 0; i < variances.size(); i++) {
            variances.set(i, variances.get(i) / observedJointStates);
        }

        return variances;
    }

    private Movement createAnnotation(JCas aJCas, JointState js, int index) {
        Movement moving = new Movement(aJCas);
        moving.setJointName(js.getName(index));
        moving.setBegin(js.getSeq() - observedJointStates);
        moving.setEnd(js.getSeq());
        return moving;
    }
}
