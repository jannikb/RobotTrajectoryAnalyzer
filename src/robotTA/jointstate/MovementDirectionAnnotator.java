package robotTA.jointstate;

import java.util.Collection;
import java.util.Iterator;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.JointState;
import robotTA.types.Movement;
import robotTA.types.NegativeMovement;
import robotTA.types.PositiveMovement;
import robotTA.utils.JointStateUtils;

public class MovementDirectionAnnotator extends JCasAnnotator_ImplBase {

    @Override
    public void process(JCas aJCas) throws AnalysisEngineProcessException {
        FSIndex<Annotation> movIndex = aJCas.getAnnotationIndex(Movement.type);
        Iterator<Annotation> movIter = movIndex.iterator();
        Movement move;
        Collection<JointState> jointStates;
        double prevPos = 0;
        double actPos;
        double posDiff;
        boolean newMove;
        PositiveMovement posMove = null;
        NegativeMovement negMove = null;

        while (movIter.hasNext()) {
            move = (Movement) movIter.next();
            jointStates = JCasUtil.selectCovered(JointState.class, move);
            newMove = true;
            posMove = null;
            negMove = null;

            for (JointState js : jointStates) {
                actPos = JointStateUtils.getPosition(js,
                        move.getJointName());
                if (newMove) {
                    newMove = false;
                } else {
                    posDiff = actPos - prevPos;
                    if (posMove != null) {
                        if (posDiff >= 0) {
                            posMove.setEnd(js.getEnd());
                        } else {
                            posMove.setEndPosition(prevPos);
                            posMove.addToIndexes();
                            posMove = null;
                            negMove = createNegMovAnn(aJCas, js, move.getJointName(), prevPos);
                        }
                    } else if (negMove != null) {
                        if (posDiff <= 0) {
                            negMove.setEnd(js.getEnd());
                        } else {
                            negMove.setEndPosition(prevPos);
                            negMove.addToIndexes();
                            negMove = null;
                            posMove = createPosMovAnn(aJCas, js, move.getJointName(), prevPos);
                        }
                    } else {
                        if (posDiff >= 0) {
                           posMove = createPosMovAnn(aJCas, js, move.getJointName(), prevPos); 
                        } else {
                            negMove = createNegMovAnn(aJCas, js, move.getJointName(), prevPos);
                        }
                    }
                }
                prevPos = actPos;
            }
            
            if (posMove != null)
                posMove.addToIndexes();
            
            if (negMove != null)
                negMove.addToIndexes();
        }
    }

    private PositiveMovement createPosMovAnn(JCas jCas, JointState js, String name, double pos) {
        PositiveMovement posMove = new PositiveMovement(jCas);
        posMove.setJointName(name);
        posMove.setStartPosition(pos);
        posMove.setBegin(js.getBegin() - 1);
        posMove.setEnd(js.getEnd());

        return posMove;
    }
    
    private NegativeMovement createNegMovAnn(JCas jCas, JointState js, String name, double pos) {
        NegativeMovement negMove = new NegativeMovement(jCas);
        negMove.setJointName(name);
        negMove.setBegin(js.getBegin() -1);
        negMove.setEnd(js.getEnd());

        return negMove;
    }

}
