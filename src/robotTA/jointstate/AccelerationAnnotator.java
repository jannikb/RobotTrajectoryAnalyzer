package robotTA.jointstate;

import java.util.Iterator;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.cas.DoubleArray;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.Acceleration;
import robotTA.types.JointState;

public class AccelerationAnnotator extends
        org.apache.uima.fit.component.JCasAnnotator_ImplBase {

    @Override
    public void process(JCas aJCas) throws AnalysisEngineProcessException {
        FSIndex<Annotation> jsIndex = aJCas.getAnnotationIndex(JointState.type);
        Iterator<Annotation> jsIter = jsIndex.iterator();

        DoubleArray prevVel;
        Double prevTime;
        JointState js;

        if (jsIter.hasNext()) {
            js = (JointState) jsIter.next();
            prevVel = js.getJointTrajectoryPoint().getVelocities();
            prevTime = js.getTime();
        } else {
            return;
        }

        double velDiff;
        double timeDiff;

        while (jsIter.hasNext()) {
            js = (JointState) jsIter.next();
            Acceleration accel = new Acceleration(aJCas);

            accel.setValue(new DoubleArray(aJCas, prevVel.size()));
            for (int i = 0; i < prevVel.size(); i++) {
                velDiff = js.getJointTrajectoryPoint().getVelocities(i) - prevVel.get(i);
                timeDiff = js.getTime() - prevTime;
                if (timeDiff != 0) {
                    accel.setValue(i, velDiff / timeDiff);
                } else {
                    continue;
                }
            }

            accel.setBegin(js.getSeq());
            accel.setEnd(js.getSeq());
            accel.addToIndexes();
            
            prevVel = js.getJointTrajectoryPoint().getVelocities();
            prevTime = js.getTime();
        }
    }

}
