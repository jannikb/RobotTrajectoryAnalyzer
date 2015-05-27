package robotTA.collision;

import java.util.Collection;
import java.util.Iterator;

import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.descriptor.ConfigurationParameter;
import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.ControllerError;
import robotTA.types.PlannedMovement;
import robotTA.types.UnplannedStop;

public class UnplannedStopAnnotator extends JCasAnnotator_ImplBase {

	public static final String CONTROLLERTYPE = "controllerType";
	@ConfigurationParameter(name = CONTROLLERTYPE)
	private String controllerType;

	@Override
	public void process(JCas aJCas) throws AnalysisEngineProcessException {
		FSIndex<Annotation> errIndex = aJCas
				.getAnnotationIndex(ControllerError.type);
		Iterator<Annotation> errIter = errIndex.iterator();
		ControllerError err;
		UnplannedStop ups;
		boolean didStop;
		Collection<PlannedMovement> moves = JCasUtil.select(aJCas,
				PlannedMovement.class);

		while (errIter.hasNext()) {
			err = (ControllerError) errIter.next();

			ups = new UnplannedStop(aJCas);
			ups.setJointName(err.getJointName());
			ups.setEnd(err.getEnd());
			didStop = false;

			for (PlannedMovement move : moves) {
				if (move.getJointName().equals(err.getJointName())) {
					// Check if the end of the movement and the start of the
					// error correlate
					if (err.getBegin() >= (move.getEnd() - 15)
							&& err.getBegin() <= (move.getEnd() + 15)) {
						ups.setBegin(err.getBegin());
						didStop = true;
					}

					// Check if a movement started while the error persisted
					if (move.getBegin() >= (err.getBegin() + 15)
							&& move.getEnd() < err.getEnd()
							&& ups.getEnd() > move.getBegin()) {
						ups.setEnd(move.getBegin());
					}
				}
			}

			if (didStop) {
				ups.addToIndexes();
			}
		}
	}
}
