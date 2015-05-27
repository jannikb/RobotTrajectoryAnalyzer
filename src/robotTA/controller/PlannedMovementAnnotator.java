package robotTA.controller;

import java.util.Collection;
import java.util.Iterator;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.ControllerError;
import robotTA.types.Movement;
import robotTA.types.PlannedMovement;
import robotTA.types.UnplannedMovement;

public class PlannedMovementAnnotator extends JCasAnnotator_ImplBase {

	@Override
	public void process(JCas aJCas) throws AnalysisEngineProcessException {
		FSIndex<Annotation> movIndex = aJCas.getAnnotationIndex(Movement.type);
		Iterator<Annotation> movIter = movIndex.iterator();
		Movement move;
		boolean foundError;
		Collection<ControllerError> errors = JCasUtil.select(aJCas,
				ControllerError.class);
		UnplannedMovement umov;
		PlannedMovement pmov;

		while (movIter.hasNext()) {
			move = (Movement) movIter.next();

			foundError = false;
			for (ControllerError err : errors) {
				// Check if a error started as the movement started
				if (move.getJointName().equals(err.getJointName())
						&& move.getBegin() >= (err.getBegin() - 20)
						&& move.getBegin() <= (err.getBegin() + 20)) {
					foundError = true;
					break;
				}
			}

			if (foundError) {
				umov = new UnplannedMovement(aJCas);
				umov.setBegin(move.getBegin());
				umov.setEnd(move.getEnd());
				umov.setJointName(move.getJointName());
				umov.addToIndexes();
			} else {
				pmov = new PlannedMovement(aJCas);
				pmov.setBegin(move.getBegin());
				pmov.setEnd(move.getEnd());
				pmov.setJointName(move.getJointName());
				pmov.addToIndexes();
			}
		}

	}

}
