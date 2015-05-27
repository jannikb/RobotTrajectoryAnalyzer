package robotTA.controller;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.descriptor.ConfigurationParameter;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.ControllerError;
import robotTA.types.ControllerInput;

public class ErrorAnnotator extends JCasAnnotator_ImplBase {
    
    public static final String MINERROR = "minError";
    @ConfigurationParameter(name = MINERROR)
    private double minError;
    
    public static final String MINLENGTH = "minLength";
    @ConfigurationParameter(name = MINLENGTH)
    private int minLength;

//	private double minError = 0.01;
//	private int minLength = 5;

	@Override
	public void process(JCas aJCas) throws AnalysisEngineProcessException {
		FSIndex<Annotation> ciIndex = aJCas
				.getAnnotationIndex(ControllerInput.type);
		Iterator<Annotation> ciIter = ciIndex.iterator();
		ControllerInput cInput;
		HashMap<String, ArrayList<ControllerError>> controllerErrors = new HashMap<String, ArrayList<ControllerError>>();
		ArrayList<ControllerError> errors;
		boolean inErrState;

		while (ciIter.hasNext()) {
			cInput = (ControllerInput) ciIter.next();

			if (!controllerErrors.containsKey(cInput.getControllerType())) {
				controllerErrors.put(
						cInput.getControllerType(),
						new ArrayList<ControllerError>(Collections.nCopies(cInput
								.getJointNames().size(), null)));
			}
			errors = controllerErrors.get(cInput.getControllerType());

			for (int i = 0; i < cInput.getError().getPositions().size(); i++) {
				inErrState = Math.abs(cInput.getError().getPositions(i)) >= minError;
				if (errors.get(i) != null && inErrState) {
					errors.get(i).setEnd(cInput.getBegin());
				} else if (errors.get(i) == null && inErrState) {
					errors.set(i, createAnnotation(aJCas, cInput, i));
				} else if (errors.get(i) != null && !inErrState) {
					if (errors.get(i).getEnd() - errors.get(i).getBegin() >= minLength)
						errors.get(i).addToIndexes();
					errors.set(i, null);
				}
			}
		}

		for (ArrayList<ControllerError> errs : controllerErrors.values())
			for (ControllerError err : errs)
				if (err != null && err.getEnd() - err.getBegin() >= minLength) 
					err.addToIndexes();

	}

	private ControllerError createAnnotation(JCas aJCas, ControllerInput cInput, int index) {
		ControllerError err = new ControllerError(aJCas);
		err.setBegin(cInput.getBegin());
		err.setEnd(cInput.getBegin());
		err.setJointName(cInput.getJointNames(index));
		return err;
	}

}
