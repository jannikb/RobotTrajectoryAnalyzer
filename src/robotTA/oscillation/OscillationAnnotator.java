package robotTA.oscillation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.descriptor.ConfigurationParameter;
import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.Movement;
import robotTA.types.Oscillation;
import robotTA.types.PositiveMovement;

public class OscillationAnnotator extends JCasAnnotator_ImplBase {
	
	public static final String MAXTIMEVARIANCE = "maxTimeVariance";
    @ConfigurationParameter(name = MAXTIMEVARIANCE)
    private double maxTimeVariance;
    
    public static final String MAXPOSITIONVARIANCE = "maxPositionVariance";
    @ConfigurationParameter(name = MAXPOSITIONVARIANCE)
    private double maxPositionVariance;
	
	//private double maxTimeVariance = 50;
	///private double maxPositionVariance = 0.1;

	@Override
	public void process(JCas aJCas) throws AnalysisEngineProcessException {
		FSIndex<Annotation> movIndex = aJCas.getAnnotationIndex(Movement.type);
		Iterator<Annotation> movIter = movIndex.iterator();
		Movement move;
		Collection<PositiveMovement> posMoves;
		ArrayList<Double> posDiffs;
		ArrayList<Double> timeDiffs;
		Double posVar;
		Double timeVar;
		int moveCtr;
		Oscillation osc;

		while (movIter.hasNext()) {
			move = (Movement) movIter.next();
			posMoves = JCasUtil.selectCovered(PositiveMovement.class, move);
			posDiffs = new ArrayList<Double>();
			timeDiffs = new ArrayList<Double>();
			moveCtr = 0;

			for (PositiveMovement posMove : posMoves) {
				if (move.getJointName().equals(posMove.getJointName())) {
					posDiffs.add(posMove.getStartPosition()
							- posMove.getEndPosition());
					timeDiffs.add((double) (posMove.getBegin() - posMove
							.getEnd()));
					moveCtr++;
				}
			}

			posVar = calcVariance(posDiffs);
			timeVar = calcVariance(timeDiffs);

//			System.out.println(move.getJointName() + " " + posVar + " "
//					+ timeVar + " " + moveCtr);

			if (moveCtr > 3 && posVar < maxPositionVariance && timeVar < maxTimeVariance) {
				osc = new Oscillation(aJCas);
				osc.setBegin(move.getBegin());
				osc.setEnd(move.getEnd());
				osc.setJointName(move.getJointName());
				// osc.setFrequency();
				osc.addToIndexes();
			}

		}
	}

	private Double calcVariance(ArrayList<Double> numbers) {
		double sum = 0;
		double mean = 0;
		double varSum = 0;
		double variance = 0;

		for (Double num : numbers) {
			sum += num;
		}
		mean = sum / numbers.size();

		for (Double num : numbers) {
			varSum += Math.pow((mean - num), 2.0);
		}
		variance = varSum / numbers.size();

		return variance;
	}
}
