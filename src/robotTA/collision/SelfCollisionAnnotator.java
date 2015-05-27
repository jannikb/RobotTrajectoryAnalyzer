package robotTA.collision;

import java.util.Collection;
import java.util.Iterator;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;

import robotTA.types.SelfCollision;
import robotTA.types.UnplannedMovement;
import robotTA.types.UnplannedStop;

public class SelfCollisionAnnotator extends JCasAnnotator_ImplBase {

	@Override
	public void process(JCas aJCas) throws AnalysisEngineProcessException {
		FSIndex<Annotation> upsIndex = aJCas
				.getAnnotationIndex(UnplannedStop.type);
		Iterator<Annotation> upsIter = upsIndex.iterator();
		UnplannedStop ups;
		String victim;

		while (upsIter.hasNext()) {
			ups = (UnplannedStop) upsIter.next();

			victim = findCollsion(aJCas, ups);
			if (victim != null) {
				SelfCollision sc = new SelfCollision(aJCas);
				sc.setBegin(ups.getBegin());
				sc.setEnd(ups.getEnd());
				sc.setPerpetrator(ups.getJointName());
				sc.setVictim(victim);
				sc.addToIndexes();
			}
		}
	}

	private String findCollsion(JCas aJCas, UnplannedStop ups) {
		Collection<UnplannedMovement> unplMoves = JCasUtil.select(aJCas,
				UnplannedMovement.class);
		String victim = null;

		for (UnplannedMovement unplMove : unplMoves) {
			if (!unplMove.getJointName().equals(ups.getJointName())
					&& unplMove.getBegin() >= (ups.getBegin() - 15)
					&& unplMove.getBegin() <= (ups.getBegin() + 15)) {
				victim = unplMove.getJointName();
				break;
			}
		}
		return victim;
	}
}
