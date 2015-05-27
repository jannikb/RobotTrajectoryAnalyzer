package robotTA.utils;

import java.util.ArrayList;
import java.util.Iterator;

import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.cas.StringArray;

import robotTA.types.Acceleration;
import robotTA.types.JointState;
import robotTA.types.JointTrajectoryPoint;
import robotTA.types.Oscillation;

import com.mongodb.DBObject;

public class JointStateUtils {

	public static int getIndex(JointState js, String name) {
		StringArray arr = js.getName();

		for (int i = 0; i < arr.size(); i++) {
			if (arr.get(i).equals(name))
				return i;
		}

		throw new IllegalArgumentException("Name not found.");
	}

	public static int getIndex(JCas jCas, String name) {
		ArrayList<String> names = getJointNames(jCas);
		for (int i = 0; i < names.size(); i++) {
			if (name.equals(names.get(i)))
				return i;
		}
		return -1;
	}

	public static double getPosition(JointState js, String name) {
		return js.getJointTrajectoryPoint().getPositions(getIndex(js, name));
	}
	
	public static double getVelocity(JointState js, String name) {
		return js.getJointTrajectoryPoint().getVelocities(getIndex(js, name));
	}

	public static ArrayList<String> getJointNames(JCas jCas) {
		Iterator<JointState> jsIter = JCasUtil.select(jCas, JointState.class)
				.iterator();
		if (jsIter.hasNext()) {
			StringArray joints = jsIter.next().getName();
			return EasyArray.createArrayList(joints);
		} else
			return null;
	}

	public static ArrayList<Oscillation> getOscillations(JCas jCas,
			String jointName) {
		Iterator<Oscillation> oscIter = JCasUtil
				.select(jCas, Oscillation.class).iterator();
		ArrayList<Oscillation> oscillations = new ArrayList<Oscillation>();
		Oscillation osc;

		while (oscIter.hasNext()) {
			osc = oscIter.next();
			if (osc.getJointName().equals(jointName))
				oscillations.add(osc);
		}

		return oscillations;
	}

	@SuppressWarnings("unchecked")
	public static JointTrajectoryPoint readJointTrajectoryPoint(JCas jCas,
			DBObject dbObject) {
		Object positions = dbObject.get("positions");
		Object velocities = dbObject.get("velocities");
		Object accelerations = dbObject.get("accelerations");
		Object effort = dbObject.get("effort");
		JointTrajectoryPoint jtPoint = new JointTrajectoryPoint(jCas);
		if (positions != null && velocities != null && accelerations != null
				&& effort != null) {
			jtPoint.setPositions(EasyArray.createDoubleArray(jCas,
					(ArrayList<Double>) positions));
			jtPoint.setVelocities(EasyArray.createDoubleArray(jCas,
					(ArrayList<Double>) velocities));
			jtPoint.setAccelerations(EasyArray.createDoubleArray(jCas,
					(ArrayList<Double>) accelerations));
			jtPoint.setEffort(EasyArray.createDoubleArray(jCas,
					(ArrayList<Double>) effort));
		}
		return jtPoint;
	}

}
