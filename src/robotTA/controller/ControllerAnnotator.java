package robotTA.controller;

import org.apache.uima.UimaContext;
import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.cas.FSIndex;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.descriptor.ConfigurationParameter;
import org.apache.uima.jcas.JCas;
import org.apache.uima.jcas.tcas.Annotation;
import org.apache.uima.resource.ResourceInitializationException;

import robotTA.types.ControllerInput;
import robotTA.types.JointState;
import robotTA.utils.EasyArray;
import robotTA.utils.JointStateUtils;

import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.MongoClient;

import java.net.UnknownHostException;
import java.util.Date;
import java.util.ArrayList;
import java.util.Iterator;

public class ControllerAnnotator extends JCasAnnotator_ImplBase {

	public static final String COLLECTION = "collection";
	@ConfigurationParameter(name = COLLECTION)
	private String collection;

	public static final String DBADDRESS = "dbAddress";
	@ConfigurationParameter(name = DBADDRESS)
	private String dbAddress;

	public static final String DBNAME = "dbName";
	@ConfigurationParameter(name = DBNAME)
	private String dbName;
	
	public static final String CONTROLLERTYPE = "controllerType";
    @ConfigurationParameter(name = CONTROLLERTYPE)
    private String controllerType;

	private MongoClient mongoClient;
	private DB db;

	@Override
	public void initialize(UimaContext aContext)
			throws ResourceInitializationException {
		super.initialize(aContext);

		try {
			mongoClient = new MongoClient(dbAddress, 27017);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}

		db = mongoClient.getDB(dbName);
	}

	@SuppressWarnings("unchecked")
	@Override
	public void process(JCas aJCas) throws AnalysisEngineProcessException {
		DBCollection coll = db.getCollection(collection);
		DBCursor cursor = coll.find();
		Object jointNames;
		DBObject desired;
		DBObject actual;
		DBObject error;
		DBObject header;
		Date date;
		ControllerInput cInput;
		ArrayList<ControllerInput> cInputs = new ArrayList<ControllerInput>();

		// Create the ControllerInputs with the info from the DB
		try {
			while (cursor.hasNext()) {
				cursor.next();
				header = (DBObject) cursor.curr().get("header");
				date = (Date) header.get("stamp");
				jointNames = cursor.curr().get("joint_names");
				desired = (DBObject) cursor.curr().get("desired");
				actual = (DBObject) cursor.curr().get("actual");
				error = (DBObject) cursor.curr().get("error");
				if (date != null && jointNames != null && desired != null
						&& actual != null && error != null) {
					cInput = new ControllerInput(aJCas);
					cInput.setTime((long) date.getTime());
					cInput.setJointNames(EasyArray.createStringArray(aJCas,
							(ArrayList<String>) jointNames));
					cInput.setDesired(JointStateUtils.readJointTrajectoryPoint(
							aJCas, desired));
					cInput.setActual(JointStateUtils.readJointTrajectoryPoint(
							aJCas, actual));
					cInput.setError(JointStateUtils.readJointTrajectoryPoint(
							aJCas, error));
					cInputs.add(cInput);
				}
			}
		} finally {
			cursor.close();
		}

		// Set the start of the annotator
		FSIndex<Annotation> jsIndex = aJCas.getAnnotationIndex(JointState.type);
		Iterator<Annotation> jsIter = jsIndex.iterator();
		JointState js;
		JointState prevJs = null;
		int cIndex = 0;
		while (jsIter.hasNext()) {
			js = (JointState) jsIter.next();
			if (cIndex < cInputs.size() && cInputs.get(cIndex).getTime() < js.getTime() && prevJs != null) {
				cInputs.get(cIndex).setBegin(js.getBegin());
				cInputs.get(cIndex).setEnd(js.getEnd());
				cInputs.get(cIndex).setControllerType(controllerType);
				cInputs.get(cIndex).addToIndexes(aJCas);
				cIndex++;
			}
			prevJs = js;
		}
	}

	@Override
	public void destroy() {
		mongoClient.close();
	}

}
