package robotTA.jointstate;

import org.apache.uima.UimaContext;
import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.fit.component.JCasAnnotator_ImplBase;
import org.apache.uima.fit.descriptor.ConfigurationParameter;
import org.apache.uima.jcas.JCas;
import org.apache.uima.resource.ResourceInitializationException;

import robotTA.types.JointState;
import robotTA.types.JointTrajectoryPoint;
import robotTA.utils.EasyArray;

import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.MongoClient;

import java.net.UnknownHostException;
import java.util.Date;
import java.util.ArrayList;

public class JointStateAnnotator extends JCasAnnotator_ImplBase {

    public static final String COLLECTION = "collection";
    @ConfigurationParameter(name = COLLECTION)
    private String collection;

    public static final String DBADDRESS = "dbAddress";
    @ConfigurationParameter(name = DBADDRESS)
    private String dbAddress;

    public static final String DBNAME = "dbName";
    @ConfigurationParameter(name = DBNAME)
    private String dbName;

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
        Object name;
        Object seq;
        Object frameID;
        Object position;
        Object velocity;
        Object effort;
        DBObject header;
        Date date;
        JointTrajectoryPoint jtPoint;
        JointState annotation;
        try {
            while (cursor.hasNext()) {
                cursor.next();
                annotation = new JointState(aJCas);
                jtPoint = new JointTrajectoryPoint(aJCas);
                name = cursor.curr().get("name");
                header = (DBObject) cursor.curr().get("header");
                seq = header.get("seq");
                date = (Date) header.get("stamp");
                frameID = header.get("frame_id");
                position = cursor.curr().get("position");
                velocity = cursor.curr().get("velocity");
                effort = cursor.curr().get("effort");
                if (name != null && seq != null && date != null
                        && frameID != null && position != null
                        && velocity != null && effort != null) {
                    annotation.setName(EasyArray.createStringArray(aJCas,
                            (ArrayList<String>) name));
                    annotation.setSeq((int) seq);
                    annotation.setTime((long) date.getTime());
                    annotation.setFrameID((String) frameID);
                    jtPoint.setPositions(EasyArray.createDoubleArray(aJCas,
                            (ArrayList<Double>) position));
                    jtPoint.setVelocities(EasyArray.createDoubleArray(aJCas,
                            (ArrayList<Double>) velocity));
                    jtPoint.setEffort(EasyArray.createDoubleArray(aJCas,
                            (ArrayList<Double>) effort));
                    annotation.setJointTrajectoryPoint(jtPoint);
                    annotation.setBegin(annotation.getSeq());
                    annotation.setEnd(annotation.getSeq());
                    annotation.addToIndexes();
                }
            }
        } finally {
            cursor.close();
        }
    }
    
    @Override
    public void destroy() {
        mongoClient.close();
    }

}