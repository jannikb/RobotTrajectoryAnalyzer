package robotTA.pipeline;

import org.apache.uima.UIMAException;
import org.apache.uima.analysis_engine.AnalysisEngine;
import org.apache.uima.analysis_engine.AnalysisEngineDescription;
import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.fit.factory.AggregateBuilder;
import org.apache.uima.fit.factory.AnalysisEngineFactory;
import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.jcas.JCas;
import org.apache.uima.resource.ResourceInitializationException;

import robotTA.collision.SelfCollisionAnnotator;
import robotTA.collision.UnplannedStopAnnotator;
import robotTA.controller.ControllerAnnotator;
import robotTA.controller.ErrorAnnotator;
import robotTA.controller.PlannedMovementAnnotator;
import robotTA.jointstate.AccelerationAnnotator;
import robotTA.jointstate.JointStateAnnotator;
import robotTA.jointstate.MovementAnnotator;
import robotTA.jointstate.MovementDirectionAnnotator;
import robotTA.oscillation.OscillationAnnotator;
import robotTA.robotstate.RobotStateAnnotator;
import robotTA.types.ControllerError;
import robotTA.types.Movement;
import robotTA.types.NegativeMovement;
import robotTA.types.Oscillation;
import robotTA.types.PlannedMovement;
import robotTA.types.PositiveMovement;
import robotTA.types.SelfCollision;
import robotTA.types.UnplannedMovement;
import robotTA.types.UnplannedStop;

public class JointStatePipeline {

    public static JCas getJCas(String dbAddress, String dbName,
            String collection) throws ResourceInitializationException,
            AnalysisEngineProcessException {

        AggregateBuilder builder = new AggregateBuilder();

        AnalysisEngineDescription jointState = AnalysisEngineFactory
                .createEngineDescription(JointStateAnnotator.class,
                        JointStateAnnotator.COLLECTION, collection,
                        JointStateAnnotator.DBADDRESS, dbAddress,
                        JointStateAnnotator.DBNAME, dbName);
        builder.add(jointState);

        AnalysisEngineDescription acceleration = AnalysisEngineFactory
                .createEngineDescription(AccelerationAnnotator.class);
        builder.add(acceleration);

        AnalysisEngineDescription movement = AnalysisEngineFactory
                .createEngineDescription(MovementAnnotator.class,
                        MovementAnnotator.MINVARIANCE, 0.000003f,
                        MovementAnnotator.OBSERVEDJOINTSTATES, 10);
        builder.add(movement);

        AnalysisEngineDescription movementDirection = AnalysisEngineFactory
                .createEngineDescription(MovementDirectionAnnotator.class);
        builder.add(movementDirection);

        AnalysisEngineDescription oscillation = AnalysisEngineFactory
                .createEngineDescription(OscillationAnnotator.class,
                        OscillationAnnotator.MAXTIMEVARIANCE, 50f,
                        OscillationAnnotator.MAXPOSITIONVARIANCE, 0.1f);
        builder.add(oscillation);

        AnalysisEngineDescription taskSpace = AnalysisEngineFactory
                .createEngineDescription(RobotStateAnnotator.class,
                        RobotStateAnnotator.DBADDRESS, dbAddress,
                        RobotStateAnnotator.DBNAME, "pr2");
        builder.add(taskSpace);

        AnalysisEngineDescription rcontrollerInput = AnalysisEngineFactory
                .createEngineDescription(ControllerAnnotator.class,
                        ControllerAnnotator.COLLECTION,
                        "r_arm_controller_state",
                        ControllerAnnotator.DBADDRESS, dbAddress,
                        ControllerAnnotator.DBNAME, dbName,
                        ControllerAnnotator.CONTROLLERTYPE,
                        "r_arm_controller_state");
        builder.add(rcontrollerInput);

        AnalysisEngineDescription lcontrollerInput = AnalysisEngineFactory
                .createEngineDescription(ControllerAnnotator.class,
                        ControllerAnnotator.COLLECTION,
                        "l_arm_controller_state",
                        ControllerAnnotator.DBADDRESS, dbAddress,
                        ControllerAnnotator.DBNAME, dbName,
                        ControllerAnnotator.CONTROLLERTYPE,
                        "l_arm_controller_state");
        builder.add(lcontrollerInput);

        AnalysisEngineDescription error = AnalysisEngineFactory
                .createEngineDescription(ErrorAnnotator.class,
                        ErrorAnnotator.MINERROR, 0.01f,
                        ErrorAnnotator.MINLENGTH, 5);
        builder.add(error);

        AnalysisEngineDescription plannedMovement = AnalysisEngineFactory
                .createEngineDescription(PlannedMovementAnnotator.class);
        builder.add(plannedMovement);

        AnalysisEngineDescription unplannedStop = AnalysisEngineFactory
                .createEngineDescription(UnplannedStopAnnotator.class,
                        UnplannedStopAnnotator.CONTROLLERTYPE,
                        "l_arm_controller_state");
        builder.add(unplannedStop);

        AnalysisEngineDescription selfCollision = AnalysisEngineFactory
                .createEngineDescription(SelfCollisionAnnotator.class);
        builder.add(selfCollision);

        AnalysisEngine aggregateEngine = builder.createAggregate();

        JCas jCas = aggregateEngine.newJCas();

        aggregateEngine.process(jCas);

        return jCas;
    }

    public static void main(String[] args) throws UIMAException {

        String[] collections = { "roslog", "collision4", "collision5",
                "oscillation2", "oscillation3", "oscillation4" };
        int cIndex = 4;

        System.out.println("Analysing " + collections[cIndex]);

        JCas jCas = getJCas("localhost", collections[cIndex], "joint_states");

        System.out.println("----------------- stop");

        for (UnplannedStop ups : JCasUtil.select(jCas, UnplannedStop.class)) {
            System.out.println("Start: " + ups.getBegin());
            System.out.println("End: " + ups.getEnd());
            System.out.println("Name: " + ups.getJointName());
        }

        System.out.println("---------------- collision");

        for (SelfCollision sc : JCasUtil.select(jCas, SelfCollision.class)) {
            System.out.println("Start: " + sc.getBegin());
            System.out.println("End: " + sc.getEnd());
            System.out.println("P: " + sc.getPerpetrator() + " V: "
                    + sc.getVictim());
        }

        System.out.println("----------------error");

        for (ControllerError err : JCasUtil.select(jCas, ControllerError.class)) {
            System.out.println("Name: " + err.getJointName());
            System.out.println("Start: " + err.getBegin());
            System.out.println("End: " + err.getEnd());
        }

        System.out.println("----------------movement");

        for (Movement move : JCasUtil.select(jCas, Movement.class)) {
            System.out.println("Name: " + move.getJointName());
            System.out.println("Start: " + move.getBegin());
            System.out.println("End: " + move.getEnd());
        }

        System.out.println("----------------planned movement");

        for (PlannedMovement move : JCasUtil
                .select(jCas, PlannedMovement.class)) {
            System.out.println("Name: " + move.getJointName());
            System.out.println("Start: " + move.getBegin());
            System.out.println("End: " + move.getEnd());
        }

        System.out.println("----------------unplanned movement");

        for (UnplannedMovement move : JCasUtil.select(jCas,
                UnplannedMovement.class)) {
            System.out.println("Name: " + move.getJointName());
            System.out.println("Start: " + move.getBegin());
            System.out.println("End: " + move.getEnd());
        }

        System.out.println("----------------positive movement");

        for (PositiveMovement move : JCasUtil.select(jCas, PositiveMovement.class)) {
            System.out.println("Name: " + move.getJointName());
            System.out.println("Start: " + move.getBegin());
            System.out.println("End: " + move.getEnd());
        }

        System.out.println("----------------negative movement");

        for (NegativeMovement move : JCasUtil.select(jCas, NegativeMovement.class)) {
            System.out.println("Name: " + move.getJointName());
            System.out.println("Start: " + move.getBegin());
            System.out.println("End: " + move.getEnd());
        }

        System.out.println("----------------oscillation");

        for (Oscillation osc : JCasUtil.select(jCas, Oscillation.class)) {
            System.out.println("Name: " + osc.getJointName());
            System.out.println("Start: " + osc.getBegin());
            System.out.println("End: " + osc.getEnd());
        }

    }
}