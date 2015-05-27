package robotTA.gui;

import java.awt.EventQueue;

import javax.swing.BorderFactory;
import javax.swing.DefaultListModel;
import javax.swing.JFrame;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

import java.awt.FlowLayout;

import javax.swing.JTextField;
import javax.swing.JComboBox;
import javax.swing.JButton;

import org.apache.uima.analysis_engine.AnalysisEngineProcessException;
import org.apache.uima.fit.util.JCasUtil;
import org.apache.uima.jcas.JCas;
import org.apache.uima.resource.ResourceInitializationException;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import robotTA.pipeline.JointStatePipeline;
import robotTA.types.Acceleration;
import robotTA.types.ControllerError;
import robotTA.types.ControllerInput;
import robotTA.types.DistanceToLimit;
import robotTA.types.JointState;
import robotTA.types.Movement;
import robotTA.types.NegativeMovement;
import robotTA.types.Oscillation;
import robotTA.types.PositiveMovement;
import robotTA.types.SelfCollision;
import robotTA.utils.EasyArray;
import robotTA.utils.JointStateUtils;

import com.mongodb.MongoClient;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Collection;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;
import java.awt.Stroke;

public class ApplicationMain {

    private JFrame frmTrajectoryAnalyzer;
    private JTextField txtLocalhost;
    private MongoClient mongoClient;
    private String dbAddress;
    private String dbName = "collsion4";
    private JCas jCas;
    private String[] trajProperties = { "Position", "Velocity", "PositionVelocity", "Effort",
            "Acceleration", "Movement", "MovementDirection", "Oscillation",
            "DistanceToLimit", "ControllerInputPosition",
            "ControllerInputVelocity", "ControllerInputAcceleration", "Error",
            "SelfCollision" };

    /**
     * Launch the application.
     */
    public static void main(String[] args) {
        EventQueue.invokeLater(new Runnable() {
            public void run() {
                try {
                    ApplicationMain window = new ApplicationMain();
                    window.frmTrajectoryAnalyzer.setVisible(true);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
    }

    /**
     * Create the application.
     */
    public ApplicationMain() {
        initialize();
    }

    /**
     * Initialize the contents of the frame.
     */
    private void initialize() {
        frmTrajectoryAnalyzer = new JFrame();
        frmTrajectoryAnalyzer.setTitle("Trajectory Analyzer");
        frmTrajectoryAnalyzer.setBounds(100, 100, 840, 450);
        frmTrajectoryAnalyzer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frmTrajectoryAnalyzer.getContentPane()
                .setLayout(new BorderLayout(0, 0));

        JPanel panel = new JPanel();
        frmTrajectoryAnalyzer.getContentPane().add(panel);
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));

        JPanel panel_0 = new JPanel();
        panel.add(panel_0);
        panel_0.setLayout(new FlowLayout(FlowLayout.LEFT, 5, 5));

        JPanel panel_1 = new JPanel();
        panel_0.add(panel_1);
        panel_1.setLayout(new GridLayout(2, 3, 4, 4));
        // panel_1.setMaximumSize(new Dimension(600, 80));
        // panel_1.setAlignmentX(Component.RIGHT_ALIGNMENT);

        JLabel lblDatabaseAddress = new JLabel("Database Address");
        panel_1.add(lblDatabaseAddress);

        txtLocalhost = new JTextField();
        panel_1.add(txtLocalhost);
        txtLocalhost.setText("localhost");
        txtLocalhost.setColumns(10);

        JButton btnConnect = new JButton("Connect");
        panel_1.add(btnConnect);

        JLabel lblTrajectory = new JLabel("Trajectory");
        panel_1.add(lblTrajectory);

        JComboBox<String> cbTrajectories = new JComboBox<String>();
        cbTrajectories.setEnabled(false);
        panel_1.add(cbTrajectories);

        JButton btnAnalyze = new JButton("Analyze");
        btnAnalyze.setEnabled(false);
        panel_1.add(btnAnalyze);

        JPanel panel_2 = new JPanel();
        panel.add(panel_2);
        panel_2.setLayout(new BorderLayout());
        // panel_2.setPreferredSize(new Dimension(1000, 400));

        JPanel panel_3 = new JPanel();
        panel_2.add(panel_3, BorderLayout.WEST);
        panel_3.setBorder(BorderFactory.createLoweredSoftBevelBorder());
        GridBagLayout gbl_panel_3 = new GridBagLayout();
        gbl_panel_3.columnWidths = new int[] { 240, 0 };
        gbl_panel_3.rowHeights = new int[] { 60, 100, 0 };
        gbl_panel_3.columnWeights = new double[] { 0.0, Double.MIN_VALUE };
        gbl_panel_3.rowWeights = new double[] { 0.0, 0.0, Double.MIN_VALUE };
        panel_3.setLayout(gbl_panel_3);

        JPanel panel_4 = new JPanel();
        GridBagConstraints gbc_panel_4 = new GridBagConstraints();
        gbc_panel_4.anchor = GridBagConstraints.NORTHWEST;
        gbc_panel_4.insets = new Insets(10, 10, 5, 5);
        gbc_panel_4.gridx = 0;
        gbc_panel_4.gridy = 0;
        panel_3.add(panel_4, gbc_panel_4);
        GridBagLayout gbl_panel_4 = new GridBagLayout();
        gbl_panel_4.columnWidths = new int[] { 104, 0 };
        gbl_panel_4.rowHeights = new int[] { 25, 25, 0 };
        gbl_panel_4.columnWeights = new double[] { 0.0, Double.MIN_VALUE };
        gbl_panel_4.rowWeights = new double[] { 0.0, 0.0, Double.MIN_VALUE };
        panel_4.setLayout(gbl_panel_4);

        JComboBox<String> cbProperties = new JComboBox<String>();
        cbProperties.setEnabled(false);
        for (int i = 0; i < trajProperties.length; i++) {
            cbProperties.addItem(trajProperties[i]);
        }
        GridBagConstraints gbc_cbProperties = new GridBagConstraints();
        gbc_cbProperties.anchor = GridBagConstraints.NORTHWEST;
        gbc_cbProperties.insets = new Insets(0, 0, 5, 0);
        gbc_cbProperties.gridx = 0;
        gbc_cbProperties.gridy = 0;
        panel_4.add(cbProperties, gbc_cbProperties);

        JPanel panel_5 = new JPanel();
        GridBagConstraints gbc_panel_5 = new GridBagConstraints();
        gbc_panel_5.anchor = GridBagConstraints.NORTHWEST;
        gbc_panel_5.insets = new Insets(10, 10, 5, 5);
        gbc_panel_5.gridx = 0;
        gbc_panel_5.gridy = 1;
        panel_3.add(panel_5, gbc_panel_5);
        GridBagLayout gbl_panel_5 = new GridBagLayout();
        gbl_panel_5.columnWidths = new int[] { 105, 0 };
        gbl_panel_5.rowHeights = new int[] { 25, 25, 25, 25, 0 };
        gbl_panel_5.columnWeights = new double[] { 0.0, Double.MIN_VALUE };
        gbl_panel_5.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0,
                Double.MIN_VALUE };
        panel_5.setLayout(gbl_panel_5);

        JScrollPane spJoints = new JScrollPane();
        DefaultListModel<String> lstJointsMdl = new DefaultListModel<String>();
        JList<String> lstJoints = new JList<String>(lstJointsMdl);
        spJoints.setViewportView(lstJoints);
        lstJoints.setEnabled(false);
        // lstJoints.setPreferredSize(new Dimension(200, 32));
        // lstJoints.setSize(200, 32);
        // lstJoints.setMinimumSize(new Dimension(200, 32));
        GridBagConstraints gbc_cbJoints = new GridBagConstraints();
        gbc_cbJoints.anchor = GridBagConstraints.NORTHWEST;
        gbc_cbJoints.fill = GridBagConstraints.BOTH;
        gbc_cbJoints.insets = new Insets(0, 0, 5, 0);
        gbc_cbJoints.gridx = 0;
        gbc_cbJoints.gridy = 2;
        panel_5.add(spJoints, gbc_cbJoints);

        JButton btnShow = new JButton("Show");
        btnShow.setEnabled(false);
        GridBagConstraints gbc_btnShow = new GridBagConstraints();
        gbc_btnShow.anchor = GridBagConstraints.NORTHWEST;
        gbc_btnShow.fill = GridBagConstraints.BOTH;
        gbc_btnShow.gridx = 0;
        gbc_btnShow.gridy = 3;
        panel_5.add(btnShow, gbc_btnShow);

        JPanel panel_6 = new JPanel();
        panel_2.add(panel_6, BorderLayout.CENTER);
        panel_6.setLayout(new BorderLayout());

        btnConnect.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent ae) {
                try {
                    dbAddress = txtLocalhost.getText();
                    mongoClient = new MongoClient(dbAddress, 27017);
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                }

                for (String dbName : mongoClient.getDatabaseNames())
                    cbTrajectories.addItem(dbName);

                cbTrajectories.setEnabled(true);
                btnAnalyze.setEnabled(true);
            }
        });

        btnAnalyze.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent ae) {
                dbName = (String) cbTrajectories.getSelectedItem();
                try {
                    jCas = JointStatePipeline.getJCas(dbAddress, dbName,
                            "joint_states");
                } catch (AnalysisEngineProcessException
                        | ResourceInitializationException e) {
                    e.printStackTrace();
                }

                ArrayList<String> joints = JointStateUtils.getJointNames(jCas);
                // lstJoints.add(joints);
                for (int i = 0; i < joints.size(); i++) {
                    lstJointsMdl.add(i, joints.get(i));
                }

                lstJoints.setEnabled(true);
                cbProperties.setEnabled(true);
                btnShow.setEnabled(true);
            }
        });

        btnShow.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent ae) {
                panel_6.removeAll();

                switch ((String) cbProperties.getSelectedItem()) {
                case "Position":
                    panel_6.add(getPositionChart(lstJoints.getSelectedValue()));
                    break;
                case "Velocity":
                    panel_6.add(getVelocityChart(lstJoints.getSelectedValue()));
                    break;
                case "PositionVelocity":
                	panel_6.add(getPositionVelocityChart(lstJoints.getSelectedValue()));
                    break;
                case "Acceleration":
                    panel_6.add(getAccelerationChart(lstJoints
                            .getSelectedValue()));
                    break;
                case "Movement":
                    panel_6.add(getMovingChart(lstJoints.getSelectedValue()));
                    break;
                case "MovementDirection":
                    panel_6.add(getMovementDirectionChart(lstJoints
                            .getSelectedValue()));
                    break;
                case "Oscillation":
                    panel_6.add(getOscillationChart(lstJoints
                            .getSelectedValue()));
                    break;
                case "DistanceToLimit":
                    panel_6.add(getLimitChart(lstJoints.getSelectedValue()));
                    break;
                case "Effort":
                    panel_6.add(getEffortChart(lstJoints.getSelectedValue()));
                    break;
                case "ControllerInputPosition":
                    panel_6.add(getControllerPositionInputChart(lstJoints
                            .getSelectedValue()));
                    break;
                case "ControllerInputVelocity":
                    panel_6.add(getControllerVelocityInputChart(lstJoints
                            .getSelectedValue()));
                    break;
                case "ControllerInputAcceleration":
                    panel_6.add(getControllerAccelerationInputChart(lstJoints
                            .getSelectedValue()));
                    break;
                case "Error":
                    panel_6.add(getErrorChart(lstJoints.getSelectedValue()));
                    break;
                case "SelfCollision":
                    panel_6.add(getSelfCollisionPositionChart(lstJoints
                            .getSelectedValue()));
                    break;
                }

                panel_6.revalidate();
                frmTrajectoryAnalyzer.revalidate();
            }
        });
    }

    private ChartPanel getPositionChart(String name) {
        XYSeries positions = new XYSeries("Position: " + name);
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            positions.add(js.getTime(), JointStateUtils.getPosition(js, name));
        }

        XYDataset xyDataset = new XYSeriesCollection(positions);
        JFreeChart chart = ChartFactory.createXYLineChart("Position over time",
                "Time", "Position", xyDataset, PlotOrientation.VERTICAL,
                true, true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getEffortChart(String name) {
        XYSeries effort = new XYSeries("Effort: " + name);
        int i;
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            i = JointStateUtils.getIndex(js, name);
            effort.add(js.getTime(), js.getJointTrajectoryPoint().getEffort(i));
        }

        XYDataset xyDataset = new XYSeriesCollection(effort);
        JFreeChart chart = ChartFactory.createXYLineChart("Effort over time",
                "Time", "Effort", xyDataset, PlotOrientation.VERTICAL, true,
                true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getVelocityChart(String name) {
        XYSeries velocities = new XYSeries("Velocity: " + name);
        int i;
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            i = JointStateUtils.getIndex(js, name);
            velocities.add(js.getTime(), js.getJointTrajectoryPoint()
                    .getVelocities(i));
        }

        XYDataset xyDataset = new XYSeriesCollection(velocities);
        JFreeChart chart = ChartFactory.createXYLineChart("Velocity over time",
                "Time", "Velocity", xyDataset, PlotOrientation.VERTICAL,
                true, true, false);

        return getChartPanel(chart);
    }
    
    private ChartPanel getPositionVelocityChart(String name) {
        XYSeries positions = new XYSeries("Position: " + name);
        XYSeries velocities = new XYSeries("Velocity: " + name);
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            positions.add(js.getTime(), JointStateUtils.getPosition(js, name));
            velocities.add(js.getTime(), JointStateUtils.getVelocity(js, name));
        }
        
        XYSeriesCollection xyDataset = new XYSeriesCollection();
        xyDataset.addSeries(positions);
        xyDataset.addSeries(velocities);
        JFreeChart chart = ChartFactory.createXYLineChart("Position and velocity over time",
                "Time", "Position/Velocity", xyDataset, PlotOrientation.VERTICAL,
                true, true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getMovingChart(String name) {
        XYSeriesCollection xyDataset = new XYSeriesCollection();

        XYSeries positions = new XYSeries("Position: " + name);
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            positions.add(js.getTime(), JointStateUtils.getPosition(js, name));
        }
        xyDataset.addSeries(positions);

        XYSeries moves;
        Collection<Movement> moveAnnotations = JCasUtil.select(jCas,
                Movement.class);
        for (Movement move : moveAnnotations) {
            if (move.getJointName().equals(name)) {
                moves = new XYSeries(move.getBegin() + "/" + move.getEnd());
                for (JointState js : JCasUtil.selectCovered(jCas,
                        JointState.class, move)) {
                    moves.add(js.getTime(),
                            JointStateUtils.getPosition(js, name));
                }
                xyDataset.addSeries(moves);
            }
        }

        JFreeChart chart = ChartFactory.createXYLineChart("Movements",
                "Time", "Position", xyDataset, PlotOrientation.VERTICAL,
                true, true, false);
        Stroke thickStroke = new BasicStroke(3.0f);
        for (int i = 1; i <= moveAnnotations.size(); i++) {
            chart.getXYPlot().getRenderer().setSeriesStroke(i, thickStroke);
        }

        return getChartPanel(chart);
    }

    private ChartPanel getMovementDirectionChart(String name) {
        XYSeriesCollection xyDataset = new XYSeriesCollection();

        XYSeries positions = new XYSeries("Position: " + name);
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            positions.add(js.getTime(), JointStateUtils.getPosition(js, name));
        }
        xyDataset.addSeries(positions);

        XYSeries posMoves;
        Collection<PositiveMovement> posMoveAnnotations = JCasUtil.select(jCas,
                PositiveMovement.class);
        for (PositiveMovement move : posMoveAnnotations) {
            if (move.getJointName().equals(name)) {
                posMoves = new XYSeries("Positive: " + move.getBegin() + "/"
                        + move.getEnd());
                for (JointState js : JCasUtil.selectCovered(jCas,
                        JointState.class, move)) {
                    posMoves.add(js.getTime(),
                            JointStateUtils.getPosition(js, name));
                }
                xyDataset.addSeries(posMoves);
            }
        }

        XYSeries negMoves;
        Collection<NegativeMovement> negMoveAnnotations = JCasUtil.select(jCas,
                NegativeMovement.class);
        for (NegativeMovement move : negMoveAnnotations) {
            if (move.getJointName().equals(name)) {
                negMoves = new XYSeries("Negative: " + move.getBegin() + "/"
                        + move.getEnd());
                for (JointState js : JCasUtil.selectCovered(jCas,
                        JointState.class, move)) {
                    negMoves.add(js.getTime(),
                            JointStateUtils.getPosition(js, name));
                }
                xyDataset.addSeries(negMoves);
            }
        }

        JFreeChart chart = ChartFactory.createXYLineChart(
                "Movement directions", "Time", "Position", xyDataset,
                PlotOrientation.VERTICAL, true, true, false);
        Stroke thickStroke = new BasicStroke(3.0f);
        for (int i = 1; i <= negMoveAnnotations.size(); i++) {
            chart.getXYPlot().getRenderer().setSeriesStroke(i, thickStroke);
        }

        return getChartPanel(chart);
    }

    private ChartPanel getAccelerationChart(String name) {
        XYSeries accelerations = new XYSeries("Joint Acceleration");
        int i;
        for (Acceleration acc : JCasUtil.select(jCas, Acceleration.class)) {
            i = JointStateUtils.getIndex(jCas, name);
            accelerations.add(acc.getBegin(), acc.getValue()
                    .get(i));
        }

        XYDataset xyDataset = new XYSeriesCollection(accelerations);
        JFreeChart chart = ChartFactory.createXYLineChart(
                "Acceleration", "Seq", "Acceleration", xyDataset,
                PlotOrientation.VERTICAL, true, true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getOscillationChart(String name) {
        XYSeriesCollection xyDataset = new XYSeriesCollection();

        XYSeries positions = new XYSeries("Position: " + name);
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            positions.add(js.getTime(), JointStateUtils.getPosition(js, name));
        }
        xyDataset.addSeries(positions);

        XYSeries oscillations;
        Collection<Oscillation> oscAnnotations = JCasUtil.select(jCas,
                Oscillation.class);
        for (Oscillation osc : oscAnnotations) {
            if (osc.getJointName().equals(name)) {
                oscillations = new XYSeries(osc.getBegin() + "/" + osc.getEnd());
                for (JointState js : JCasUtil.selectCovered(jCas,
                        JointState.class, osc)) {
                    oscillations.add(js.getTime(),
                            JointStateUtils.getPosition(js, name));
                }
                xyDataset.addSeries(oscillations);
            }
        }

        JFreeChart chart = ChartFactory.createXYLineChart("Oscillations",
                "Time", "Position", xyDataset, PlotOrientation.VERTICAL,
                true, true, false);
        Stroke thickStroke = new BasicStroke(3.0f);
        for (int i = 1; i <= oscAnnotations.size(); i++) {
            chart.getXYPlot().getRenderer().setSeriesStroke(i, thickStroke);
        }

        return getChartPanel(chart);
    }

    private ChartPanel getLimitChart(String name) {
        XYSeries upperLimit = new XYSeries("Upper limit");
        XYSeries lowerLimit = new XYSeries("Lower limit");
        XYSeries effort = new XYSeries("Effort limit");
        XYSeries velocity = new XYSeries("Velocity limit");
        int i;
        for (DistanceToLimit dis : JCasUtil.select(jCas, DistanceToLimit.class)) {
            i = EasyArray.find(dis.getJointNames(), name);
            if (i >= 0) {
                upperLimit.add(dis.getBegin(), dis.getUpperLimit(i));
                lowerLimit.add(dis.getBegin(), dis.getLowerLimit(i));
                effort.add(dis.getBegin(), dis.getEffort(i));
                velocity.add(dis.getBegin(), dis.getVelocity(i));
            }
        }

        XYSeriesCollection xyDataset = new XYSeriesCollection();
        xyDataset.addSeries(upperLimit);
        xyDataset.addSeries(lowerLimit);
        xyDataset.addSeries(effort);
        xyDataset.addSeries(velocity);
        JFreeChart chart = ChartFactory.createXYLineChart("Distance to limit",
                "Seq", "Limit", xyDataset, PlotOrientation.VERTICAL, true,
                true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getControllerPositionInputChart(String name) {
        XYSeries desiredPosition = new XYSeries("Desired position");
        XYSeries actualPosition = new XYSeries("Actual position");
        XYSeries errorPosition = new XYSeries("Error");
        int i;
        for (ControllerInput ci : JCasUtil.select(jCas, ControllerInput.class)) {
            i = EasyArray.find(ci.getJointNames(), name);
            if (i >= 0) {
                desiredPosition.add(ci.getBegin(), ci.getDesired()
                        .getPositions(i));
                actualPosition.add(ci.getBegin(), ci.getActual()
                        .getPositions(i));
                errorPosition.add(ci.getBegin(), ci.getError().getPositions(i));
            }

        }

        XYSeriesCollection xyDataset = new XYSeriesCollection();
        xyDataset.addSeries(desiredPosition);
        xyDataset.addSeries(actualPosition);
        xyDataset.addSeries(errorPosition);

        JFreeChart chart = ChartFactory.createXYLineChart(
                "Controller input position", "Seq", "Position", xyDataset,
                PlotOrientation.VERTICAL, true, true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getControllerVelocityInputChart(String name) {
        XYSeries desiredVelocity = new XYSeries("Desired velocity");
        XYSeries actualVelocity = new XYSeries("Actual velocity");
        XYSeries errorVelocity = new XYSeries("Error");
        int i;
        for (ControllerInput ci : JCasUtil.select(jCas, ControllerInput.class)) {
            i = EasyArray.find(ci.getJointNames(), name);
            if (i >= 0) {
                desiredVelocity.add(ci.getBegin(), ci.getDesired()
                        .getVelocities(i));
                actualVelocity.add(ci.getBegin(),
                        ci.getActual().getVelocities(i));
                errorVelocity
                        .add(ci.getBegin(), ci.getError().getVelocities(i));
            }

        }

        XYSeriesCollection xyDataset = new XYSeriesCollection();
        xyDataset.addSeries(desiredVelocity);
        xyDataset.addSeries(actualVelocity);
        xyDataset.addSeries(errorVelocity);

        JFreeChart chart = ChartFactory.createXYLineChart(
                "Controller input Velocity", "Seq", "Velocity", xyDataset,
                PlotOrientation.VERTICAL, true, true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getControllerAccelerationInputChart(String name) {
        XYSeries desiredAcceleration = new XYSeries("Desired acceleration");
        XYSeries actualAcceleration = new XYSeries("Actual acceleration");
        XYSeries errorAcceleration = new XYSeries("Error");
        int i;
        for (ControllerInput ci : JCasUtil.select(jCas, ControllerInput.class)) {
            if (ci.getDesired().getAccelerations().size() > 0) {
                i = EasyArray.find(ci.getJointNames(), name);
                if (i >= 0) {
                    desiredAcceleration.add(ci.getBegin(), ci.getDesired()
                            .getAccelerations(i));
                    actualAcceleration.add(ci.getBegin(), ci.getActual()
                            .getAccelerations(i));
                    errorAcceleration.add(ci.getBegin(), ci.getError()
                            .getAccelerations(i));
                }
            }
        }

        XYSeriesCollection xyDataset = new XYSeriesCollection();
        xyDataset.addSeries(desiredAcceleration);
        xyDataset.addSeries(actualAcceleration);
        xyDataset.addSeries(errorAcceleration);

        JFreeChart chart = ChartFactory.createXYLineChart(
                "Controller input acceleration", "Seq", "Acceleration",
                xyDataset, PlotOrientation.VERTICAL, true, true, false);

        return getChartPanel(chart);
    }

    private ChartPanel getErrorChart(String name) {
        XYSeriesCollection xyDataset = new XYSeriesCollection();

        XYSeries positions = new XYSeries("Position: " + name);
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            positions.add(js.getSeq(), JointStateUtils.getPosition(js, name));
        }
        xyDataset.addSeries(positions);

        XYSeries errors;
        Collection<ControllerError> errorAnnotations = JCasUtil.select(jCas,
                ControllerError.class);
        for (ControllerError error : errorAnnotations) {
            if (error.getJointName().equals(name)) {
                errors = new XYSeries(error.getBegin() + "/" + error.getEnd());
                for (JointState js : JCasUtil.selectCovered(jCas,
                        JointState.class, error)) {
                    errors.add(js.getSeq(),
                            JointStateUtils.getPosition(js, name));
                }
                xyDataset.addSeries(errors);
            }
        }

        JFreeChart chart = ChartFactory.createXYLineChart("Errors", "Seq",
                "Position", xyDataset, PlotOrientation.VERTICAL, true, true,
                false);
        Stroke thickStroke = new BasicStroke(3.0f);
        for (int i = 1; i <= errorAnnotations.size(); i++) {
            chart.getXYPlot().getRenderer().setSeriesStroke(i, thickStroke);
        }

        return getChartPanel(chart);
    }

    private ChartPanel getSelfCollisionPositionChart(String name) {
        XYSeriesCollection xyDataset = new XYSeriesCollection();

        XYSeries positions = new XYSeries("Position: " + name);
        for (JointState js : JCasUtil.select(jCas, JointState.class)) {
            positions.add(js.getSeq(), JointStateUtils.getPosition(js, name));
        }
        xyDataset.addSeries(positions);

        XYSeries collisions;
        Collection<SelfCollision> collisionsAnnotations = JCasUtil.select(jCas,
                SelfCollision.class);
        for (SelfCollision collision : collisionsAnnotations) {
            if (collision.getPerpetrator().equals(name)) {
                collisions = new XYSeries(collision.getVictim() + ": "
                        + collision.getBegin() + "/" + collision.getEnd());
                for (JointState js : JCasUtil.selectCovered(jCas,
                        JointState.class, collision)) {
                    collisions.add(js.getSeq(),
                            JointStateUtils.getPosition(js, name));
                }
                xyDataset.addSeries(collisions);
            }
        }

        JFreeChart chart = ChartFactory.createXYLineChart("SelfCollision",
                "Seq", "Position", xyDataset, PlotOrientation.VERTICAL, true,
                true, false);
        Stroke thickStroke = new BasicStroke(3.0f);
        for (int i = 1; i <= collisionsAnnotations.size(); i++) {
            chart.getXYPlot().getRenderer().setSeriesStroke(i, thickStroke);
        }

        return getChartPanel(chart);
    }

    private ChartPanel getChartPanel(JFreeChart chart) {
    	chart.getXYPlot().setBackgroundPaint(Color.white);
    	chart.getXYPlot().setRangeGridlinePaint(Color.gray);
        ChartPanel cp = new ChartPanel(chart);
        // cp.setSize(320, 240);
        cp.setMouseWheelEnabled(true);
        //cp.setBackground(new Color(255, 255, 255));  
        return cp;
    }
}
