package robotTA.gui;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class GuiMain {

    public static void main(String[] args) {

        JFrame mainFrame = new JFrame("Trajectory Analyzer");
        mainFrame.setSize(600, 400);

        String dbAddress = getDBAddress(mainFrame);

        JPanel panel = new JPanel();

        // Add label
        JLabel description = new JLabel("Lorem Ipsum " + dbAddress);
        panel.add(description);
        panel.add(getChart());

        mainFrame.add(panel);
        

        mainFrame.setVisible(true);

    }

    private static String getDBAddress(JFrame parent) {
        return JOptionPane.showInputDialog(parent, "Database address",
                "localhost");
    }

    private static ChartPanel getChart() {
        XYSeries Goals = new XYSeries("Goals Scored");
        Goals.add(1, 1.0);
        Goals.add(2, 3.0);
        Goals.add(3, 2.0);
        Goals.add(4, 0.0);
        Goals.add(5, 3.0);
        XYDataset xyDataset = new XYSeriesCollection(Goals);
        JFreeChart chart = ChartFactory.createXYLineChart(
                "Goals Scored Over Time", "Fixture Number", "Goals", xyDataset,
                PlotOrientation.VERTICAL, true, true, false);
        ChartPanel cp = new ChartPanel(chart);
        cp.setSize(320, 240);
        cp.setMouseWheelEnabled(true);
        
        return cp;
    }
}
