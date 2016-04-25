package myProj;

import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.blockdude.BlockDude;
import burlap.domain.singleagent.blockdude.BlockDudeLevelConstructor;
import burlap.domain.singleagent.blockdude.BlockDudeTF;
import burlap.domain.singleagent.blockdude.BlockDudeVisualizer;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.GoalBasedRF;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
import burlap.oomdp.visualizer.Visualizer;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class BlockDudeAssignment{

    public static void main(String [] args){
        BlockDude bd = new BlockDude();
        Domain domain = bd.generateDomain();
        State s = BlockDudeLevelConstructor.getLevel3(domain);
        TerminalFunction tf = new BlockDudeTF();
        RewardFunction rf = new UniformCostRF();
        SimpleHashableStateFactory hashingFactory = new SimpleHashableStateFactory();




        writeVITrialToFile(domain,rf,tf,hashingFactory,s,120,5,1,"/home/harry/gtech/ml/hw4/javaproj/results/vi/results_bd.csv");
		writePITrialToFile(domain,rf,tf,hashingFactory,s,120,5,1,"/home/harry/gtech/ml/hw4/javaproj/results/pi/results_bd.csv");
        writeQLTrialToFile(domain,rf,tf,hashingFactory,s,350000,1,"/home/harry/gtech/ml/hw4/javaproj/results/ql/results_bd.csv");

//        BlockDude bd = new BlockDude();
//        Domain domain = bd.generateDomain();
//        State s = BlockDudeLevelConstructor.getLevel3(domain);
//
//        TerminalFunction tf = new BlockDudeTF();
//        RewardFunction rf = new UniformCostRF();
//        final SimpleHashableStateFactory hashingFactory = new SimpleHashableStateFactory();
//
//
//        Visualizer v = BlockDudeVisualizer.getVisualizer(bd.getMaxx(), bd.getMaxy());
//
//
//        VisualExplorer exp = new VisualExplorer(domain, v, s);
//
//        exp.addKeyAction("w", "up");
//        exp.addKeyAction("d", "east");
//        exp.addKeyAction("a", "west");
//        exp.addKeyAction("s", "pickup");
//        exp.addKeyAction("x", "putdown");
//
//        exp.initGUI();
//        }

    }

    public static void simpleValueFunctionVis(ValueFunction valueFunction, Policy p, Domain domain, SimpleHashableStateFactory hashingFactory, State initialState){

        List<State> allStates = StateReachability.getReachableStates(initialState, (SADomain)domain, hashingFactory);
        ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(allStates, valueFunction, p);
        gui.initGUI();

    }

    public static void writeVITrialToFile(Domain domain,RewardFunction rf, TerminalFunction tf, SimpleHashableStateFactory hashingFactory, State s, Integer maxIter,Integer skipIter, Integer numTrials, String path){
        try {
            String sFileName = path;
            FileWriter writer = new FileWriter(sFileName);
            for(int t = 1; t <= numTrials; t+=1){
                System.out.println("t = " + t);

                for (int i = 1; i <= maxIter; i += skipIter) {
                    System.out.println("i = " + i);
                    long startTime = System.nanoTime();
                    Planner plan = new ValueIteration(domain, rf, tf,.99, hashingFactory, .05, i);
                    Policy p = plan.planFromState(s);
                    EpisodeAnalysis ea = p.evaluateBehavior(s, rf, tf,1000);
                    writer.append(Integer.toString(i));
                    writer.append(',');
                    writer.append(Integer.toString(ea.numTimeSteps()));
                    writer.append(',');
                    writer.append(Long.toString((System.nanoTime() - startTime) / 1000000));
                    writer.append('\n');
                    System.out.println("ea.numTimeSteps() = " + ea.numTimeSteps());


                }
            }

            writer.flush();
            writer.close();
        }
        catch (IOException var26) {
            var26.printStackTrace();
        }
    }

    public static void writePITrialToFile(Domain domain,RewardFunction rf, TerminalFunction tf, SimpleHashableStateFactory hashingFactory, State s, Integer maxIter,Integer skipIter, Integer numTrials, String path){
        try {
            String sFileName = path;
            FileWriter writer = new FileWriter(sFileName);
            for(int t = 1; t <= numTrials; t+=1){

                for (int i = 1; i <= maxIter; i += skipIter) {
                    long startTime = System.nanoTime();
                    Planner plan = new PolicyIteration(domain, rf, tf, 0.99, hashingFactory, .001, 1, i);
                    Policy p = plan.planFromState(s);
                    EpisodeAnalysis ea = p.evaluateBehavior(s, rf, tf,1000);
                    writer.append(Integer.toString(i));
                    writer.append(',');
                    writer.append(Integer.toString(ea.numTimeSteps()));
                    writer.append(',');
                    writer.append(Long.toString((System.nanoTime() - startTime) / 1000000));
                    writer.append('\n');


                }
            }

            writer.flush();
            writer.close();
        }
        catch (IOException var26) {
            var26.printStackTrace();
        }
    }

    public static void writeQLTrialToFile(Domain domain,RewardFunction rf, TerminalFunction tf, SimpleHashableStateFactory hashingFactory, State s, Integer maxIter,Integer numTrials, String path){
        try {
            String sFileName = path;
            FileWriter writer = new FileWriter(sFileName);



            for(int t = 1; t <= numTrials; t+=1) {

                Environment env = new SimulatedEnvironment(domain, rf, tf, s);
                LearningAgent agent = new QLearning(domain, .99, hashingFactory, 0.1, .05);
                long startTime = System.nanoTime();
                //run learning for 50 episodes
                for (int i = 0; i < maxIter; i++) {
                    EpisodeAnalysis ea = agent.runLearningEpisode(env,400);
                    writer.append(Integer.toString(i));
                    writer.append(',');
                    writer.append(Integer.toString(ea.numTimeSteps()));
                    writer.append(',');
                    writer.append(Long.toString((System.nanoTime() - startTime) / 1000000));
                    writer.append('\n');
                    env.resetEnvironment();
                    if(i%10000==0){
                        System.out.println("i = " + i);
                    }
                }
            }
            writer.flush();
            writer.close();
        }
        catch (IOException var26) {
            var26.printStackTrace();
        }
    }

    public static void runVisualizationIterationsValue(Domain domain,State s,SimpleHashableStateFactory hashingFactory,RewardFunction rf,TerminalFunction tf, Integer start, Integer max, Integer step){
        for(int i = start;i<=max;i+=step){
            Planner plan = new ValueIteration(domain, rf, tf, .99,hashingFactory,.001, i);
            Policy p =plan.planFromState(s);
            manualValueFunctionVis((ValueFunction)plan, p,domain,s,hashingFactory,Integer.toString(i));
        }

    }
    public static void runVisualizationIterationsPolicy(Domain domain,State s,SimpleHashableStateFactory hashingFactory,RewardFunction rf,TerminalFunction tf, Integer start, Integer max, Integer step){
        for(int i = start;i<=max;i+=step){
            Planner plan = new PolicyIteration(domain, rf, tf, 0.99, hashingFactory, -1, 1, i);
            Policy p =plan.planFromState(s);
            manualValueFunctionVis((ValueFunction)plan, p,domain,s,hashingFactory,Integer.toString(i));
        }

    }



    public static void manualValueFunctionVis(ValueFunction valueFunction, Policy p, Domain domain, State initialState,SimpleHashableStateFactory hashingFactory ,String title){

        List<State> allStates = StateReachability.getReachableStates(initialState, (SADomain)domain, hashingFactory);

        //define color function
        LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
        rb.addNextLandMark(0., Color.RED);
        rb.addNextLandMark(1., Color.BLUE);

        //define a 2D painter of state values, specifying which attributes correspond to the x and y coordinates of the canvas
        StateValuePainter2D svp = new StateValuePainter2D(rb);
        svp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX,
                GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);


        //create our ValueFunctionVisualizer that paints for all states
        //using the ValueFunction source and the state value painter we defined
        ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, valueFunction);

        //define a policy painter that uses arrow glyphs for each of the grid world actions
        PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
        spp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX,
                GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);
        spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONNORTH, new ArrowActionGlyph(0));
        spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONSOUTH, new ArrowActionGlyph(1));
        spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONEAST, new ArrowActionGlyph(2));
        spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONWEST, new ArrowActionGlyph(3));
        spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);


        //add our policy renderer to it
        gui.setSpp(spp);
        gui.setPolicy(p);
        gui.setTitle(title);
        //set the background color for places where states are not rendered to grey
        gui.setBgColor(Color.GRAY);

        //start it
        gui.initGUI();


    }

//	public static void writeToCSV(){
//		try {
//			BufferedReader instances = new BufferedReader(new FileReader(new File("src/opt/test/attributes.csv")));
//
//			for(i = 0; i < attributes.length; ++i) {
//				Scanner scan = new Scanner(instances.readLine());
//				scan.useDelimiter(",");
//				attributes[i] = new double[2][];
//				attributes[i][0] = new double[27];
//				attributes[i][1] = new double[1];
//
//				for(int j = 0; j < 27; ++j) {
//					attributes[i][0][j] = Double.parseDouble(scan.next());
//				}
//
//				attributes[i][1][0] = Double.parseDouble(scan.next());
//			}
//		} catch (Exception var5) {
//			var5.printStackTrace();
//		}
//	}




}