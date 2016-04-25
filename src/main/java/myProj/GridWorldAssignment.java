
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
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
import burlap.oomdp.auxiliary.common.SinglePFTF;
import burlap.oomdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.GoalBasedRF;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;


public class GridWorldAssignment{

    public static void main(String [] args){

        GridWorldDomain gw = new GridWorldDomain(11,11); //11x11 grid world
        gw.setMapToFourRooms(); //four rooms layout
        gw.setProbSucceedTransitionDynamics(0.8); //stochastic transitions with 0.8 success rate
        Domain domain = gw.generateDomain(); //generate the grid world domain

        //setup initial state
        State s = GridWorldDomain.getOneAgentOneLocationState(domain);
        GridWorldDomain.setAgent(s, 0, 0);
        GridWorldDomain.setLocation(s, 0, 10, 10);

        final TerminalFunction tf = new SinglePFTF(domain.
                getPropFunction(GridWorldDomain.PFATLOCATION));

        //reward function definition
        final RewardFunction rf = new GoalBasedRF(new TFGoalCondition(tf), 5., -0.1);
        //initial state generator
        final ConstantStateGenerator sg = new ConstantStateGenerator(s);
        //set up the state hashing system for looking up states
        final SimpleHashableStateFactory hashingFactory = new SimpleHashableStateFactory();

        Environment env = new SimulatedEnvironment(domain, rf, tf, s);
        LearningAgent agent = new QLearning(domain, 0.5, hashingFactory, 0.1, .5);


		runVisualizationIterationsValue(domain,s,hashingFactory,rf,tf,25,25,1);
		runVisualizationIterationsPolicy(domain,s,hashingFactory,rf,tf,5,6,1);

		writeVITrialToFile(domain,rf,tf,hashingFactory,s,10,100,"/home/harry/gtech/ml/hw4/javaproj/results/vi/results.csv");
        writeQLTrialToFile(domain,rf,tf,hashingFactory,s,300,100,"/home/harry/gtech/ml/hw4/javaproj/results/ql/results.csv");
		writePITrialToFile(domain,rf,tf,hashingFactory,s,10,100,"/home/harry/gtech/ml/hw4/javaproj/results/pi/results.csv");


    }

    public static void simpleValueFunctionVis(ValueFunction valueFunction, Policy p, Domain domain, SimpleHashableStateFactory hashingFactory, State initialState){

        List<State> allStates = StateReachability.getReachableStates(initialState, (SADomain)domain, hashingFactory);
        ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(allStates, valueFunction, p);
        gui.initGUI();

    }

    public static void writeVITrialToFile(Domain domain,RewardFunction rf, TerminalFunction tf, SimpleHashableStateFactory hashingFactory, State s, Integer maxIter,Integer numTrials, String path){
        try {
            String sFileName = path;
            FileWriter writer = new FileWriter(sFileName);
            for(int t = 1; t <= numTrials; t+=1){

                for (int i = 1; i <= maxIter; i += 1) {
                    long startTime = System.nanoTime();
                    Planner plan = new ValueIteration(domain, rf, tf,.9, hashingFactory, .001, i);
                    Policy p = plan.planFromState(s);
                    EpisodeAnalysis ea = p.evaluateBehavior(s, rf, tf);
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

    public static void writePITrialToFile(Domain domain,RewardFunction rf, TerminalFunction tf, SimpleHashableStateFactory hashingFactory, State s, Integer maxIter,Integer numTrials, String path){
        try {
            String sFileName = path;
            FileWriter writer = new FileWriter(sFileName);
            for(int t = 1; t <= numTrials; t+=1){

                for (int i = 1; i <= maxIter; i += 1) {
                    long startTime = System.nanoTime();
                    Planner plan = new PolicyIteration(domain, rf, tf, 0.99, hashingFactory, .001, 1, i);
                    Policy p = plan.planFromState(s);
                    EpisodeAnalysis ea = p.evaluateBehavior(s, rf, tf);
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


            for (double lr = .0; lr <=1.;lr+=.1){

                for(int t = 1; t <= numTrials; t+=1) {

                    Environment env = new SimulatedEnvironment(domain, rf, tf, s);
                    LearningAgent agent = new QLearning(domain, lr, hashingFactory, 0.1, .5);
                    long startTime = System.nanoTime();
                    //run learning for 50 episodes
                    for (int i = 0; i < maxIter; i++) {
                        EpisodeAnalysis ea = agent.runLearningEpisode(env);
                        writer.append(Integer.toString(i));
                        writer.append(',');
                        writer.append(Integer.toString(ea.numTimeSteps()));
                        writer.append(',');
                        writer.append(Long.toString((System.nanoTime() - startTime) / 1000000));
                        writer.append(',');
                        writer.append(Double.toString(lr));
                        writer.append('\n');
                        env.resetEnvironment();
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
