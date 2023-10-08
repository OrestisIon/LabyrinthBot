/**
 * In this exercise I made the choice to create a tree and store each Junction as a node of the tree.
 * Each node of the tree has the ability to have three children nodes. One child for east, one child for south and one
 * child for west. The first junction that is met it will be stored as a junction with null parent. Then the next junction
 * will be a child of the current junction. The type of child is decided from the leave property of the current junction
 * which shows what was the heading of the robot when it left the junction. So the child junction created will become
 * current junction with parent as the previous junction.
 *
 * If a junction leads to  dead-ends only then the robot will delete the current junction by setting it to null and
 * set its parent junction as the current junction , in order to explore other paths of the parent junction or leave from the
 * parent junction too if it leads to only dead-ends as well.
 *
 * That way when the robot reaches the target for the first time , there is a tree created, that is the path of the robot
 * to reach the target. Each node of the tree has exactly one child that leads to another node that will eventually lead to
 * the target without getting into any dead ends.
 *
 * In order for the robot to be able to find the first junction of the tree we also store the first heading that of the robot.
 * Because if the robot is not at a junction initially and instead it's at a corridor, when it runs for the second time it
 * won't know what direction to take in order to reach the first junction(node of the tree).
 * The next step in the implementation of this exercise is to think a way for the robot to work in loopy mazes too.
 * Having loops in the junction tree will cause many problems, thus I decided to change the record junction and make it so
 * it returns a Boolean value(true when the robot has just done a loop, or false if it has not done a loop). If the robot has
 * just done a loop then it doesn't record that junction again and backs to the last recorded junction.
 * Like that the we will have only planning trees without any cycles created.
 *
 * I specially chose to create a tree structure because it represents exactly the map that the movement of the robot creates.
 * Maybe I could have used a stack or two dimensional arrays but creating those wouldn't really be as representative as the tree
 * structure is. By creating the exact representation of the robot’s movement then I can more easily adapt the code into new
 * requirements that the customer may have.
 *
 * Testing: Testing is important in this exercise because not only of the length of the code but also because of the increased
 * number of properties that each junction has. Testing the code helped me find scenarios that my robot will not work
 * and will get stuck and modify the existing cod in order to work in every possible situation. One of those situations was
 * when the target is behind the robot in a dead-end and the first move of the robot is outside the dead end. My program
 * initially did not have anything to address that. When it backtracked into the first junction it just got stacked and couldn't
 * move any further. So, I added a method that returns -1 if it's not the first junction and it returns the reverse
 * of the first heading of the robot if it's the first junction that it backtracks to. That heading is also stored as the
 * heading of the robot, since there is no way of going into the first junction again since that leads to dead ends only.
 * So in the second run it will immediately reach the target with just the first heading.
 *
 * Last words: This algorithm works perfectly in a prim generated Junction, as in this kind of mazes the only path that leads to
 * the target is also the quickest. Also the it works fairly good on the hill generator too.When it comes to loopy mazes though,
 * even though the robot successfully finds the target faster than the first time, the difference is not great. This is because in loopy mazes
 * there are many ways for the robot to reach the target and some paths are longer than other paths. A possible solution for that problem
 * is for the robot explore all the maze before reaching the target and then in the second time to calculate and follow the shortest path of
 * all the paths. Lastly on blank mazes even tho the robot finds an optimal path in comparison with the first run, it is
 * not very noticable that the second time goes faster, because in a blanc maze there aren't any dead-ends for the robot to directly
 * back up and know to not take again that direction that leads to the dead-end
 */


import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.ArrayList;


public class Robot{
    private int pollRun = 0;  // Incremented after each pass
    private static RobotData robotData; // Data store for junctions
    public int[] passExits= new int[4]; //declaring a global array- maximum 3 exits each time are passages since it has just come out from the one
    //Prefered to ignore the Exercise, since a boolean value is much more suited fo this variable
    public boolean backtrackMode=false;
    public int firstheading;//the robot's first heading
    public boolean isLoop=false;

    /**
     * The main method where it initializes the robotData variable if it's the first run, it generates the next direction of the robot by using
     * other methods and it then faces the robot to that direction
     * @param robot
     */
    public void controlRobot(IRobot robot) {
        int direction=0, exits,heading=0;

        // On the first move of the first run of a new maze

        exits=nonwallExits(robot);//using a method to store in the exits variable the number of available exits
        if(robot.getRuns() == 0){ //if it's the first run of the robot
            if ( (pollRun == 0)) {
                robotData = new RobotData(); //reset the data store
                backtrackMode=false;
                firstheading=0;
            }

            if(exits<3) { //if at a corridor or dead-end
                direction=mainMove(robot, exits); // call the method of the standard moving
               if(pollRun==0){//if it's the robot's first move and is also in a corridor or dead-end then
                firstheading=directiontoHeading(robot.getHeading(),direction);// store the heading of the robot as the in a global variable as the  firstheading
               }
            }
            else {
                if(backtrackMode==false) { //if in the explorer mode
                    isLoop=robotData.recordJunction(robot.getLocation().x,robot.getLocation().y, robot.getHeading(), beenbeforeExits(robot)); //call the function to check if junction is new and  store it
                }
                if(isLoop){//if the junction has just done a loop
                    direction=loop();//calling the loop method to give next direction
                }
                else if(passageExits(robot)==0){ //if it's a junction and the is no unexplored path then
                    direction=backtrackControl(robot);//switch to backtrack(back up)
                }
                else { //else if there are still unexplored paths
                    direction= exploreControl(robot);//switch to the explorer mode
                }
            }
        }
        else{ //if it's not the first run of the robot
           direction=secondRun(robot,exits);
        }
        robot.face(direction);
        pollRun++; // Increment pollRun so that the data is not reset each time the robot moves
    }

    public int secondRun(IRobot robot, int exits){
       int heading=0;
        if(pollRun==0 && firstheading!=0 ){ //and it's the first move of the robot, and a first heading exists(means that the robot was not initially in a junction)
            robot.setHeading(firstheading); //then set the first heading stored in a variable from the first run as the next heading of the robot
           return IRobot.AHEAD; //and move ahead to that heading
        }
        else { //and if it's not the first move of the robot
            if (exits < 3) { //and the robot is at a corridor or dead-end
                return mainMove(robot, exits); // call the method of the standard moving
            } else {
                heading = robotData.path();//call the path method to give the next heading
                robot.setHeading(heading);
                return IRobot.AHEAD; //and move ahead to that heading
            }
        }
    }

    /**
     *The method used to calculate the next robot's direction for every time the number of exits around the robot is less than 3
     * If there are only 2 exits it means that it is in a cosrridor and calls a method for that and if there is only 1 available exit
     * it calls a method for a dead-end
     * @param robot
     * @param exits
     * @return the direction that the robot will head to next
     */
    private int mainMove(IRobot robot, int exits){
        int direction=IRobot.AHEAD;
        if(exits==1){ //if it's a dead-end
            direction=deadEnd(robot);
        }
        else if(exits==2) //if it's a corridor
            direction=corridor(robot);

        return direction;
    }

    /**
     *
     * @param robot
     * @return the direction generated by the junction method-which explores a junction
     */
    private int exploreControl(IRobot robot) {
        int direction,heading;
        backtrackMode=false; //free the ability to create new junctions next because the robot is exploring
        direction=junction(robot);//call the junction function to decide which junction path to follow
        heading=directiontoHeading(robot.getHeading(),direction);//using method to get the heading of the new direction generated
        robotData.leaveDirection(heading); //store the heading that will leave the junction
        return direction;
    }
    /**
     * Gives the heading of a robot's direction
     * @param robotHeading is what heading the robot currently has
     * @param direction is the direction of the robot(IRobot.AHEAD,IRobot.BEHIND,IRobot.LEFT,IRobot.RIGHT) which we want to find its heading
     * @return the calculated heading of the given robot's direction
     */
    public int directiontoHeading(int robotHeading,int direction){
        int y,x,heading=0;
       x=robotHeading;
       y=direction-IRobot.AHEAD;
        switch(x){
        case IRobot.NORTH:
                heading=x+y;
                break;
            case IRobot.SOUTH:
                if(y>=2)
                    heading=IRobot.NORTH+(y-2);
                else heading=x+y;
                break;
            case IRobot.EAST:
                if(y==3)
                    heading=IRobot.NORTH;
                else
                    heading=x+y;
                break;
            case IRobot.WEST:
                if(y==0)
                    heading=x;
                else
                heading=IRobot.NORTH+(y-1);
        }
        return heading;
    }

    /**
     * This method calls another method to get the heading of the robot when it got for the first time in the junction that it currently is.
     * Then it reverses that heading since the aim is to get out of the junction from the exit that it came from the first time(It's the exact opposite direction that
     * it first came). Then it sets the current heading to the newly generated one.
     * Moreover it calls the isitEnd method to check if the robot is backtracking to the first junction, which means that the target is
     * @param robot
     * @return direction to go forward to the newly set heading
     */
    private int backtrackControl(IRobot robot){
        int heading, heading2;
        backtrackMode=true;
        //If no unexplored head to where the robot came from in the junction
        heading =robotData.getJunctionHeading(robot.getLocation().x, robot.getLocation().y);//return the heading to where it come from the first time in this junction
        //getting the reverse of that heading
        robot.setHeading(heading);//set heading of the robot to go where it first came from
        if(robotData.isitEnd(firstheading)!=-1)
            firstheading=robotData.isitEnd(firstheading);
        return IRobot.AHEAD;//set direction of the robot ahead to the new heading
    }

    private int loop(){
        backtrackMode=true; //turn on the backtrack mode
        isLoop=false; //set the boolean variable false(initializing it for the next move)
        return IRobot.BEHIND; //it's a cycle so go back, since we don't want cycles
    }



    /**
     * The method that takes any integer  and calculates a random integer between 0  and that number included.
     * @param The highest boundary of the random integer generated
     * @return the random integer that was just calculated.
     */
    private int random( int n ){
        int rando=0;
        rando= (int)(Math.random()*n); //method for generating a random number
        return rando;
    }

    /**
     * Generates a random direction around the robot that does not lead the robot to a wall
     * @param robot
     * @return the next direction of the robot
     */
    private int gotonotwall(IRobot robot){
        int direction=IRobot.AHEAD,rando=0;
        do{
            // Select a random number
            rando = random(4);
            // Convert this to a direction
            switch (rando) {
                case 0:
                    direction = IRobot.LEFT;
                    break;
                case 1:
                    direction = IRobot.RIGHT;
                    break;
                case 2:
                    direction = IRobot.BEHIND;
                    break;
                default:
                    direction = IRobot.AHEAD;
            }
        }while(robot.look(direction)==IRobot.WALL); //if the random direction chosen is leading to a wall then go back and choose direction again
        return direction;
    }

    /**
     * Calculates the number of exits around the robot that do not lead to a wall
     * @param robot
     * @return the total number of exits that don't lead to a wall
     */
    private int nonwallExits (IRobot robot) {  // Your new method
        int nonwalls=0;
        for(int i=0; i<=3; i++){ //checking all the directions
            if(robot.look(IRobot.AHEAD+i)!=IRobot.WALL)//if current diection leads to a wall
                nonwalls++; //increment the counter varible
        }
        return nonwalls;

    }

    /**
     * Calculates the number of available unexplored passages around the robot. Also stores the directions of these
     * passages in a global array
     * @param robot
     * @return the number of unexplored passages
     */
    private int passageExits(IRobot robot) {
        int counter=0;
        for(int i=0; i<=3; i++){ //checking all the directions
            if(robot.look(IRobot.AHEAD+i)==IRobot.PASSAGE){//if current direction leads to a passage
                passExits[counter]=IRobot.AHEAD+i; //storing to a global array the direction that leads to a passage
                counter++;} //increament the counter variable
        }
        return counter;
    }

    /**
     * Calculates the number of exits around the robot that it has already been there before
     * @param robot
     * @return the number of been before exits that surround the robot
     */
    private int beenbeforeExits(IRobot robot ){
        int counter=0;
        for(int i=0; i<=3; i++) { //checking all the directions
            if (robot.look(IRobot.AHEAD + i) == IRobot.BEENBEFORE) {//if current direction leads to a been before square
                counter++;//increament the counter variable
            }
        }
        return counter-1; //minus one- which is the one it just came from
    }

    /**
     *Method called when the robot's at a dead-end. In this case it just calls the method that will give the direction
     * that will not lead the robot to a wall
     * @param robot
     * @return the next direction of the robot
     */
    private int deadEnd (IRobot robot) {
        backtrackMode=true;//indicating that the robot is going back to last junction, so no need to store new ones
        return gotonotwall(robot);
    }

    /**
     *Generates the next direction of the robot when it is in a corridor. Will move direct the robot to the only direction that
     * there is no wall and it is not behind.
     * @param robot
     * @return the next direction of the robot
     */
    private int corridor (IRobot robot) {
        int direction,i=0;
        do{
            direction=IRobot.AHEAD+i;
            i++;
        }while((robot.look(direction)==IRobot.WALL || direction==IRobot.BEHIND)); //Choose the direction that doesn't lead to a wall and doesn't go back
        return direction;
    }

    /**
     *It chooses randomly one of the unexplored exits and generates the direction that will lead the robot to that exit.
     * If the available exit is only one then it just generates the direction to go to that exit
     * @param robot
     * @return the direction that will lead the robot to one of the unexplored exits that are not walls
     */
    private int junction (IRobot robot) {
        int rando=0;
        int passagesNumber=0;
        passagesNumber=passageExits(robot); //storing the number of number of passages in the
        rando=random(passagesNumber); //generate a random number between 0 and the number of passages minus one
        return passExits[rando];
    }
    /**
     * Used every time the reset button in the Robot maze interface is pressed in oreder to initialize the values used for a new round
     */
    public void reset() {
        robotData.resetJunctionCounter();

        pollRun = 0;//maybe not needed
    }

}
/**
 * The class that creates a tree of Junction. Provides a method to add a node(junction) to the tree, one to print the current node,
 * one that returns the heading to leave backtrack in the previous node and deletes the current one. It also provides methods useful
 * after the second run in order to find the heading of the path that leads to the target. Finally there is a method that checks when
 * the robot has done a loop
 */
class RobotData {
    private int junctionCounter; // No. of junctions stored
    public    Junction junc;
    private   Junction currentJunc;
    public   int x,y; //used to store x and y coordinates of a junction that we want to use in more that one methods
    public Junction firstJunc; //stores the first Junction
    private Junction target;//this is the variable used for finding the path in after the first run. It is declared global as we don't want it to reset every time the method is called
    public int pathCounter; //the equivalent junction counter but after the second round used for the path

    /**
     * The constructor of the RobotData class that inistializes the variables that need to be initialized
     */
    public RobotData() {
        junctionCounter = 0;
        pathCounter=0;
        target=null;
    }


    /**
     * Method called to record a Junction by adding it to the tree. It only records a junction if it's not a cycle, which is
     * check using the isCycle method. Also if it's the first Junction it creates the first tree node with null parent
     * @param x coordinate of the robot
     * @param y coordinate of the robot
     * @param arrived is the dirrection that the robot had when it  arrived at the junction(the current direction of the robot )
     */
    public boolean recordJunction(int x, int y, int arrived, int beenbefore) {
        Junction first;
        this.x=x;
        this.y=y;
        if (junctionCounter != 0) { //if it's not the first junction that the robot meets
            if (beenbefore > 1) { //if there are exits of that junction that have been visited before then
                first=getFirstJunc();//using the getFirstJunc metjod assigns the first junction to the variable first
                if(isCycle(first)) //if it's a cycle don't store it
                    return true; //and return true
            }
            //if it's not a cycle then create a child store it as the current junction
            junc=junc.creatChild(x, y,arrived, junc.getLeave(),junc);
            junctionCounter++; //increment the junction counter
            printArriveJunction();
        }
        else{ // else if it's the first junction that the robot meets
            junc= new Junction(x,y,arrived,null);//create the first node of the Junction tree without a parent(null)
            printArriveJunction();
            junctionCounter++;//increment junction counter
        }
        return false; //returns that the junction was not a cycle

    }


    /**
     *Responsible in finding the next heading of the robot in order to follow the path that is already made from the first run
     * @return the next direction that will follow the path
     */
    public int path(){
       int heading;
        if (pathCounter==0){// if it's the first junction of the path
            target=getFirstJunc(); //set the target variable as the first node
            //printTree(target); //print it to ensure that is correct

        }
        heading=target.getLeave();//Store in the heading variable the leave heading that is stored in the current junction(the last heading that the robot had when left the junction)
        switch (target.getLeave()){ //Find which is the next child from value of the leave heading of the current junction
            //and set that child as the current junction(the junction that the robot will head next)
            case IRobot.NORTH:
                    target=target.getcN();
                    break;
                case IRobot.SOUTH:
                    target=target.getcS();
                    break;
                case IRobot.EAST:
                   target=target.getcE();
                    break;
                case IRobot.WEST:
                    target=target.getcW();
        }
        pathCounter++;//increment path counter
        return heading;//return the heading that was just calculated
    }

    /**
     * This method check if the robot has done a loop. If there is a node in the with the same coordinates as the junction
     * that the robot is right now, it means that is a cycle(loop). It goess through every existing node of the tree and compares
     * it's coordinates with the robots current coordinates. If they match it imediatly returns true else it continoues tp
     * compare until the all the nodes have been compared. In order to search for the whole tree this is a recursive algorithm
     * to traverse the tree.
     * @param junc
     * @return true if that junction already exists
     */
    public boolean isCycle(Junction junc){
        boolean isfound=false;//initialiaze the boolean variable to false, that means that still haven't found the junction
        if(junc.getX()==x && junc.getY()==y )//if the coordinates of the current node matches the one that we search for
            return true;//return true
        if(junc.getcN()!=null&& isfound==false)//if there is a child node north th and the junction still hasn't been found
            isfound=isCycle(junc.getcN());//search if it matches that child next
        if(junc.getcE()!=null&& isfound==false)//if there is a child node east and the junction still hasn't been found
            isfound=isCycle(junc.getcE());
        if(junc.getcS()!=null && isfound==false)
            isfound=isCycle(junc.getcS());
        if(junc.getcW()!=null && isfound==false)
            isfound=isCycle(junc.getcW());

        return isfound;//return the boolean value of is found variable-which is true if is found and false if the junction recorded is not in the tree
    }


    /**
     * Method that turns a heading into its string value
     * @param heading
     * @return the given heading as a string
     */
    public String headingtoString(int heading) {
        switch (IRobot.WEST - heading) { //**VERY IMPORTANT--AKIRA TELIKA TOUTA APLA AFERESTA VLAKA**//
            case 0:
                return "WEST";//HERE WE DON'T WANT THE DIRECTION THAT THE ROBOT IS HEADING TO
            case 1:
                return "SOUTH";//THAT'S WHY WE CONVERT TO STRING THE EXACT OPOSSITE DIRECTION THAT THE ROBOT IS HEADING TO
            case 2:
                return "EAST";//SINCE WE WANT THE ONE THAT HAS ARRIVED FROM
            default:
                return "NORTH";
        }
    }

    /**
     * Will adjust the Junction data for the robot to backtrack in the current junction. If it's the first junction that
     * is backtracking in then set Junction Counter to -1 which means that the whole tree is deleted. If not then
     * delete current junction from the tree and make its parent node the current one. Also return the heading that will
     * lead the robot back to tje new current junction.
     * @param x coordinate that the robot is currently
     * @param y coordinate that the robot is currently
     * @return the reverse heading that the robot had when it first arrived in that junction
     */
    public int getJunctionHeading(int x, int y ) {
        int heading,direction;
        heading= junc.getArrived();
       if(junc.getParent()!=null) { //if it's not backtracking in the first junction
           //need to make previous junction the current one and delete the other
           junc = junc.getParent();
           junc.deleteJunction();//deleting this junction since it leads to nowhere
       }
       else{junctionCounter=-1;} //if the robot is backtracking in the first junction set the junctionCounter to -1 because the whole tree is deleted
        if (heading >= IRobot.SOUTH)
            heading = heading - 2;
        else heading = heading + 2;
        return heading;
    }

    /**
     * Storing in the current node the heading that the robot had when it left the current junction
     * @param heading
     */
    public void leaveDirection(int heading){
        junc.setLeave(heading);
    }

    /**
     * Generates the first node of the whole tree(The parent)
     * @return the first junction of the tree
     */
    public Junction getFirstJunc(){
        Junction junc1;
        junc1=junc;
        while(junc1.getParent()!=null){//If there is a parent of that node
            junc1= junc1.getParent();} //move to the parent node

        return junc1; //return the node that doesn't have a parent- means that is the ultimate parent node
    }

    /**
     *Checks if the tree has been deleted(Which means that the robot has backtracked in the first junction). If yes then the first
     * heading is being reversed.
     * @param firstHeading the  heading of the robot when it first arrived in the first junction
     * @return the new first heading if it was reversed in this method, or -1 if nothing happened because the robot has not backtracked in
     * the first junction
     */
    public int isitEnd(int firstHeading){
        if(junctionCounter==-1){ //if the tree was deleted
            //reverse the first heading
            if (firstHeading >= IRobot.SOUTH)
                firstHeading = firstHeading - 2;
            else firstHeading = firstHeading + 2;
        }
        else{return -1;} //if it is the first heading return -1
        return firstHeading;
    }


    /**
     *This method prints the last created Junction instance
     */
    public void printArriveJunction() {
        System.out.println("Junction "+(junctionCounter-1)+" (x=" + junc.getX() + ",y=" +junc.getY() + ") heading " + headingtoString(junc.getArrived()));
    }

    /**
     *The method used when the robot has finished the maze to initialize the
     * variables for the next run
     */
    public void resetJunctionCounter() {
        junctionCounter = 0;
        pathCounter=0;
        target=null;
    }
}


/**
 *The Junction class is created in a way that it is a tree. Each node is a Junction instance with a parent node and children
 * nodes that are also Junction instances.
 */
class Junction {
    private  int x, y, arrived, leave;
    private  Junction cN,cS,cW,cE,parent; //the childrens and the parent node, which are also Junction objects

    /**
     * The constructor of the Junction class. It sets the coordinates of the junction, the parent node and initializes the children nodes
     * by setting them us null
     * @param juncx
     * @param juncy
     * @param juncarrived
     * @param junc
     */
    public Junction(int juncx, int juncy, int juncarrived,  Junction junc) {
        setX(juncx);
        setY(juncy);
        setArrived(juncarrived);
        setParent(junc);
        cS=null;
        cN=null;
        cE=null;
        cW=null;
    }

    /**
     * Takes the heading that the robot had when left from the next junction and matches it to create the appropriate new child
     * @param juncx is the x coordinate of the new child junction
     * @param juncy is the y coordinate of the new child junction
     * @param juncarrived is the  heading that the robot had when it arrived for the first time to that junction
     * @param juncleave is the heading that the robot had when it left from the previous junction
     * @param junc it is the previous Junction that will be stored as a parent of the new one
     * @return
     */
    public Junction creatChild(int juncx, int juncy, int juncarrived, int juncleave, Junction junc ){ //choosing which child to create
        switch (juncleave) {
            case IRobot.NORTH:
                cN= new Junction(juncx, juncy, juncarrived, junc);
                return cN;
            case IRobot.EAST:
                cE = new Junction(juncx, juncy, juncarrived, junc);
                return cE;
            case IRobot.WEST:
                cW = new Junction(juncx, juncy, juncarrived, junc);
                return cW;
            default: cS=new Junction(juncx, juncy, juncarrived, junc);
                return cS;
        }
    }

    /**
     * Finds the child of the current node that was lastsly created and deletes it(sets it null). It matches the heading that
     * is stored in the leave variable with the type of child to delete
     */
    public void deleteJunction(){
        switch (leave){
            case IRobot.NORTH:
               cN=null ;
                break;
            case IRobot.EAST:
                cE=null;
                break;
            case IRobot.WEST:
                cW=null;
                break;
            case IRobot.SOUTH: cS=null;
            break;
        }
    }


    /**
     * Method used in the second run of the robot in order to get the child of the current junction that will lead to
     * the target
     * @return the next junction that the robot will head to in order to reach the target
     */
    public Junction findPath(){
        //choose the child that is not null. There is only one of that path because all the other childs that do not lead to the target are deleted
        if(getcN()!=null)
            return getcN();
        if(getcS()!=null)
            return getcS();
        if(getcW()!=null)
            return getcW();
        if(getcE()!=null)
            return getcE();
        System.out.println("Target is found");
        return parent;
    }
    public void setParent(Junction parent){
        this.parent=parent;
    } //setting the parent node of the junction

    public Junction getParent() {
        return parent;
    } //returning the parent of the current node

    public Junction getcN() {
        return cN;
    }

    public Junction getcE() {
        return cE;
    }

    public Junction getcW() {
        return cW;
    }

    public Junction getcS() {
        return cS;
    }

    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }

    public int getX(){
        return x;
    }

    public int getY(){
        return y;
    }

    public void setArrived(int arrived) {
        this.arrived = arrived;
    }

    public int getArrived() {
        return arrived;
    }

    public void setLeave(int leave){this.leave=leave;}

    public int getLeave(){return leave;}

}

