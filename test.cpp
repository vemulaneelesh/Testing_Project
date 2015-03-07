//============================================================================
// Name        : Final.cpp
// Author      : Neelesh Vemula
// Version     :
// Copyright   : Your copyright notice
// Description : Project1 in C++, Ansi-style
//============================================================================

///////////////////////////////////////////////////////////////////////
// 3-value Logic Simulator, written by Michael Hsiao
//   Began in 1992, revisions and additions of functions till 2013
///////////////////////////////////////////////////////////////////////
using namespace std;


#include <stdio.h>
#include <stdlib.h>
//#include <sys/times.h>
#include <strings.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <tr1/unordered_map>
#include "Tree.h"
#include <fstream>
//#include "header.h"
using namespace mytree;

#define HZ 100
#define RETURN '\n'
#define EOS '\0'
#define COMMA ','
#define SPACE ' '
#define TAB '\t'
#define COLON ':'
#define SEMICOLON ';'

#define SPLITMERGE 'M'

#define T_INPUT 1
#define T_OUTPUT 2
#define T_SIGNAL 3
#define T_MODULE 4
#define T_COMPONENT 5
#define T_EXIST 9
#define T_COMMENT 10
#define T_END 11

#define TABLESIZE 5000
#define MAXIO 5000
#define MAXMODULES 5000
#define MAXDFF 10560

#define GOOD 1
#define FAULTY 2
#define DONTCARE -1
#define ALLONES 0xffffffff

#define MAXlevels 10000
#define MAXIOS 5120
#define MAXFanout 10192
#define MAXFFS 40048
#define MAXGATES 100000
#define MAXevents 100000

#define TRUE 1
#define FALSE 0

#define EXCITED_1_LEVEL 1
#define POTENTIAL 2
#define LOW_DETECT 3
#define HIGH_DETECT 4
#define REDUNDANT 5

enum
{
   JUNK,           /* 0 */
   T_input,        /* 1 */
   T_output,       /* 2 */
   T_xor,          /* 3 */
   T_xnor,         /* 4 */
   T_dff,          /* 5 */
   T_and,          /* 6 */
   T_nand,         /* 7 */
   T_or,           /* 8 */
   T_nor,          /* 9 */
   T_not,          /* 10 */
   T_buf,          /* 11 */
   T_tie1,         /* 12 */
   T_tie0,         /* 13 */
   T_tieX,         /* 14 */
   T_tieZ,         /* 15 */
   T_mux_2,        /* 16 */
   T_bus,          /* 17 */
   T_bus_gohigh,   /* 18 */
   T_bus_golow,    /* 19 */
   T_tristate,     /* 20 */
   T_tristateinv,  /* 21 */
   T_tristate1     /* 22 */
};


////////////////////////////////////////////////////////////////////////
// gateLevelCkt class
////////////////////////////////////////////////////////////////////////

class gateLevelCkt
{
    // circuit information
	//int faultExcited;
    int numgates;	// total number of gates (faulty included)
    int numFaultFreeGates;	// number of fault free gates
    int numpri;		// number of PIs
    int numout;		// number of POs
    int maxlevels;	// number of levels in gate level ckt
    int maxLevelSize;	// maximum number of gates in one given level
    int levelSize[MAXlevels];	// levelSize for each level
    int inputs[MAXIOS];
    int outputs[MAXIOS];
    int ff_list[MAXFFS];
    int *ffMap;
    unsigned char *gtype;	// gate type
    unsigned char *faultygtype; //gate type
    short *fanin;		// number of fanin, fanouts
    short *fanout;
    int *levelNum;		// level number of gate
    unsigned *po;
    int **inlist;		// fanin list
    int **fnlist;		// fanout list
    char *sched;		// scheduled on the wheel yet?
    unsigned int *value1;
    unsigned int *value2;	// value of gate
    unsigned int *fvalue1;
    unsigned int *fvalue2;	// faulty value of gate
    int **predOfSuccInput;      // predecessor of successor input-pin list
    int **succOfPredOutput;     // successor of predecessor output-pin list
    int netnum; //netnum
    int *dFrontier;
    int dfrontierpointer;


    // for simulator
    int **levelEvents;	// event list for each level in the circuit
    int *levelLen;	// evenlist length
    int numlevels;	// total number of levels in wheel
    int currLevel;	// current level
    int *activation;	// activation list for the current level in circuit
    int actLen;		// length of the activation list
    int *actFFList;	// activation list for the FF's
    int actFFLen;	// length of the actFFList
    int *visited;		// Visited Matrix to keep track if the gate has been visited
public:
    int numff;		// number of FF's
    bool faultExcited;
    int faultGate;
    int cnt;
    bool prop =false;
    unsigned int *RESET_FF1;	// value of reset ffs read from *.initState
    unsigned int *RESET_FF2;	// value of reset ffs read from *.initState
    Tree *theTree;
    std::vector <int> vec;
    typedef std::tr1::unordered_map <int,int> Mymap;
    Mymap m;
    gateLevelCkt(char *);	// constructor
    void setFaninoutMatrix();	// builds the fanin-out map matrix

    void applyVector(char *);	// apply input vector

    // simulator information
    void setupWheel(int, int);
    void insertEvent(int, int);
    int retrieveEvent();
    void goodsim(int swap);		// logic sim (no faults inserted)
    void badsim();

    void setTieEvents();	// inject events from tied nodes

    void observeOutputs();	// print the fault-free outputs
    void printGoodSig(FILE *, int);	// print the fault-free outputs to *.sig
    void initializevisits();
    void printfanins(int);
    void printcommon(int);
    void addinitial(int);
    void addtomap();
    void printcommonsimulated(int input);
    char *goodState;		// good state (without scan)
    int validate(int gate);
    int validateFaultyGate(int input);
    int validateGateTypeReplaced(int n,int gatetypereplaced);
    void insertFault(int faultygate,int gatetypereplaced);
    void podem();
    int podem_recursion();
    void getObjective(int &a, int &b);
    int backtrace(int );
    int non_controlling(int );
    void applyFaultVector(char *vec);
    int OutputDiffValues();
    bool CheckXPath();
    bool cone(int n);
    void computefrontier(int n);
    bool isFaultExcited();
};


////////////////////////////////////////////////////////////////////////
// 	void initializevisits()
////////////////////////////////////////////////////////////////////////
//	input -
//	output -
//
// 	return value -
////////////////////////////////////////////////////////////////////////
// This function reinitialized the vector for reusability. Also it clears
// the visited array.
// Call this function first
////////////////////////////////////////////////////////////////////////
    void gateLevelCkt::initializevisits()     // Reinitalized the visited matrix and also the vector so it can be used
				// for the next input
    {
    	vec.clear();
        for(int i=0;i<numgates;i++)
        {
            visited[i] = 0;
        }
 //       printf("Initialization Complete \n\n\n");
    }

////////////////////////////////////////////////////////////////////////
// 	void printfanins(int netnum)
////////////////////////////////////////////////////////////////////////
//	input = the gate to do depth first searcg
//	output -
//
// 	return value -
////////////////////////////////////////////////////////////////////////
// This function implements the depth first search alogirthm and builds
// the output into a vector.
////////////////////////////////////////////////////////////////////////
    void gateLevelCkt::printfanins(int netnum)  //Actual DFS Algorithm
    {
        if(netnum <= numgates+1)
		{
		visited[netnum]=1;
	        if(fanin[netnum] == 0 || gtype[netnum] == 5)
	        {
        	    return;
	        }
        	else
	        {
        	    for(int i=0;i<fanin[netnum];i++)
	            {
        	        if(visited[inlist[netnum][i]] == 1)
                	{
	                    continue;
        	        }
                vec.push_back(inlist[netnum][i]);
	            printfanins(inlist[netnum][i]);
	            }
	        }
		}
		else
		{
			printf("Invalid gate number, Exiting \n");
			exit(0);
		}
    }

////////////////////////////////////////////////////////////////////////
// 	void addtomap()
////////////////////////////////////////////////////////////////////////
//	input  -
//	output -
//
// 	return value -
////////////////////////////////////////////////////////////////////////
// This function iterates through the vector obtained from depth first search
// and adds to the hashmap if alreaady not present. Else it just increments
// the value of the particular key.
////////////////////////////////////////////////////////////////////////
    void gateLevelCkt::addtomap() //Adding to the hashmap so that it can be used for finding common gates
    {
		for(unsigned int i=0;i<vec.size();i++)
		{
//			printf("The value of vector being added is %d \n",vec[i]);
			if(m.count(vec[i]) == 0)
			{
				m.insert(Mymap::value_type(vec[i],1));
//				printf("Adding new value \n");
			}
			else
			{
				int temp = m[vec[i]] + 1;
				m[vec[i]] = temp;
//			printf("Incrementing new value \n");
			}
		}
    }

////////////////////////////////////////////////////////////////////////
// 	void addinitial(int gate)
////////////////////////////////////////////////////////////////////////
//	input = gate to be inserted
//
//
// 	return value -
////////////////////////////////////////////////////////////////////////
// This function adds the inputted gate to the hashmap first.
//
////////////////////////////////////////////////////////////////////////
    void gateLevelCkt::addinitial(int gate)   //Adding the inital gate to the map first
    {

		if(m.count(gate) == 0)
		{
				m.insert(Mymap::value_type(gate,1));
//				printf("Adding new value \n");
		}
		else
		{
			int temp = m[gate] + 1;
			m[gate] = temp;
//			printf("Incrementing new value \n");
		}
    }

////////////////////////////////////////////////////////////////////////
// 	void printcommon(int input)
////////////////////////////////////////////////////////////////////////
//	input = Number of inputs given by the user
//	output = Print the common values
//
// 	return value -
////////////////////////////////////////////////////////////////////////
// This function iterates through the hash map and prints the keys whose
// values are equal to number of inputs, which means keys common across
// all inputted gates
////////////////////////////////////////////////////////////////////////
    void gateLevelCkt::printcommon(int input) //Checking the value of key with number of inputs to find the common gates
    {

	printf("The common gates among all the inputs are: \n");
	for(Mymap::iterator it = m.begin(); it!=m.end(); ++it)
	{
		if(it->second == input)
		{
			 printf("%d ", it->first);
		}
	}
		printf("\n");
    }


    ////////////////////////////////////////////////////////////////////////
    // 	void printcommonsimulated(int input)
    ////////////////////////////////////////////////////////////////////////
    //	input = Number of inputs given by the user
    //	output = Print the common values
    //
    // 	return value -
    ////////////////////////////////////////////////////////////////////////
    // This function iterates through the hash map and prints the keys whose
    // values are equal to number of inputs, which means keys common across
    // all inputted gates
    ////////////////////////////////////////////////////////////////////////

    void gateLevelCkt::printcommonsimulated(int input)
    {

        for(int j=1; j < numgates;j++)
        {
        	if(m[j] == input && gtype[j] != 1 && gtype[j] !=5)
        	{

        		if(value2[j] && value1[j])
        		{
        			cout<<"The output of gate " <<j<<" is 1" << endl;
        		}
        		else if ((value2[j] == 0) && (value1[j] ==0))
        		{
        			cout<<"The output of gate " <<j<<" is 0 " << endl;
        		}
        		else
        		{
        			cout<<"The output of gate " <<j<<" is X " << endl;
        		}
        	}
        }


    }

    int gateLevelCkt::validate(int gate)
    {
    	if (gtype[gate] == 2)
    	{
    		return 1;
    	}
    	else
    	{
    		return 0;
    	}

    }
    int gateLevelCkt::validateFaultyGate(int faultygate)
    {
    	if(faultygate <= numgates)
    	{
    		return 1;
    	}
    	else
    	{
    		return 0;
    	}
    }
    int gateLevelCkt::validateGateTypeReplaced(int faultygate,int gatetypereplaced)
    {
    	if (gatetypereplaced <= 9 && gatetypereplaced >=3 && gatetypereplaced !=5 && gtype[faultygate] != gatetypereplaced)
    	{
    		if(gtype[faultygate] <=9 && gtype[faultygate] >=3 && gtype[faultygate] !=5)
    			{
    			return 1;
    			}
    		else
    		{
    			return 0;
    		}
    	}
    	else
    	{
    		return 0;
    	}
    }
    void gateLevelCkt::insertFault(int faultygate,int gatetypereplaced)
    {
    	faultygtype[faultygate] = gatetypereplaced ;
    	faultGate = faultygate;
    }
    void gateLevelCkt::podem()
    {
    	//intialize values to x and d-frontier already empty
    	theTree = new Tree();
    	theTree->initialize(numpri);
    	int detected = podem_recursion();
    	int printonemore = faultygtype[faultGate];
    	ofstream myfile;
    	myfile.open ("FaultStatus.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    	myfile <<faultGate;
    	myfile << " ";
    	myfile << printonemore;
    	myfile <<" ";
    	if(detected)
    	{
    		cout << "The Fault is Detected and the Vector is: " <<endl;


    		for(int i=1;i<=numpri;i++)
    		{
    			int printing = theTree->inputarray[i];
    			switch(printing)
    			{
    			case 0 : myfile <<"0";cout <<"0";break;
    			case 1: myfile <<"1";cout << "1";break;
    			default: myfile <<"X"; cout << "X";
    			}
    		}
    		myfile<<endl;
    		cout <<endl;
    		myfile.close();
    		return;
    	}
    	else
    	{
    		printf("Fault is Untestable");
    		myfile << "Fault_Not_Detected" <<endl;
    		myfile.close();
    		return;
    	}
    }

    int gateLevelCkt::podem_recursion()
    {
    	int gateObjective,gateValue,piToApply,result,outputdetected;
    	char *vec;
    	faultExcited = false;
    	faultExcited = isFaultExcited();
        int gateN,i,successor;
        cnt = 0;
        computefrontier(faultGate);
        /*cout<<"\n D front ";
        for(i=0;D_front[i]!=0;i++)
        	cout<<D_front[i]<<" ";
        cout<<endl;*/
        //fault_effect = false; ////////////////tomorrow

        outputdetected = OutputDiffValues();
            if((outputdetected == 1))
            {
                return 1;
            }

        if(outputdetected != 1)
        {

            if(CheckXPath() == 1)
            {
                if(faultExcited == false)
                {
                	piToApply = backtrace(faultGate);
                }
                else
                {
                	getObjective(gateObjective,gateValue);
                	piToApply = backtrace(gateObjective);
                }

                theTree->inputarray[piToApply] = 0;
                vec = new char[numpri + 2];
                for(int i=1;i<=numpri;i++)
                {
                    switch(theTree->inputarray[i])
                    {
                    case 0: vec[i] = '0';
                            break;
                    case 1: vec[i] = '1';
                            break;
                    default: vec[i] = 'X';

                    }
                }
                vec = vec+1;
                applyVector(vec);
                goodsim(0);
                applyFaultVector(vec);
                badsim();
                //observeOutputs();
                // Backtrack if previous PI assignment was incorrect
                if(podem_recursion() == 1)
                {
                    return true;
                }
                theTree->inputarray[piToApply] = 1;
                vec = new char[numpri + 2];
                for(int i=1;i<=numpri;i++)
                {
                    switch(theTree->inputarray[i])
                    {
                    case 0: vec[i] = '0';
                            break;
                    case 1: vec[i] = '1';
                            break;
                    default: vec[i] = 'X';

                    }
                }
                vec = vec+1;
                applyVector(vec);
                goodsim(0);
                applyFaultVector(vec);
                badsim();
                //observeOutputs();
                if(podem_recursion() == 1)
                {

                    return 1;
                }

                // Assign current PI to 'X'
                theTree->inputarray[piToApply] = 10000002;
                vec = new char[numpri + 2];
                for(int i=1;i<=numpri;i++)
                {
                    switch(theTree->inputarray[i])
                    {
                    case 0: vec[i] = '0';
                            break;
                    case 1: vec[i] = '1';
                            break;
                    default: vec[i] = 'X';

                    }
                }
                vec = vec+1;
                applyVector(vec);
                goodsim(0);
                applyFaultVector(vec);
                badsim();
                //observeOutputs();

                return 0;
            }	// if XPathChk ...
            else
                return 0;												// Backtrack
        }		// if fault effect ...

        return 1;
    }

    bool gateLevelCkt::isFaultExcited()
    {
    	if( ((fvalue1[faultGate]==0 && fvalue2[faultGate]==0) && (value1[faultGate] && value2[faultGate])) || ((fvalue1[faultGate] && fvalue2[faultGate]) && (value1[faultGate]==0 && value2[faultGate]==0))  )
    	{
    	    	            return true;
    	}
    	else
    	{
    		return false;
    	}
    }

    int gateLevelCkt::OutputDiffValues()
    {
    	for ( int i = 0; i < numout; i++)  //if the PO has a D or Dbar Then fault_effect==true
    	    {

    	        if( ((fvalue1[outputs[i]]==0 && fvalue2[outputs[i]]==0) && (value1[outputs[i]] && value2[outputs[i]])) || ((fvalue1[outputs[i]] && fvalue2[outputs[i]]) && (value1[outputs[i]]==0 && value2[outputs[i]]==0))  )
    	        {
    	            return 1;
    	        }
    	    }
    	return 0;
    }

    bool gateLevelCkt::CheckXPath()
    {
    	 int i = 0,j;
    	    prop = 0;
    	    while(dFrontier[i])
    	    {
    	        j = cone(dFrontier[i]);
    	        if( j == 1)
    	            return(true);
    	        else
    	        {
    	            i++;
    	        }
    	    }
    	    return false;
    }

    bool gateLevelCkt::cone(int gate)
    {
        if((gtype[gate]) == 2 || prop == 1)   //if gatetype==2 =output
        {
            prop = 1;
            return prop;
        }
        else
        {
            for(int i = 0; i <fanout[gate]; i++)
            {
   	            if((value1[fnlist[gate][i]] == 0 && value2[fnlist[gate][i]]) || (fvalue1[fnlist[gate][i]] == 0 && fvalue2[fnlist[gate][i]]) || (value1[fnlist[gate][i]] && value2[fnlist[gate][i]]==0) || (fvalue1[fnlist[gate][i]] && fvalue2[fnlist[gate][i]]==0) )
   	            {
   	                (cone(fnlist[gate][i]));
   	            }
   	        }
   	    }
   	    return prop;
   	}

    void gateLevelCkt :: computefrontier(int gateN)//// if value is dont care D frontier is updated or if value is D or Dbar the functionn keeps checkin g the fanouts until a x is found on a aget
    {
        // checks dont cares in that gate
        if(((value1[gateN] == 0 && value2[gateN]) || (fvalue1[gateN] == 0 && fvalue2[gateN]) || (value1[gateN] && value2[gateN]==0) || (fvalue1[gateN] && fvalue2[gateN]==0) ))
        {
            dFrontier[cnt]=gateN;
            cnt++;
        }

        else if((value1[gateN] && value2[gateN] && (fvalue1[gateN] == 0) && (fvalue2[gateN] == 0)) || (value1[gateN] == 0 && value2[gateN] == 0 && (fvalue1[gateN]) && (fvalue2[gateN])))
        {
            for(int i = 0; i <fanout[gateN]; i++)
            {
                computefrontier(fnlist[gateN][i]);
            }
        }

        dFrontier[cnt]=0;
    }


    void gateLevelCkt::getObjective(int &a ,int &b)
    {
    	int dfrontiergate;
    	dfrontiergate = dFrontier[0];
    	for(int i =0; i<=fanin[dfrontiergate];i++)
    	{
    		if (value1[inlist[dfrontiergate][i]] != value2[inlist[dfrontiergate][i]])
    		{
    			a = inlist[dfrontiergate][i];
    			b = non_controlling(gtype[dfrontiergate]);
    			break;
    		}
    	}
    	return;
    }
    int gateLevelCkt::backtrace(int gateObjective)
    {
    	int i = gateObjective;
    	int num_inverters = 0;
    	while(gtype[i] != 1)
    	{
    		for(int j =0; j<=fanin[i];j++)
    	   	{
    			if (value1[inlist[i][j]] != value2[inlist[i][j]])
    	   		{
    				if (gtype[i] == 10)
    				{
    					num_inverters++;
    				}
    				i = inlist[i][j];
    	   			break;
    	   		}
    	   	}
    	}
    	/*if((num_inverters % 2) == 1)
    	{
    		arr[1] = !arr[1];
    	}*/
    	return i;
    }

    int gateLevelCkt::non_controlling(int gtype)
    {
    	switch(gtype)
    	{
    	  	case 6 : return 1;
    	  	case 7 : return 1;
    	  	case 8 : return 0;
    	  	case 9 : return 0;
    	  	default : return 0;
    	}
    }

//////////////////////////////////////////////////////////////////////////
// Global Variables

char vector1[5120];
gateLevelCkt *circuit;
int OBSERVE, INIT0;
int vecNum=0;
int numTieNodes;
int TIES[512];
int input;
int faultExcited;

//////////////////////////////////////////////////////////////////////////
// Functions start here


//////////////////////////////////////////////////////////////////////////
// returns 1 if a regular vector, returns 2 if a scan
//////////////////////////////////////////////////////////////////////////
int getVector(FILE *inFile, int vecSize)
{
    int i;
    char thisChar;

    fscanf(inFile, "%c", &thisChar);
    while ((thisChar == SPACE) || (thisChar == RETURN))
	fscanf(inFile, "%c", &thisChar);

    vector1[0] = thisChar;
    if (vector1[0] == 'E')
	return (0);

    for (i=1; i<vecSize; i++)
	fscanf(inFile, "%c", &vector1[i]);
    vector1[i] = EOS;

    fscanf(inFile, "%c", &thisChar);

    // read till end of line
    while ((thisChar != RETURN) && (thisChar != EOF))
    	fscanf(inFile, "%c", &thisChar);

    return(1);
}

// logicSimFromFile() - logic simulates all vectors in vecFile on the circuit
int logicSimFromFile(FILE *vecFile, int vecWidth,int swap)
{
    int moreVec;

    moreVec = 1;
    while (moreVec)
    {
     moreVec = getVector(vecFile, vecWidth);
     if (moreVec == 1)
     {
printf("vector #%d: %s\n", vecNum, vector1);
	circuit->applyVector(vector1);
        circuit->goodsim(swap);      // simulate the vector


circuit->applyFaultVector(vector1);
        circuit->badsim();
        if (OBSERVE) circuit->observeOutputs();
        vecNum++;
     }  // if (moreVec == 1)
    }   // while (getVector...)

    return (vecNum);
}



////////////////////////////////////////////////////////////////////////
inline void gateLevelCkt::insertEvent(int levelN, int gateN)
{
//printf("InsertEvent %d %d levelsize %d\n", levelN, gateN, levelLen[levelN]);
    levelEvents[levelN][levelLen[levelN]] = gateN;
    levelLen[levelN]++;
}

////////////////////////////////////////////////////////////////////////
// gateLevelCkt class
////////////////////////////////////////////////////////////////////////

// constructor: reads in the *.lev file for the gate-level ckt
gateLevelCkt::gateLevelCkt(char *cktName)
{
    FILE *yyin;
    char fName[256];
    int i, j, count;
    char c;
    int junk;
    int f1, f2, f3;
    int levelSize[MAXlevels];

    strcpy(fName, cktName);
    strcat(fName, ".lev");
    yyin = fopen(fName, "r");
    if (yyin == NULL)
    {
	fprintf(stderr, "Can't open .lev file\n");
	exit(-1);
    }

    numpri = numgates = numout = maxlevels = numff = 0;
    maxLevelSize = 32;
    for (i=0; i<MAXlevels; i++)
	levelSize[i] = 0;

    fscanf(yyin, "%d", &count);	// number of gates
    fscanf(yyin, "%d", &junk);

    // allocate space for gates
    gtype = new unsigned char[count+64];
    faultygtype = new unsigned char[count+64];
    fanin = new short[count+64];
    fanout = new short[count+64];
    levelNum = new int[count+64];
    po = new unsigned[count+64];
    inlist = new int * [count+64];
    fnlist = new int * [count+64];
    sched = new char[count+64];
    value1 = new unsigned int[count+64];
    value2 = new unsigned int[count+64];
    fvalue1 = new unsigned int[count+64];
    fvalue2 = new unsigned int[count+64];
    visited = new int[count];
    dFrontier = new int[count];

    // now read in the circuit
    numTieNodes = 0;
    for (i=1; i<count; i++)
    {
	fscanf(yyin, "%d", &netnum);
	fscanf(yyin, "%d", &f1);
	fscanf(yyin, "%d", &f2);
	fscanf(yyin, "%d", &f3);

	numgates++;
	gtype[netnum] = (unsigned char) f1;
	f2 = (int) f2;
	levelNum[netnum] = f2;
	levelSize[f2]++;

	if (f2 >= (maxlevels))
	    maxlevels = f2 + 5;
	if (maxlevels > MAXlevels)
	{
	    fprintf(stderr, "MAXIMUM level (%d) exceeded.\n", maxlevels);
	    exit(-1);
	}

	fanin[netnum] = (int) f3;
	if (f3 > MAXFanout)
	    fprintf(stderr, "Fanin count (%d) exceeded\n", fanin[netnum]);

	if (gtype[netnum] == T_input)
	{
	    inputs[numpri] = netnum;
	    numpri++;
	}
	if (gtype[netnum] == T_dff)
	{
	    if (numff >= (MAXFFS-1))
	    {
		fprintf(stderr, "The circuit has more than %d FFs\n", MAXFFS-1);
		exit(-1);
	    }
	    ff_list[numff] = netnum;
	    numff++;
	}

	sched[netnum] = 0;

	// now read in the inlist
	inlist[netnum] = new int[fanin[netnum]];
	for (j=0; j<fanin[netnum]; j++)
	{
	    fscanf(yyin, "%d", &f1);
	    inlist[netnum][j] = (int) f1;
	}

	for (j=0; j<fanin[netnum]; j++)	  // followed by close to samethings
	    fscanf(yyin, "%d", &junk);

	// read in the fanout list
	fscanf(yyin, "%d", &f1);
	fanout[netnum] = (int) f1;

	if (gtype[netnum] == T_output)
	{
	    po[netnum] = TRUE;
	    outputs[numout] = netnum;
	    numout++;
	}
	else
	    po[netnum] = 0;

	if (fanout[netnum] > MAXFanout)
	    fprintf(stderr, "Fanout count (%d) exceeded\n", fanout[netnum]);

	fnlist[netnum] = new int[fanout[netnum]];
	for (j=0; j<fanout[netnum]; j++)
	{
	    fscanf(yyin, "%d", &f1);
	    fnlist[netnum][j] = (int) f1;
	}

	if (gtype[netnum] == T_tie1)
        {
	    TIES[numTieNodes] = netnum;
	    numTieNodes++;
	    if (numTieNodes > 511)
	    {
		fprintf(stderr, "Can't handle more than 512 tied nodes\n");
		exit(-1);
	    }
            value1[netnum] = ALLONES;
            value2[netnum] = ALLONES;
        }
        else if (gtype[netnum] == T_tie0)
        {
	    TIES[numTieNodes] = netnum;
	    numTieNodes++;
	    if (numTieNodes > 511)
	    {
		fprintf(stderr, "Can't handle more than 512 tied nodes\n");
		exit(-1);
	    }
            value1[netnum] = 0;
            value2[netnum] = 0;
        }
        else
        {
	    // assign all values to unknown
	    value1[netnum] = 0;
	    value2[netnum] = ALLONES;
	}

	// read in and discard the observability values
/*
	fscanf(yyin, "%d", &junk);
	fscanf(yyin, "%c", &c);		// ';' or 'O'
	fscanf(yyin, "%d", &junk);	// 0 controllability
	fscanf(yyin, "%d", &junk);	// 1 controllability
*/

	// read till end of line
	while ((c = getc(yyin)) != '\n' && c != EOF)
	    ;
    }	// for (i...)
    fclose(yyin);
    numgates++;
    numFaultFreeGates = numgates;

    // now compute the maximum width of the level
    for (i=0; i<maxlevels; i++)
    {
	if (levelSize[i] > maxLevelSize)
	    maxLevelSize = levelSize[i] + 1;
    }

    // allocate space for the faulty gates
    for (i = numgates; i < numgates+64; i+=2)
    {
        inlist[i] = new int[2];
        fnlist[i] = new int[MAXFanout];
        po[i] = 0;
        fanin[i] = 2;
        inlist[i][0] = i+1;
	sched[i] = 0;
    }

    //printf("Successfully read in circuit:\n");
    //printf("\t%d PIs.\n", numpri);
    //printf("\t%d POs.\n", numout);
    //printf("\t%d Dffs.\n", numff);
    //printf("\t%d total number of gates\n", numFaultFreeGates);
    //printf("\t%d levels in the circuit.\n", maxlevels / 5);
    goodState = new char[numff];
    ffMap = new int[numgates];
    // get the ffMap
    for (i=0; i<numff; i++)
    {
	ffMap[ff_list[i]] = i;
	goodState[i] = 'X';
    }

    setupWheel(maxlevels, maxLevelSize);
    setFaninoutMatrix();

    if (INIT0)	// if start from a initial state
    {
	RESET_FF1 = new unsigned int [numff+2];
	RESET_FF2 = new unsigned int [numff+2];

	strcpy(fName, cktName);
	strcat(fName, ".initState");
	yyin = fopen(fName, "r");
	if (yyin == NULL)
	{	fprintf(stderr, "Can't open %s\n", fName); exit(-1);}

	for (i=0; i<numff; i++)
	{
	    fscanf(yyin, "%c", &c);

	    if (c == '0')
	    {
		RESET_FF1[i] = 0;
		RESET_FF2[i] = 0;
	    }
	    else if (c == '1')
	    {
		RESET_FF1[i] = ALLONES;
		RESET_FF2[i] = ALLONES;
	    }
	    else
	    {
		RESET_FF1[i] = 0;
		RESET_FF2[i] = ALLONES;
	    }
	}

	fclose(yyin);
    }

    for(i=0;i<numgates;i++)
    {
    	faultygtype[i] = gtype[i];
    	fvalue1[i] = 0;
    	fvalue2[i] = ALLONES;
    }
}

////////////////////////////////////////////////////////////////////////
// setFaninoutMatrix()
//	This function builds the matrix of succOfPredOutput and
// predOfSuccInput.
////////////////////////////////////////////////////////////////////////

void gateLevelCkt::setFaninoutMatrix()
{
    int i, j, k;
    int predecessor, successor;
    int checked[MAXFanout];
    int checkID;	// needed for gates with fanouts to SAME gate
    int prevSucc, found;

    predOfSuccInput = new int *[numgates+64];
    succOfPredOutput = new int *[numgates+64];
    for (i=0; i<MAXFanout; i++)
	checked[i] = 0;
    checkID = 1;

    prevSucc = -1;
    for (i=1; i<numgates; i++)
    {
	predOfSuccInput[i] = new int [fanout[i]];
	succOfPredOutput[i] = new int [fanin[i]];

	for (j=0; j<fanout[i]; j++)
	{
	    if (prevSucc != fnlist[i][j])
		checkID++;
	    prevSucc = fnlist[i][j];

	    successor = fnlist[i][j];
	    k=found=0;
	    while ((k<fanin[successor]) && (!found))
	    {
		if ((inlist[successor][k] == i) && (checked[k] != checkID))
		{
		    predOfSuccInput[i][j] = k;
		    checked[k] = checkID;
		    found = 1;
		}
		k++;
	    }
	}

	for (j=0; j<fanin[i]; j++)
	{
	    if (prevSucc != inlist[i][j])
		checkID++;
	    prevSucc = inlist[i][j];

	    predecessor = inlist[i][j];
	    k=found=0;
	    while ((k<fanout[predecessor]) && (!found))
	    {
		if ((fnlist[predecessor][k] == i) && (checked[k] != checkID))
		{
		    succOfPredOutput[i][j] = k;
		    checked[k] = checkID;
		    found=1;
		}
		k++;
	    }
	}
    }

    for (i=numgates; i<numgates+64; i+=2)
    {
	predOfSuccInput[i] = new int[MAXFanout];
	succOfPredOutput[i] = new int[MAXFanout];
    }
}

////////////////////////////////////////////////////////////////////////
// setTieEvents()
//	This function set up the events for tied nodes
////////////////////////////////////////////////////////////////////////

void gateLevelCkt::setTieEvents()
{
    int predecessor, successor;
    int i, j;

    for (i = 0; i < numTieNodes; i++)
    {
	  // different from previous time frame, place in wheel
	  for (j=0; j<fanout[TIES[i]]; j++)
	  {
	    successor = fnlist[TIES[i]][j];
	    if (sched[successor] == 0)
	    {
	    	insertEvent(levelNum[successor], successor);
		sched[successor] = 1;
	    }
	  }
    }	// for (i...)

    // initialize state if necessary
    if (INIT0 == 1)
    {
printf("Initialize circuit to values in *.initState!\n");
	for (i=0; i<numff; i++)
	{
	    value1[ff_list[i]] = value2[ff_list[i]] = RESET_FF1[i];

	  for (j=0; j<fanout[ff_list[i]]; j++)
	  {
	    successor = fnlist[ff_list[i]][j];
	    if (sched[successor] == 0)
	    {
	    	insertEvent(levelNum[successor], successor);
		sched[successor] = 1;
	    }
	  }	// for j

	    predecessor = inlist[ff_list[i]][0];
	    value1[predecessor] = value2[predecessor] = RESET_FF1[i];

	  for (j=0; j<fanout[predecessor]; j++)
	  {
	    successor = fnlist[predecessor][j];
	    if (sched[successor] == 0)
	    {
	    	insertEvent(levelNum[successor], successor);
		sched[successor] = 1;
	    }
	  }	// for j

	}	// for i
    }	// if (INIT0)
}

////////////////////////////////////////////////////////////////////////
// applyVector()
//	This function applies the vector to the inputs of the ckt.
////////////////////////////////////////////////////////////////////////

void gateLevelCkt::applyVector(char *vec)
{
    unsigned int origVal1, origVal2;
    char origBit;
    int successor;
    int i, j;

    for (i = 0; i < numpri; i++)
    {
	origVal1 = value1[inputs[i]] & 1;
	origVal2 = value2[inputs[i]] & 1;
	if ((origVal1 == 1) && (origVal2 == 1))
	    origBit = '1';
	else if ((origVal1 == 0) && (origVal2 == 0))
	    origBit = '0';
	else
	    origBit = 'x';

	if ((origBit != vec[i]) && ((origBit != 'x') || (vec[i] != 'X')))
	{
	  switch (vec[i])
	  {
	    case '0':
		value1[inputs[i]] = 0;
		value2[inputs[i]] = 0;
		break;
	    case '1':
		value1[inputs[i]] = ALLONES;
		value2[inputs[i]] = ALLONES;
		break;
	    case 'x':
	    case 'X':
		value1[inputs[i]] = 0;
		value2[inputs[i]] = ALLONES;
		break;
	    default:
		fprintf(stderr, "%c: error in the input vector.\n", vec[i]);
		exit(-1);
	  }	// switch

	  // different from previous time frame, place in wheel
	  for (j=0; j<fanout[inputs[i]]; j++)
	  {
	    successor = fnlist[inputs[i]][j];
	    if (sched[successor] == 0)
	    {
	    	insertEvent(levelNum[successor], successor);
		sched[successor] = 1;
	    }
	  }
	}	// if ((origBit...)
    }	// for (i...)
}

////////////////////////////////////////////////////////////////////////
// lowWheel class
////////////////////////////////////////////////////////////////////////

void gateLevelCkt::setupWheel(int numLevels, int levelSize)
{
    int i;

    numlevels = numLevels;
    levelLen = new int[numLevels];
    levelEvents = new int * [numLevels];
    for (i=0; i < numLevels; i++)
    {
	levelEvents[i] = new int[levelSize];
	levelLen[i] = 0;
    }
    activation = new int[levelSize];

    actFFList = new int[numff + 1];
}

////////////////////////////////////////////////////////////////////////
int gateLevelCkt::retrieveEvent()
{
    while ((levelLen[currLevel] == 0) && (currLevel < maxlevels))
	currLevel++;

    if (currLevel < maxlevels)
    {
    	levelLen[currLevel]--;
        return(levelEvents[currLevel][levelLen[currLevel]]);
    }
    else
	return(-1);
}

////////////////////////////////////////////////////////////////////////
// goodsim() -
//	Logic simulate. (no faults inserted)
////////////////////////////////////////////////////////////////////////

void gateLevelCkt::goodsim(int swap)
{
    int sucLevel;
    int gateN, predecessor, successor;
    int *predList;
    int i;
    unsigned int val1, val2, tmpVal;

    currLevel = 0;
    actLen = actFFLen = 0;
    while (currLevel < maxlevels)
    {
    	gateN = retrieveEvent();

	if (gateN != -1)	// if a valid event
	{
	    sched[gateN]= 0;
    	switch (gtype[gateN])
    	    {
	      case T_and:
    	    	val1 = val2 = ALLONES;
		predList = inlist[gateN];
    	    	for (i=0; i<fanin[gateN]; i++)
    	    	{
		    predecessor = predList[i];
		    val1 &= value1[predecessor];
		    val2 &= value2[predecessor];
    	    	}
	    	break;
	      case T_nand:
    	        val1 = val2 = ALLONES;
		predList = inlist[gateN];
    	    	for (i=0; i<fanin[gateN]; i++)
    	    	{
		    predecessor = predList[i];
		    val1 &= value1[predecessor];
		    val2 &= value2[predecessor];
    	    	}
	    	tmpVal = val1;
	    	val1 = ALLONES ^ val2;
	    	val2 = ALLONES ^ tmpVal;
	    	break;
	      case T_or:
    	        val1 = val2 = 0;
		predList = inlist[gateN];
    	        for (i=0; i<fanin[gateN]; i++)
    	        {
		    predecessor = predList[i];
		    val1 |= value1[predecessor];
		    val2 |= value2[predecessor];
    	    	}
	    	break;
	      case T_nor:
    	    	val1 = val2 = 0;
		predList = inlist[gateN];
		for (i=0; i<fanin[gateN]; i++)
    	    	{
		    predecessor = predList[i];
		    val1 |= value1[predecessor];
		    val2 |= value2[predecessor];
    	    	}
	    	tmpVal = val1;
	    	val1 = ALLONES ^ val2;
	    	val2 = ALLONES ^ tmpVal;
	    	break;
	      case T_not:
	    	predecessor = inlist[gateN][0];
	    	val1 = ALLONES ^ value2[predecessor];
	    	val2 = ALLONES ^ value1[predecessor];
	    	break;
	      case T_buf:
	    	predecessor = inlist[gateN][0];
	    	val1 = value1[predecessor];
	    	val2 = value2[predecessor];
		break;
	      case T_dff:
	    	predecessor = inlist[gateN][0];
	    	val1 = value1[predecessor];
	    	val2 = value2[predecessor];
		actFFList[actFFLen] = gateN;
		actFFLen++;
	    	break;
	      case T_xor:
		predList = inlist[gateN];
	    	val1 = value1[predList[0]];
	    	val2 = value2[predList[0]];

            	for(i=1;i<fanin[gateN];i++)
            	{
	    	    predecessor = predList[i];
                    tmpVal = ALLONES^(((ALLONES^value1[predecessor]) &
              		(ALLONES^val1)) | (value2[predecessor]&val2));
                    val2 = ((ALLONES^value1[predecessor]) & val2) |
                  	(value2[predecessor] & (ALLONES^val1));
                    val1 = tmpVal;
            	}
	    	break;
	      case T_xnor:
		predList = inlist[gateN];
		val1 = value1[predList[0]];
	    	val2 = value2[predList[0]];

            	for(i=1;i<fanin[gateN];i++)
            	{
	    	    predecessor = predList[i];
                    tmpVal = ALLONES^(((ALLONES^value1[predecessor]) &
                   	(ALLONES^val1)) | (value2[predecessor]&val2));
                    val2 = ((ALLONES^value1[predecessor]) & val2) |
                  	(value2[predecessor]& (ALLONES^val1));
                    val1 = tmpVal;
            	}
	    	tmpVal = val1;
		val1 = ALLONES ^ val2;
	    	val2 = ALLONES ^ tmpVal;
	    	break;
	      case T_output:
		predecessor = inlist[gateN][0];
	    	val1 = value1[predecessor];
	    	val2 = value2[predecessor];
	        break;
	      case T_input:
	      case T_tie0:
	      case T_tie1:
	      case T_tieX:
	      case T_tieZ:
	    	val1 = value1[gateN];
	    	val2 = value2[gateN];
	    	break;
	      default:
		fprintf(stderr, "illegal gate type1 %d %d\n", gateN, gtype[gateN]);
		exit(-1);
    	    }	// switch

	    // if gate value changed
    	if ((val1 != value1[gateN]) || (val2 != value2[gateN]))
	    {
		value1[gateN] = val1;
		value2[gateN] = val2;
		//cout << "Value of gate " <<gateN << "is " <<value2[gateN] <<endl;

		for (i=0; i<fanout[gateN]; i++)
		{
		    successor = fnlist[gateN][i];
		    sucLevel = levelNum[successor];
		    if (sched[successor] == 0)
		    {
		      if (sucLevel != 0)
			insertEvent(sucLevel, successor);
		      else	// same level, wrap around for next time
		      {
			activation[actLen] = successor;
			actLen++;
		      }
		      sched[successor] = 1;
		    }
		}	// for (i...)
	    }	// if (val1..)
	}	// if (gateN...)
    }	// while (currLevel...)
    // now re-insert the activation list for the FF's
    for (i=0; i < actLen; i++)
    {
	insertEvent(0, activation[i]);
	sched[activation[i]] = 0;

        predecessor = inlist[activation[i]][0];
        gateN = ffMap[activation[i]];
        if (value1[predecessor])
            goodState[gateN] = '1';
        else if (value2[predecessor] == 0)
            goodState[gateN] = '0';
        else
            goodState[gateN] = 'X';
    }
}


////////////////////////////////////////////////////////////////////////
// badsim() -
//	Faulty simulate.
////////////////////////////////////////////////////////////////////////

void gateLevelCkt::badsim()
{
    int sucLevel;
    int gateN, predecessor, successor;
    int *predList;
    int i;
    unsigned int val1, val2, tmpVal;

    currLevel = 0;
    actLen = actFFLen = 0;
    while (currLevel < maxlevels)
    {
    	gateN = retrieveEvent();

	if (gateN != -1)	// if a valid event
	{
	    sched[gateN]= 0;
    	switch (faultygtype[gateN])
    	    {
	      case T_and:
    	    	val1 = val2 = ALLONES;
		predList = inlist[gateN];
    	    	for (i=0; i<fanin[gateN]; i++)
    	    	{
		    predecessor = predList[i];
		    val1 &= fvalue1[predecessor];
		    val2 &= fvalue2[predecessor];
    	    	}
	    	break;
	      case T_nand:
    	        val1 = val2 = ALLONES;
		predList = inlist[gateN];
    	    	for (i=0; i<fanin[gateN]; i++)
    	    	{
		    predecessor = predList[i];
		    val1 &= fvalue1[predecessor];
		    val2 &= fvalue2[predecessor];
    	    	}
	    	tmpVal = val1;
	    	val1 = ALLONES ^ val2;
	    	val2 = ALLONES ^ tmpVal;
	    	break;
	      case T_or:
    	        val1 = val2 = 0;
		predList = inlist[gateN];
    	        for (i=0; i<fanin[gateN]; i++)
    	        {
		    predecessor = predList[i];
		    val1 |= fvalue1[predecessor];
		    val2 |= fvalue2[predecessor];
    	    	}
	    	break;
	      case T_nor:
    	    	val1 = val2 = 0;
		predList = inlist[gateN];
		for (i=0; i<fanin[gateN]; i++)
    	    	{
		    predecessor = predList[i];
		    val1 |= fvalue1[predecessor];
		    val2 |= fvalue2[predecessor];
    	    	}
	    	tmpVal = val1;
	    	val1 = ALLONES ^ val2;
	    	val2 = ALLONES ^ tmpVal;
	    	break;
	      case T_not:
	    	predecessor = inlist[gateN][0];
	    	val1 = ALLONES ^ fvalue2[predecessor];
	    	val2 = ALLONES ^ fvalue1[predecessor];
	    	break;
	      case T_buf:
	    	predecessor = inlist[gateN][0];
	    	val1 = fvalue1[predecessor];
	    	val2 = fvalue2[predecessor];
		break;
	      case T_dff:
	    	predecessor = inlist[gateN][0];
	    	val1 = fvalue1[predecessor];
	    	val2 = fvalue2[predecessor];
		actFFList[actFFLen] = gateN;
		actFFLen++;
	    	break;
	      case T_xor:
		predList = inlist[gateN];
	    	val1 = fvalue1[predList[0]];
	    	val2 = fvalue2[predList[0]];

            	for(i=1;i<fanin[gateN];i++)
            	{
	    	    predecessor = predList[i];
                    tmpVal = ALLONES^(((ALLONES^fvalue1[predecessor]) &
              		(ALLONES^val1)) | (fvalue2[predecessor]&val2));
                    val2 = ((ALLONES^fvalue1[predecessor]) & val2) |
                  	(fvalue2[predecessor] & (ALLONES^val1));
                    val1 = tmpVal;
            	}
	    	break;
	      case T_xnor:
		predList = inlist[gateN];
		val1 = fvalue1[predList[0]];
	    	val2 = fvalue2[predList[0]];

            	for(i=1;i<fanin[gateN];i++)
            	{
	    	    predecessor = predList[i];
                    tmpVal = ALLONES^(((ALLONES^fvalue1[predecessor]) &
                   	(ALLONES^val1)) | (value2[predecessor]&val2));
                    val2 = ((ALLONES^fvalue1[predecessor]) & val2) |
                  	(value2[predecessor]& (ALLONES^val1));
                    val1 = tmpVal;
            	}
	    	tmpVal = val1;
		val1 = ALLONES ^ val2;
	    	val2 = ALLONES ^ tmpVal;
	    	break;
	      case T_output:
		predecessor = inlist[gateN][0];
	    	val1 = fvalue1[predecessor];
	    	val2 = fvalue2[predecessor];
	        break;
	      case T_input:
	      case T_tie0:
	      case T_tie1:
	      case T_tieX:
	      case T_tieZ:
	    	val1 = fvalue1[gateN];
	    	val2 = fvalue2[gateN];
	    	break;
	      default:
		fprintf(stderr, "illegal gate type1 %d %d\n", gateN, faultygtype[gateN]);
		exit(-1);
    	    }	// switch

	    // if gate value changed
    	if ((val1 != fvalue1[gateN]) || (val2 != fvalue2[gateN]))
	    {
		fvalue1[gateN] = val1;
		fvalue2[gateN] = val2;
		//cout << "Value of gate " <<gateN << "is " <<value2[gateN] <<endl;

		for (i=0; i<fanout[gateN]; i++)
		{
		    successor = fnlist[gateN][i];
		    sucLevel = levelNum[successor];
		    if (sched[successor] == 0)
		    {
		      if (sucLevel != 0)
			insertEvent(sucLevel, successor);
		      else	// same level, wrap around for next time
		      {
			activation[actLen] = successor;
			actLen++;
		      }
		      sched[successor] = 1;
		    }
		}	// for (i...)
	    }	// if (val1..)
	}	// if (gateN...)
    }	// while (currLevel...)
    // now re-insert the activation list for the FF's
    for (i=0; i < actLen; i++)
    {
	insertEvent(0, activation[i]);
	sched[activation[i]] = 0;

        predecessor = inlist[activation[i]][0];
        gateN = ffMap[activation[i]];
        if (fvalue1[predecessor])
            goodState[gateN] = '1';
        else if (fvalue2[predecessor] == 0)
            goodState[gateN] = '0';
        else
            goodState[gateN] = 'X';
    }
}



void gateLevelCkt::applyFaultVector(char *vec)
{
    unsigned int origVal1, origVal2;
    char origBit;
    int successor;
    int i, j;

    for (i = 0; i < numpri; i++)
    {
	origVal1 = fvalue1[inputs[i]] & 1;
	origVal2 = fvalue2[inputs[i]] & 1;
	if ((origVal1 == 1) && (origVal2 == 1))
	    origBit = '1';
	else if ((origVal1 == 0) && (origVal2 == 0))
	    origBit = '0';
	else
	    origBit = 'x';

	if ((origBit != vec[i]) && ((origBit != 'x') || (vec[i] != 'X')))
	{
	  switch (vec[i])
	  {
	    case '0':
		fvalue1[inputs[i]] = 0;
		fvalue2[inputs[i]] = 0;
		break;
	    case '1':
		fvalue1[inputs[i]] = ALLONES;
		fvalue2[inputs[i]] = ALLONES;
		break;
	    case 'x':
	    case 'X':
		fvalue1[inputs[i]] = 0;
		fvalue2[inputs[i]] = ALLONES;
		break;
	    default:
		fprintf(stderr, "%c: error in the input vector.\n", vec[i]);
		exit(-1);
	  }	// switch

	  // different from previous time frame, place in wheel
	  for (j=0; j<fanout[inputs[i]]; j++)
	  {
	    successor = fnlist[inputs[i]][j];
	    if (sched[successor] == 0)
	    {
	    	insertEvent(levelNum[successor], successor);
		sched[successor] = 1;
	    }
	  }
	}	// if ((origBit...)
    }	// for (i...)
}

////////////////////////////////////////////////////////////////////////
// observeOutputs()
//	This function prints the outputs of the fault-free circuit.
////////////////////////////////////////////////////////////////////////

void gateLevelCkt::observeOutputs()
{
   int i;
    //printcommonsimulated(input);
   //":?"
    for (i=0; i<numout; i++)
    {
	if (value1[outputs[i]] && value2[outputs[i]])
		cout << "Gate " <<i <<" output is 1 " <<endl;
	else if ((value1[outputs[i]] == 0) && (value2[outputs[i]] == 0))
		cout << "Gate " <<i <<" output is 0 " <<endl;
	else
		cout << "Gate " <<i <<" output is X " <<endl;
		//printf("X");
    }



    int j;
        //printcommonsimulated(input);
       //":?"
        for (j=0; j<numout; j++)
        {
    	if (fvalue1[outputs[j]] && fvalue2[outputs[j]])
    	    //printf("Faulty Gate %d output is 1 \n",j);
    		cout << "Faulty Gate " <<j <<" output is 1" <<endl;
    	else if ((fvalue1[outputs[j]] == 0) && (fvalue2[outputs[j]] == 0))
    		cout << "Faulty Gate " <<j <<" output is 0" <<endl;
    		//printf("Faulty Gate %d output is 0 \n",j);
    	else
    		cout << "Faulty Gate " <<j <<" output is X" <<endl;
        }

 /* for (i=0; i<numff; i++)
    {
	if (value1[ff_list[i]] && value2[ff_list[i]])
	    printf("1");
	else if ((value1[ff_list[i]] == 0) && (value2[ff_list[i]] == 0))
	    printf("0");
	else
	    printf("X");
    }*/

    //printf("\n");
}


// main()
int main(int argc, char *argv[])
{
    FILE *vecFile;
    char cktName[256], vecName[256];
    int totalNumVec, vecWidth, i;
    int nameIndex;
    int swap = 0;
    //float ut, st;
    //struct tms t_buf;

    if ((argc != 4) && (argc != 5))
    {
	fprintf(stderr, "Usage: %s [-io] <ckt>\n", argv[0]);
	fprintf(stderr, "The -i option is to begin the circuit in a state in *.initState.\n");
	fprintf(stderr, "and -o option is to OBSERVE fault-free outputs and FF's.\n");
	fprintf(stderr, " Example: %s s27\n", argv[0]);
	fprintf(stderr, " Example2: %s -o s27\n", argv[0]);
	fprintf(stderr, " Example3: %s -io s27\n", argv[0]);
	exit(-1);
    }

    if (argc == 5)
    {
	i = 1;
	nameIndex = 2;
	while (argv[1][i] != EOS)
	{
	    switch (argv[1][i])
	    {
		case 'o':
		    OBSERVE = 1;
		    break;
		case 'i':
		    INIT0 = 1;
		    break;
		default:
	    	    fprintf(stderr, "Invalid option: %s\n", argv[1]);
	    	    fprintf(stderr, "Usage: %s [-io] <ckt> <type>\n", argv[0]);
	    	    fprintf(stderr, "The -i option is to begin the circuit in a state in *.initState.\n");
	    	    fprintf(stderr, "and o option is to OBSERVE fault-free outputs.\n");
		    exit(-1);
		    break;
	    } 	// switch
	    i++;
	}	// while
    }
    else	// no option
    {
	nameIndex = 1;
	OBSERVE = 0;
	INIT0 = 0;
    }

    strcpy(cktName, argv[nameIndex]);
    strcpy(vecName, argv[nameIndex]);
    strcat(vecName, ".vec");

    circuit = new gateLevelCkt(cktName);

    //vecFile = fopen(vecName, "r");
    if (vecFile == NULL)
    {
	fprintf(stderr, "Can't open %s\n", vecName);
	exit(-1);
    }
    //fscanf(vecFile, "%d", &vecWidth);
    /*circuit->setTieEvents();
    totalNumVec = logicSimFromFile(vecFile, vecWidth,swap);
    fclose(vecFile);*/
    int faultygate = atoi(argv[nameIndex+1]);
    int gatetypereplaced = atoi(argv[nameIndex+2]);
    //cout <<"Enter the number of faulty gate" <<endl;
   // cin  >> faultygate;
    //cout << "Enter the gate type to be replaced with" << endl;
    //cin  >> gatetypereplaced;

    if(circuit->validateFaultyGate(faultygate) && circuit ->validateGateTypeReplaced(faultygate,gatetypereplaced))
    {
    	circuit->insertFault(faultygate,gatetypereplaced);
    }
    else
    {
    	exit(0);
    }
    //fscanf(vecFile, "%d", &vecWidth);
      //  circuit->setTieEvents();
       // totalNumVec = logicSimFromFile(vecFile, vecWidth,swap);
        //fclose(vecFile);
    circuit->podem();
    return 1;
}
