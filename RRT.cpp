#ifndef RRT_C
#define RRT_C

#include "RRT.h"



std::vector<double> RRTNode::getConfig()
	{
		return _configuration;
	}

void RRTNode::setParent(RRTNode* p)
    {
    parent=p;
    }

RRTNode*  RRTNode::getParent()
    {
    return parent;
    }
RRTNode::RRTNode()
	{
		parent = NULL;
	}
RRTNode:: ~RRTNode()
	{
        _configuration.clear();
		parent = NULL;
	}
RRTNode::RRTNode(std::vector<double> ele)
	{
		_configuration = ele;
		parent = NULL;
	}

RRTNode::RRTNode(std::vector<double> ele,RRTNode* p)
    {
		_configuration = ele;
		parent = p;
    }

void NodeTree::add(RRTNode &ele)
	{
		_nodes.push_back(&ele);
	}
void NodeTree::add(RRTNode* ele)
	{
		_nodes.push_back(ele);
	}
void NodeTree::delNode(int ele)
	{
		_nodes.erase (_nodes.begin()+ele);
	}

void NodeTree::delNode(RRTNode &ele)
	{
	 for (unsigned int i=0; i< _nodes.size();i++)
		if (_nodes[i] == &ele)
		{
			_nodes.erase (_nodes.begin()+i);
			break;
		}
	}

void NodeTree::delBranch(RRTNode *start,RRTNode *endNode) //deletes including start excluding end
    {

    if (start == endNode ) return ;
    delBranch(start->getParent(),endNode);
    delete start;

    }


std::vector< std::vector<double> > NodeTree::getNodes()
{
	std::vector< std::vector<double> > ret;
	for (unsigned int i=0; i <_nodes.size(); i++)
	{
	ret.push_back(_nodes[i]->getConfig());
	}
	return ret;
}


std::vector<RRTNode*> NodeTree::getTree()
{
	return _nodes;
}


NodeTree::NodeTree()
{}
NodeTree::NodeTree(RRTNode &ele)
{
_nodes.push_back(&ele);
}


std::vector<RRTNode*> NodeTree::getPathToStart(RRTNode* ele)
{
std::vector<RRTNode*> path;
RRTNode* par = ele;
while(par!=NULL)
{
path.push_back(par);
par = par->getParent();
}
return path;
}



void NodeTree::display()
{

            for (unsigned int i=0;i<_nodes.size();i++)
            {
            cout<<'(';
            for(unsigned int j=0;j< (_nodes[i]->getConfig().size());j++)
            cout<< _nodes[i]->getConfig()[j]<<',';
            cout<<')';
            }
            cout<<endl;
}





    bool inRegion(vector<double> inp)
    {
    for (unsigned int i=0;i<inp.size();i++)
        if (!(inp[i]<=upper[i] && inp[i]>=lower[i]))
        return 0;
    return 1;
    }

    void display(vector<float> inp)
            {
            cout<<'(';
            for (unsigned int i=0;i<inp.size();i++)
            cout<< inp[i]<<',';
            cout<<')'<<endl;

            }

    void display(vector<dReal> inp)
            {
            cout<<'(';
            for (unsigned int i=0;i<inp.size();i++)
            cout<< inp[i]<<',';
            cout<<')'<<endl;
            }

    vector<double> randomSample()
            {
            vector<double> ret;

            for (unsigned int j = 0;j<lower.size();j++)
                ret.push_back(  ((float)rand()/RAND_MAX) * (upper[j]-lower[j]) +lower[j]);
            return ret;
            }

    vector<double> subtractMul(vector<double> a, vector<double> &b, float scale = 1)
            {
                  for (unsigned int j = 0;j<a.size();j++)
                  a[j] = scale*(a[j]-b[j]);
                  return a;
            }

    vector<double> add(vector<double> a,vector<double> &b)
            {
                  for (unsigned int j = 0;j<a.size();j++)
                   a[j] = (a[j]+b[j]);
                  return a;
            }
    float norm(vector<double> a)
            {
                float s =0;
                for (unsigned int j = 0;j<a.size();j++)
                s += a[j]*a[j];
                return sqrt(s);
            }


    float euclidean (vector<double> a,vector<double> b,bool flag = 0)
            {

                float s =0;

                for (unsigned int j = 0;j<a.size();j++)
                if (flag) s += metric_weight[j]*(a[j]-b[j]) * (a[j]-b[j]);
                else s += (a[j]-b[j]) * (a[j]-b[j]);
                if (flag) return s;
                return sqrt(s);
            }

    float squaredEuclidean (vector<double> a,vector<double> b)
            {

                float s =0;
                for (unsigned int j = 0;j<a.size();j++)
                s += (a[j]-b[j]) * (a[j]-b[j]);
                return s;
            }


    RRTNode* nearestNeighbor(std::vector<RRTNode*> tree, vector<double> inputConfig)

        {
        float minDist = -1;
        unsigned int nodeIndex;
        float dist;
        for (unsigned int i=0; i<tree.size();i++)
        {
        dist = euclidean(tree[i]->getConfig(),inputConfig,1);
        if (minDist > dist || minDist == -1)
        {
        minDist = dist;
        nodeIndex = i;
        }
        }
        return tree[nodeIndex];
        }

void trimToRegion(vector <double> &ele, double tol= 0.00001)
        {
        for (unsigned int i =0; i<ele.size();i++)
            if (ele[i]<=lower[i]) ele[i] = lower[i]+tol;
            else if(ele[i]>=upper[i]) ele[i] = upper[i]-tol;
        }
template <typename T>
void swapNo(T &s1,T &s2)
    {
    T temp;

    temp = s1;
    s1 = s2;
    s2 = temp;
    }
void pathAssign(vector<RRTNode*> &path, RRTNode *temp)
        {
        if (temp==NULL) return;
        pathAssign(path, temp->getParent());
        path.push_back(temp);
        }

void shortcutSmooth(OpenRAVE::EnvironmentBasePtr env, vector<RRTNode*> &path,float step=0.3,unsigned int iters=200)
        {

            vector<RobotBasePtr> vbodies;
            env->GetRobots(vbodies);
            RobotBasePtr robot;
            robot = vbodies[0];
            vector<RRTNode*> tempPath;
            unsigned int length = path.size();
            vector<double> direction,onPath; unsigned int pos1,pos2,tempLength=0;
            RRTNode *temp,*top = path[path.size()-1];
            RRTNode *node1,*node2,*start;
            NodeTree tree;

            for (unsigned int i=0;i<iters;i++)
            {

            do
            {
            pos1 = rand()%length;pos2 = rand()%length;
            }while(pos1==pos2);
            if (pos2<pos1) swapNo(pos1,pos2);

            temp = top;
            for(unsigned int k=0;k<= pos2 ; (k++,temp = temp->getParent()))
                {
                //cout<<"size"<<length -k<<endl;
                if (k==pos2) start=node1=temp;
                else if (k==pos1) node2=temp;

                }



            onPath =node1->getConfig();
            direction = subtractMul(node2->getConfig(),onPath,(float)step/euclidean(node2->getConfig(),node1->getConfig()));
            tempLength=0;


            while(node1&&node2&& !(euclidean(node2->getConfig(),node1->getConfig())<step))
            {
                    node1 = new RRTNode(add(node1->getConfig(),direction),node1);
                    tempLength++;
                    if (!inRegion(node1->getConfig())) {tree.delBranch(node1,start);break;}
                    robot->SetActiveDOFValues(node1->getConfig());
                    if ( env->CheckCollision(robot)||robot->CheckSelfCollision()) {tree.delBranch(node1,start);break;}


                    if(euclidean(node2->getConfig(),node1->getConfig()) < step)
                        {

                            tree.delBranch(node2->getParent(),start);
                            length -= ((pos2-pos1-1)-tempLength);
                            node2->setParent(node1);

                            break;
                        }

            }


            }

        path.clear();
        pathAssign(path,top);

        }



std::vector< RRTNode* > RRTplanner(OpenRAVE::EnvironmentBasePtr env, std::vector<double> goalConfig,float step = 0.3,float goalBias = 0.1)
{

cout<<"Planning using unidirectional RRT"<<endl;
        vector<RobotBasePtr> vbodies;
        env->GetRobots(vbodies);
        RobotBasePtr robot;
        robot = vbodies[0];
        vector<double> start;

        robot->GetActiveDOFValues(start);



        //Get DOF limist to limit sample space
        robot->GetActiveDOFLimits(lower,upper);
        lower[4] = lower[6] = -M_PI;
        upper[4] = upper[6] = M_PI;

        trimToRegion(start);

        NodeTree RRTTree(*(new RRTNode(start)));
        //randomseed time


        //int seed = time(NULL);

        //srand(time(NULL));
        //cout<<"SEED "<<seed<<endl;
        //good seeds: 1457835657,1457835466,1457835500

        //choose step
        float tol=step;

        int baseThreshold = 2000;
        int bigThreshold = 10000;

        //choose connect/extend
        bool connect = 1;
        vector<double> q_s,onPath,direction;
        RRTNode *nearest = new RRTNode(start),*tem;
        bool goalFlag =0;


        node<RRTNode*> kd(RRTTree.getTree());
        kdtree<RRTNode*> kdt(kd);
        kdt.formTree();
        bool myFlag=1;


        trimToRegion(goalConfig);



        while (goalFlag == 0 || (squaredEuclidean(nearest->getConfig(),goalConfig)>(tol*tol)))
        {
        goalFlag =0;
        if (((float)rand()/(float)RAND_MAX) < goalBias )
        {
        q_s = goalConfig;
        goalFlag =1;
        }
        else q_s = randomSample();


        nearest = kdt.NNS(q_s);

        onPath = nearest->getConfig();
        direction = subtractMul(q_s,onPath,(float)step/euclidean(onPath, q_s));

        //if (goalFlag) cout<<euclidean(nearest->getConfig(),goalConfig,1)<<" "<<squaredEuclidean(nearest -> getConfig(), q_s)<<" "<<kdt.totalSize  <<" "<< (squaredEuclidean(nearest->getConfig(),goalConfig)>(tol*tol))<<" "<<tol<<" "<<goalBias<<endl;

        if (myFlag && kdt.totalSize > baseThreshold)
        {
        myFlag = 0;
        kdt.reformTree();
        }
        if (kdt.totalSize > bigThreshold)
        {
        cout<<"reforming tree..."<<endl;

         bigThreshold = 2*kdt.totalSize;
        kdt.reformTree();
        cout<<"Size of the tree = "<<kdt.getRootData().size()<<endl;
        cout<<"nearest neighbor's distance to goal = "<<euclidean(kdt.NNS(goalConfig)->getConfig(),goalConfig)<<endl;

        }

        do
        {

        nearest = new RRTNode(add(nearest->getConfig(),direction),nearest);

        if (!inRegion(nearest->getConfig())) {delete nearest; break;}
        robot->SetActiveDOFValues(nearest->getConfig());
        if (env->CheckCollision(robot)||robot->CheckSelfCollision()) {goalFlag =0;delete nearest;break;}
        kdt.addPoint(nearest);
        if (squaredEuclidean(nearest -> getConfig(), q_s)<(tol*tol)) {robot->SetActiveDOFValues(q_s);if (!env->CheckCollision(robot) && !robot->CheckSelfCollision()) kdt.addPoint(new RRTNode(q_s,nearest));break;}
        }while(connect);
        }

         nearest = kdt.NNS(goalConfig);
         vector<RRTNode*> path = RRTTree.getPathToStart(nearest);
         reverse(path.begin(),path.end());
         //cout<<"Path length returned by RRT Planner = "<<path.size()<<endl;
         return path;

}

bool solved=0;
RRTNode* growBranch(OpenRAVE::EnvironmentBasePtr env, kdtree<RRTNode*> *tree,vector<double> &q_s,float &step, bool connect = 1)

{


        static vector<RobotBasePtr> vbodies;
        if(vbodies.size()<2) env->GetRobots(vbodies);
        static RobotBasePtr robot=vbodies[0];


        static vector<double> onPath;

        RRTNode *pnearest,*nearest  = tree->NNS(q_s);
        onPath = nearest->getConfig();
        vector<double> direction = subtractMul(q_s,onPath,(float)step/euclidean(onPath, q_s));

        do
        {
        pnearest=nearest;
        nearest = new RRTNode(add(nearest->getConfig(),direction),nearest);

        if (!inRegion(nearest->getConfig())) {delete nearest;nearest =pnearest;break;}
        robot->SetActiveDOFValues(nearest->getConfig());
        if (  env->CheckCollision(robot)||robot->CheckSelfCollision()) { delete nearest;nearest =pnearest;break;}
        tree->addPoint(nearest);
        if (squaredEuclidean(nearest -> getConfig(), q_s)< (step*step)) {solved=1;break;}
        }while(connect);

        return nearest;
}


std::vector< RRTNode* > RRTbiplanner(OpenRAVE::EnvironmentBasePtr env, vector<double> goalConfig,float step = 0.3,float goalBias = 0.1)
{
        cout<<"Planning using bidirectional RRT"<<endl;
        vector<RobotBasePtr> vbodies;


        env->GetRobots(vbodies);
        RobotBasePtr robot = vbodies[0];


        vector<RRTNode*> path;

        robot->GetActiveDOFLimits(lower,upper);
        lower[4] = lower[6] = -M_PI;
        upper[4] = upper[6] = M_PI;








        vector<dReal> start;

        robot->GetActiveDOFValues(start);
        trimToRegion(start);
        trimToRegion(goalConfig);
        NodeTree RRTTree1(*(new RRTNode(start)));
        NodeTree RRTTree2(*(new RRTNode(goalConfig)));



        //srand (1457835657);
        //srand(time(NULL));
        //cout<<"SEED "<<seed<<endl;
        //good seeds: 1457835657,1457835466,1457835500

        //choose step
        float tol=step;

        int baseThreshold = 2000;
        int bigThreshold1 = 10000,bigThreshold2 = 10000;

        //choose connect/extend
        bool connect = 1;
        vector<double> q_s,targetConfig;




        node<RRTNode*> kd1(RRTTree1.getTree()),kd2(RRTTree2.getTree());
        kdtree<RRTNode*> kdt1(kd1),kdt2(kd2);
        kdt1.formTree(),kdt2.formTree();

        kdtree<RRTNode*> *tree1,*tree2;
        RRTNode *target,*solPath,*temp;


        bool myFlag2,myFlag1=1;
        int resetCounts1,resetCounts2 = 1;

        bool swapTrees = 1;

        while (1)
        {

        swapTrees = !swapTrees;


        q_s = randomSample();


        if(swapTrees) {tree1 = &kdt1;tree2 = &kdt2;}
        else  {tree1 = &kdt2;tree2 = &kdt1;}




        if (myFlag1 && kdt1.totalSize > baseThreshold)
        {
        myFlag1 = 0;
        kdt1.reformTree();
        }

        if (myFlag2 && kdt2.totalSize > baseThreshold)
        {
        myFlag2 = 0;
        kdt2.reformTree();
        }

        if (kdt1.totalSize > bigThreshold1)
        {
        cout<<"reforming tree1... "<<endl;

        bigThreshold1=2*kdt1.totalSize;
        kdt1.reformTree();
        cout<<"Size of tree1 = "<<kdt1.getRootData().size()<<endl;


        }



        if (kdt2.totalSize > bigThreshold2)
        {
        cout<<"reforming tree2..."<<endl;

        bigThreshold2=2*kdt2.totalSize;
        kdt2.reformTree();
        cout<<"Size of the tree2 = "<<kdt2.getRootData().size()<<endl;

        }


        target = growBranch(env,tree1,q_s,step);
        solved = 0;
        targetConfig = target->getConfig();
        solPath = growBranch(env,tree2, targetConfig ,step);


        if (solved) {
        if(!swapTrees){temp = target;target = solPath;solPath=temp;}
        pathAssign(path,target);
        for(temp = solPath;temp!=NULL;)
        {
        target=temp->getParent();

        temp->setParent(path[path.size()-1]);
        path.push_back(temp);
        temp = target;
        }
        break;
        }
        }

         return path;

}

#endif
