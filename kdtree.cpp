#ifndef KD
#define KD
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "RRT.cpp"
#include <limits>
#include <ctime>
#include <string>
#define points_in_cell 7
using namespace std;

class less_than_key
{
    public:
    static int key;
    template<typename T>
    inline bool operator() (const T &node1, const T &node2)

    {
        return (node1->getConfig()[key] < node2->getConfig()[key]);
    }

};

template <class T>  class node
{
public:

vector <T> _data;
node<T> *left,*right;
void add(T ele)
{
_data.push_back(ele);
}


vector <T> getData()
{
return _data;
}
node()
{
left = right = NULL;
}
node(vector <T> inp)
{
_data = inp;
left = right = NULL;
}
node(vector <T> *inp)
{
_data = *inp;
left = right = NULL;
//cout<<"overrrrrrrrrrrrr";
//cout<<this->getData()[0]->getConfig()[0];
}
node(NodeTree &inp)
{
_data = inp.getTree();
left = right = NULL;
}

~node()
{
_data.clear();
left=right=NULL;
}
void sortNode()
{
sort(_data.begin(), _data.end(),less_than_key());
}
};



int less_than_key::key = -1;

float sum(vector<float> &ele)
{
float s=0;
for (unsigned int i=0;i<ele.size();i++)
s+=ele[i];
return s;
}


vector<float> defV(7,0);








template <class T> class kdtree
{
public:
node<T> *root;
unsigned int totalSize;

void formTree(node<T> *tempRoot = NULL,int dim=0)
        {

        if(tempRoot == NULL) tempRoot = root;
        if(tempRoot->getData().size() < points_in_cell) return;
        less_than_key::key = dim;
        ++dim %= lower.size();
        tempRoot->sortNode();
        tempRoot->left = leftHalf(tempRoot);
        tempRoot->right = rightHalf(tempRoot);
        formTree(tempRoot->left,dim);
        formTree(tempRoot->right,dim);
        }
        //display(sortedTree[2]);

node<T>* leftHalf(node<T> *tempRoot =NULL)
    {
    if(tempRoot == NULL) {cout<<"Empty arguement can't find left half"<<endl;return NULL;}
    return new node<T>(new vector<T>(&(tempRoot->_data[0]),&(tempRoot->_data[tempRoot->getData().size()/2])));// If you dont want to wast memory even for pointers just save the median of the data here
    }

node<T>* rightHalf(node<T> *tempRoot =NULL)
    {
    if(tempRoot == NULL) {cout<<"Empty arguement can't find right half"<<endl;return NULL;}
    return new node<T>(new vector<T>(&(tempRoot->_data[tempRoot->getData().size()/2]), &(tempRoot->_data[tempRoot->getData().size()])));// If you dont want to wast memory even for pointers just save the median of the data here
    }


void check(node<T> *tempRoot = NULL,int left = 0, int right=0)
    {
    if(tempRoot == NULL) tempRoot = root;
    cout<<"left count "<<left<<endl;
    cout<<"right count "<<right<<endl;
    if (!tempRoot->left && !tempRoot->right) cout<<"Leaf node data points are "<< tempRoot->getData().size()<<endl;
    else cout<<"data points are "<< tempRoot->getData().size()<<endl;
    if (tempRoot->left)
    check(tempRoot->left,++left,++right);
    if (tempRoot->right)
    check(tempRoot->right,++left,++right);
    }


void deleteTree(node<T> *tempRoot = NULL)
{
    if(tempRoot ==NULL) {tempRoot=root; if (!(tempRoot->left == NULL && tempRoot->right == NULL)) tempRoot -> _data.clear(); else return;}
    if( tempRoot->left == NULL && tempRoot->right == NULL) {root-> _data.insert(root -> _data.end(),tempRoot -> _data.begin(),tempRoot -> _data.end()); if(root!=tempRoot) delete tempRoot; return;}
    deleteTree (tempRoot->left);
    deleteTree (tempRoot->right);
    if (tempRoot!=root) delete tempRoot;
    else {tempRoot->left = NULL; tempRoot->right = NULL;}
}

void deleteAll(node<T> *tempRoot = NULL)
{
    if(tempRoot ==NULL) {tempRoot=root;}
    if( tempRoot->left == NULL && tempRoot->right == NULL)
        {if(root!=tempRoot) delete tempRoot; return;}
    deleteAll(tempRoot->left);
    deleteAll(tempRoot->right);
    if(root!=tempRoot) delete tempRoot;
    else {root->_data.clear();root->left=NULL;root->right=NULL;}

}

void reformTree()
{
deleteTree();
formTree();
}

void reformTree(NodeTree &inp)
{
deleteAll();
root = new node<T>(inp.getTree());
formTree();
}

T allLeafSearch(vector <double> &ele,node<T> *tempRoot = NULL)
{

    static T bestPoint = NULL;
    static float bestDist;
    if(tempRoot == NULL) {tempRoot = root; bestDist = numeric_limits<float>::max();bestPoint=NULL;}//cout<<"first"<<euclidean(ele,nearestNeighbor(tempRoot->getData(),ele)->getConfig(),1)<<endl;}
    if ( tempRoot->left ==NULL && tempRoot->right ==NULL)
    {
        T tempPoint = nearestNeighbor(tempRoot->_data,ele);

        float nn = euclidean(ele,tempPoint->getConfig(),1);
        //if(globalFlag)cout<<debug<<"mark "<<mark++<<endl;
        if (bestDist > nn)
        {
       //if(globalFlag)cout<<"here"<<endl;
        bestPoint = tempPoint;
        bestDist = nn;
        }
        if (tempRoot == root) return bestPoint;
        return NULL;

    }

        //debug.push_back('l');
        allLeafSearch(ele,tempRoot->left);
        //debug.erase( debug.size() - 1 );
        //debug.push_back('r');
        allLeafSearch(ele,tempRoot->right);

   if (tempRoot == root) return bestPoint;
  }


T NNS(vector <double> &ele,node<T> *tempRoot = NULL,int dim = 0, vector<float> bound = defV)
{
static int mark =0;
    static T bestPoint;
    static float bestDist;
    if(tempRoot == NULL) {tempRoot = root;bestDist = numeric_limits<float>::max();bestPoint=NULL;}//cout<<"first"<<euclidean(ele,nearestNeighbor(tempRoot->_data(),ele)->getConfig(),1)<<endl;}

    if ( tempRoot->left ==NULL && tempRoot->right ==NULL)
    {
        //countt+=tempRoot->_data.size();
        T tempPoint = nearestNeighbor(tempRoot->_data,ele);

        float nn = euclidean(ele,tempPoint->getConfig(),1);
       //if(globalFlag) cout<<debug<<"mark NNS "<<mark++<<endl;
        if (bestDist > nn)
        {
       //if(globalFlag) cout<<"here"<<endl;
        bestPoint = tempPoint;
        bestDist = nn;
        }
        if (tempRoot == root) return bestPoint;
        return NULL;
    }


    int callDim = (dim+1)% lower.size();
    float median = tempRoot->_data[tempRoot->_data.size()/2]->getConfig()[dim];



    if (ele[dim]<=median)
        {

        NNS(ele,tempRoot->left,callDim,bound);
        bound[dim] = (median - ele[dim])*(median - ele[dim])*metric_weight[dim];
        if (bestDist > sum(bound)) NNS(ele,tempRoot->right,callDim,bound);
        }
    else
        {

        NNS(ele,tempRoot->right,callDim,bound);


        bound[dim] = (median - ele[dim])*(median - ele[dim])*metric_weight[dim];
        if (bestDist > sum(bound)) NNS(ele,tempRoot->left,callDim,bound);
        }

    if (tempRoot == root) return bestPoint;
}


void addPoint(T ele, node<T> *tempRoot = NULL,int dim =0)
{

    if(tempRoot ==NULL) tempRoot=root;
    if( tempRoot->left == NULL && tempRoot->right == NULL)
        {
        tempRoot->add(ele); totalSize++;
        if(tempRoot->_data.size() > 2*points_in_cell) formTree(tempRoot,dim);
        return;
        }
    if(ele->getConfig()[dim] <= tempRoot->_data[tempRoot->_data.size()/2]->getConfig()[dim]) addPoint(ele, tempRoot->left,(dim+1) % lower.size());
    else addPoint(ele,tempRoot->right,(dim+1) % lower.size());
}
kdtree()
{
totalSize=0;
root=NULL;
}
kdtree(node<T> &inp)
{
totalSize=0;
root = &inp;
}
std::vector<RRTNode*> getRootData()
{
return root->getData();
}
};






#endif // KD

