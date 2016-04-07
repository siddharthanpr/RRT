#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>
#include "RRT.cpp"
#include <math.h>
#define dimensions 7
using namespace std;
using namespace OpenRAVE;




class RRTmodule : public ModuleBase
{
public:
    std::vector< dReal > lower;
    std::vector< dReal > upper;
    RRTmodule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("RRT",boost::bind(&RRTmodule::MyCommand,this,_1,_2),
                        "Input format: RRT goal %f,%f,%f,%f,%f,%f,%f; goalBias 0.1; step .2; metricWeight 900,800,60,25,0,13,0; shortCutStep 0.15; biRRTFlag 0;");
        RegisterCommand("shortCutSmooth",boost::bind(&RRTmodule::SCS,this,_1,_2),
                        "This is an example command");
    }
    virtual ~RRTmodule() {}


    bool SCS(std::ostream& sout, std::istream& sinput)
    {

    std::string input = "default ";
    char cInput ='0';
    float fInput = 0;
    RRTNode* temp=NULL;
    vector<double> config;
    vector <RRTNode*> path;
    while (cInput!=';')
    {
    config.clear();
    for (unsigned int i = 0;i<dimensions;i++)
    {
        sinput>>fInput;
        config.push_back(fInput);
        sinput>>cInput;

    }
    temp=new RRTNode(config,temp);
    path.push_back(temp);
    }

    float shortStep;
    sinput>>shortStep;
    sinput>>cInput;
    float shortIters;
    sinput>>shortIters;

    shortcutSmooth(GetEnv(),path,shortStep);
    //cout<<"SIZEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE"<<path.size()<<endl;
    for(unsigned int i=0;i<path.size();i++)
         {
         for (unsigned int j=0;j<path[0]->getConfig().size();j++)
             {
             sout<< path[i]->getConfig()[j];
             if (j !=path[0]->getConfig().size()-1) sout<<",";
             }
             if (i !=path.size()-1) sout<<endl;
         }

         return true;
    }


    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
    //srand(-1);
        //srand (1457835657);
        srand(time(NULL));
        std::string input;
        vector<double> goalConfig;
        char temp ='1';
        sinput >> input;
        float q;
        if (input == "goal")
        while(temp!= ';')
            {
                sinput >> q;
                goalConfig.push_back(q);
                sinput >> temp;
            }


        sinput >> input;
        float goalBias = 0.2;
        temp ='1';
        if (input == "goalBias")
            sinput >> goalBias;
            sinput >> temp;




        sinput >> input;
        float step =0.3;
        temp ='1';
        if (input == "step")
            sinput >> step;
            sinput >> temp;


        sinput >> input;

        if(input=="metricWeight")
        for(unsigned int i=0; i<goalConfig.size();i++)
            {

                sinput >> q;
                metric_weight[i]=q;
                sinput >> temp;
            }

        sinput >> input;

        float shortStep =0.15;
        if(input=="shortCutStep")
                    sinput >> shortStep;
                    sinput >> temp;

        float shortIters =200;
        sinput >> input;
        if(input=="shortIters")
                    sinput >> shortIters;
                    sinput >> temp;



        sinput >> input;

        bool biFlag=0;
        if(input=="biRRTFlag")
                    sinput >> biFlag;
                    sinput >> temp;


     vector<RRTNode*> path;
    if(!biFlag)
     path = RRTplanner(GetEnv(), goalConfig,step,goalBias);
    else path = RRTbiplanner(GetEnv(), goalConfig,step,goalBias);
     if(shortStep>0) shortcutSmooth(GetEnv(),path,shortStep,shortIters);


     for(unsigned int i=0;i<path.size();i++)
         {
         for (unsigned int j=0;j<path[0]->getConfig().size();j++)
             {
             sout<< path[i]->getConfig()[j];
             if (j !=path[0]->getConfig().size()-1) sout<<",";
             }
             if (i !=path.size()-1) sout<<endl;
         }

        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtmodule" ) {
        return InterfaceBasePtr(new RRTmodule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("RRTmodule");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

