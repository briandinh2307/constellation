#include "constellation.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include <vector>

#define Starlink_alt 1150 // Polar satellite altitude
#define Starlink_inc 53 // Orbit inclination
#define Starlink_nPlane 32 // Number of planes
#define Starlink_nSat 50   // Number of satellites on each plane

using namespace ns3;

static void 
PositionCall (Ptr<const Node> node, uint32_t tmp_sat)
{
    Ptr<MobilityModel> tmp_pos = node->GetObject<MobilityModel>();
    uint32_t sat_th = node->GetId();
    uint32_t plane_th = sat_th / tmp_sat;

    Vector pos = tmp_pos->GetPosition();
    
    // time, sat_th, plane_th,  longitude, lattitude, altitude
    std::cout << "t = " << Simulator::Now().GetSeconds() << "s" << ", " << sat_th << ", "
              << plane_th << ", " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
}

// Check for fifth links
static void FifthLink(Constellation* netWork, uint32_t tmp_sat)
{   
    uint32_t count = 0;

    linkInfo satLink = netWork->get_fifthISL ();
    std::vector<Vector> link = satLink.link_pos;
    std::vector<uint32_t> dex = satLink.link_dex;
    
    std::cout << std::endl;
    std::cout << std::endl << "fifth link position" << std::endl;
    for (uint i = 0; i < link.size(); i++)
    {
        // source satellite
        // uint32_t curPlane = i/tmp_sat + 1;
        // uint32_t curSat = tmp_sat - (tmp_sat*curPlane - (i+1));
        // linked satellite
        // uint32_t plane = dex[i]/tmp_sat + 1;
        // uint32_t sat = tmp_sat - (tmp_sat*plane - (dex[i]+1));

        // time, source satellite, linked longitude, linked latitude, linked altitude, linked satellite
        // if (dex[i] != i)
        //     std::cout << "t = " << Simulator::Now().GetSeconds() << "s" << ", "  << curPlane << "/" << curSat << ", " 
        //               << link[i].x << ", " << link[i].y << ", " << link[i].z << ", " << plane << "/" << sat << std::endl;

        if (dex[i] != i)
            std::cout << "t = " << Simulator::Now().GetSeconds() << "s" << ", "  << i << ", " 
                      << link[i].x << ", " << link[i].y << ", " << link[i].z << ", " << dex[i] << std::endl;
        // print "NO" at the end if no 5th link connection
        else
        {
            // std::cout << "t = " << Simulator::Now().GetSeconds() << "s" << ", "  << curPlane << "/" << curSat << ", " 
            //           << link[i].x << ", " << link[i].y << ", " << link[i].z << ", " << plane << "/" << sat << "\t\t NO" << std::endl;

            std::cout << "t = " << Simulator::Now().GetSeconds() << "s" << ", "  << i << ", " 
                      << link[i].x << ", " << link[i].y << ", " << link[i].z << ", " << dex[i] << "\t\t NO" << std::endl;
            count++;
        }
    }
    std::cout << std::endl << "Number of no 5th link: " << count << std::endl;
}
//

// Routing check
static void RoutingCheck(Constellation* netWork, uint32_t sat_source, uint32_t sat_tar)
{
    netWork->Routing (sat_source, sat_tar);
}

int main (int argc, char *argv[])
{
    double alt = Starlink_alt;
    double inc = Starlink_inc;
    uint32_t nPlane = Starlink_nPlane;
    uint32_t nSat = Starlink_nSat;
    std::string animFile = "constellation.xml";
    double watchTime = 0.0;
    double simTime = 30.0;
    uint32_t source = 0;
    uint32_t target = 0;

    CommandLine cmd;
    cmd.AddValue ("alt", "Altitude of the satellites", alt);
    cmd.AddValue ("inc", "Inclination of the orbits", inc);
    cmd.AddValue ("nPlane", "Number of planes", nPlane);
    cmd.AddValue ("nSat", "Number of satellite on each plane", nSat);
    cmd.AddValue ("animFile", "File Name for Animation Output", animFile);
    cmd.AddValue ("watchTime", "Observe satellites position at 'watchTime'", watchTime);
    cmd.AddValue ("simTime", "Time of the simulation", simTime);
    cmd.AddValue ("source", "Source satellite for routing", source);
    cmd.AddValue ("target", "Target satellite for routing", target);
    cmd.Parse (argc, argv);

    alt = alt == 0.0 ? Starlink_alt : alt;
    inc = inc == 0.0 ? Starlink_inc : inc;
    nPlane = nPlane == 0 ? Starlink_nPlane : nPlane;
    nSat = nSat == 0 ? Starlink_nSat : nSat;
    simTime = simTime == 0 ? 30.0 : simTime;
    source = source >= nPlane*nSat ? 0 : source;
    target = target >= nPlane*nSat ? 0 : target;

    NodeContainer satellite;
    satellite.Create (nPlane * nSat);

    // Mobility install is needed for SetPostion
    MobilityHelper mobility;
    mobility.Install (satellite);

    // SatPosition satNetWork (alt, inc, nPlane, nSat);
    // SetSatPos (&satellite, &satNetWork);

    Constellation satNetWork (alt, inc, nPlane, nSat);
    satNetWork.SetSatPos (&satellite);
    satNetWork.SetISL ();
    //satNetWork.Routing ();

    // Create ISL
    // PointToPointHelper pointToPoint;
    // for (uint32_t i = 0; i < nPlane*nSat; i++)
    //     for (uint32_t j = i + 1; j < nPlane*nSat; j++)
    //         pointToPoint.Install(satellite.Get(i), satellite.Get(j));

    // for(uint32_t a = 0; a < nPlane; a++)
    //     for(uint32_t i = 0; i < nSat; i++)
    //     {
    //         uint32_t link = a*nSat + i;
    //         // Intraplane link
    //         // if(i == nSat - 1)
    //         //     pointToPoint.Install(satellite.Get(link), satellite.Get(a*nSat));
    //         // else
    //         //     pointToPoint.Install(satellite.Get(link), satellite.Get(link+1));
                
    //         // Interplane link
    //         if(a == nPlane - 1)
    //             if(i >= nSat - 5)
    //                 pointToPoint.Install(satellite.Get(link), satellite.Get(i-nSat+f_Delta));
    //             else
    //                 pointToPoint.Install(satellite.Get(link), satellite.Get(i+f_Delta));
    //         else
    //             pointToPoint.Install(satellite.Get(link), satellite.Get(link + nSat));
    //     }
    
    // // Check Net Card
    // satellite.Get(0)->GetObject<PointToPointChannel>();
    // int x = satellite.Get(0)->GetNDevices();
    // std::cout << "Device: " << x << std::endl;
    // //


    // Check for satellites position
    for (NodeContainer::Iterator iter = satellite.Begin(); iter != satellite.End(); ++iter)
    {
        Ptr<Node> tmp_node = (*iter);
        Simulator::Schedule (Seconds(watchTime), &PositionCall, tmp_node, nSat);
    }

    // Check for fifth links
    Simulator::Schedule (Seconds(watchTime), &FifthLink, &satNetWork, nSat);

    // Check for shortest path
    Simulator::Schedule (Seconds(watchTime), &RoutingCheck, &satNetWork, source, target);

    AnimationInterface anim(animFile);
    anim.SetBackgroundImage("1.gif", -180.0, -90.0, 0.75, 0.75, 0.5);

    for(uint32_t a = 0; a < nPlane; a++)
            for(uint32_t i = 0; i < nSat; i++)
            {
                // Plane 1
                if(a == 0)
                {
                    anim.UpdateNodeColor(a*nSat+i, 0, 0, 255);
                    anim.UpdateNodeSize(a*nSat+i, 4, 4);
                }
                
                // Plane 2
                // if(a == 1)
                // {
                //     anim.UpdateNodeColor(a*nSat+i, 0, 255, 0);
                //     anim.UpdateNodeSize(a*nSat+i, 4, 4);
                // }

                std::string planeNum = std::to_string(a+1);
                std::string satNum = std::to_string(i+1);
                std::string label = planeNum + "/" + satNum;
                anim.UpdateNodeDescription(a*nSat+i, label);
            }
    
    Simulator::Stop (Seconds (simTime));

    Simulator::Run ();

    Simulator::Destroy ();
    return 0;
}