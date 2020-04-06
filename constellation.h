#include "ns3/core-module.h"
#include "sat-setup.h"
#include <float.h>

struct linkInfo
{
    std::vector<Vector> link_pos;
    std::vector<uint32_t> link_dex;
    std::vector<double> link_dist;
};

class Constellation
{
    public:
        Constellation (double c_alt, double c_inc, uint32_t c_nPlane, uint32_t c_nSat):
                       alt (c_alt), inc (c_inc), nPlane (c_nPlane), nSat (c_nSat) { SetSatellite(); }
        
        void SetSatPos (NodeContainer* satPos);
        void SetISL ();

        // std::vector<Vector> get_pos () { return satPosition; }

        void FifthLink ();
        void CreateLink();

        linkInfo get_firstISL () { return firstISL; }
        linkInfo get_secondISL () { return secondISL; }
        linkInfo get_thirdISL () { return thirdISL; }
        linkInfo get_fourthISL () { return fourthISL; }
        linkInfo get_fifthISL () { return fifthISL; }

        void Routing(uint32_t source, uint32_t target);

    protected:
        double alt;                 // constellation altitude
        double inc;                 // constellation inclination
        uint32_t nPlane;            // total planes
        uint32_t nSat;              // total satellites on each plane
        std::vector<OrbitalSat> orbitalElements;
        SatPosition polar;
        SatGeometry geometry;

        linkInfo firstISL;
        linkInfo secondISL;
        linkInfo thirdISL;
        linkInfo fourthISL;
        linkInfo fifthISL;

        linkInfo Dijkstra_prevNode;              // Store Dijkstra preceeding node info

    private:
        void SetSatellite ();
        bool link_init = 0;
        std::vector<uint32_t> fifth_secLinkDex;
        std::vector<Vector> satPosition;

        // Dijkstra algorithm for routing (only 4 ISL)
        uint32_t minimumDist (std::vector<double> dist, std::vector<bool> Dset);
        linkInfo dijkstra (std::vector<std::vector<double> > graph, uint32_t src, uint32_t tar);
};

void Constellation::SetSatellite ()
{
    double satAngle = 360.0/nSat;
    double planeAngle = 360.0/nPlane;
    double offsetMulti = f_Delta;
    double phaseOffset = offsetMulti/nPlane;
    for (uint32_t a = 0; a < nPlane; a++)
            for (uint32_t i = 0; i < nSat; i++)
                orbitalElements.push_back ({alt, inc, planeAngle*a, i*satAngle + a*phaseOffset*satAngle, a});
}

void Constellation::SetSatPos (NodeContainer* satPos)
{
    satPosition = polar.SatPos(orbitalElements);
    for (NodeContainer::Iterator iter = satPos->Begin(); iter != satPos->End(); ++iter)
    {

        Ptr<Node> tmp_node = (*iter);
        uint32_t sat = tmp_node->GetId();

        double longitude = satPosition[sat].x;
        double latitude = satPosition[sat].y;
        double altitude = satPosition[sat].z;

        satPos->Get(sat)->GetObject<MobilityModel>()->SetPosition(Vector(longitude, latitude, altitude));
    }

    // Update node position and link every 0.2s
    Simulator::Schedule (Seconds (0.2), &Constellation::SetSatPos, this, satPos);
}

void Constellation::SetISL ()
{
    CreateLink();           // Create the constant 4 intersatellite links
    // FifthLink();         // This 5th link is optional
    Simulator::Schedule (Seconds (0.2), &Constellation::SetISL, this);
}

void Constellation::CreateLink ()
{
    for (uint32_t a = 0; a < nPlane; a++)
        for (uint32_t i = 0; i < nSat; i++)
        {
            uint32_t link = a*nSat+i;

            // Intraplane link
            if (i != nSat - 1)
            {
                // first ISL is with the satellite in the very front
                firstISL.link_pos.push_back (satPosition[link + 1]);     
                firstISL.link_dex.push_back (link + 1);
                firstISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[link + 1]));
            }
            else
            {   
                // Last satellite's first ISL is with the first satellite in the same plane
                firstISL.link_pos.push_back (satPosition[link - i]);
                firstISL.link_dex.push_back (link - i);
                firstISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[link - i]));
            }
            
            if (i != 0)
            {
                // second ISL is with the satellite in the very back
                secondISL.link_pos.push_back (satPosition[link - 1]);    
                secondISL.link_dex.push_back (link - 1);
                secondISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[link - 1]));
            }
            else
            {   
                // First satellite's second ISL is with the last satellite in the same plane
                secondISL.link_pos.push_back (satPosition[link + nSat - 1]);
                secondISL.link_dex.push_back (link + nSat - 1);
                secondISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[link + nSat - 1]));
            }

            // Interplane link
            if (a == nPlane - 1)
                if (i >= nSat - f_Delta)
                {
                    // the last sats will connect with the first sats
                    thirdISL.link_pos.push_back (satPosition[i - nSat + f_Delta]);
                    thirdISL.link_dex.push_back (i - nSat + f_Delta);
                    thirdISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[i - nSat + f_Delta])); 
                }
                else
                {   
                    // sats in last plane will connect with the sat + f_Delta in the first plane
                    thirdISL.link_pos.push_back (satPosition[i + f_Delta]);
                    thirdISL.link_dex.push_back (i + f_Delta);
                    thirdISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[i + f_Delta]));
                }
            else
            {
                // third ISL is with the satellite in the bigger adjacent plane
                thirdISL.link_pos.push_back (satPosition[link + nSat]);
                thirdISL.link_dex.push_back (link + nSat);
                thirdISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[link + nSat]));
            }
            
            if (a == 0)
                if ( i < f_Delta)
                {
                    fourthISL.link_pos.push_back (satPosition[nPlane*nSat - f_Delta + i]);
                    fourthISL.link_dex.push_back (nPlane*nSat - f_Delta + i);
                    fourthISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[nPlane*nSat - f_Delta + i]));
                }
                else
                {
                    fourthISL.link_pos.push_back (satPosition[i - f_Delta]);
                    fourthISL.link_dex.push_back (i - f_Delta);
                    fourthISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[i - f_Delta]));
                }
            else
            {   
                // fourth ISL is with the satellite in the smaller adjacent plane
                fourthISL.link_pos.push_back (satPosition[link - nSat]);
                fourthISL.link_dex.push_back (link - nSat);
                fourthISL.link_dist.push_back (geometry.distance_geo(satPosition[link], satPosition[link - nSat]));
            }
        }
    // std::cout << "Khoang cach: " << geometry.distance_geo(satPosition[0], satPosition[1]) << std::endl;    
}

void Constellation::FifthLink ()
{
    for (uint32_t t = 0; t < nPlane*nSat; t++)
    {
        bool flag = 0;
        double minDist[2];              // 2 nearest satellites
        double tempDist;
        uint32_t satIndex[2];           // 2 nearest satellites
        uint32_t curPlane = t/nSat;
        for (uint32_t a = 0; a < nPlane; a++)
        {
            // Fifth link won't connect in the same plane
            if (a == curPlane)
                continue;
            for (uint32_t i = 0; i < nSat; i++)
            {       
                // The fifth link won't connect to satellite with same index in adjacent planes
                if (a*nSat + i == t + nSat || a*nSat + i == t - nSat)
                    continue;
                if (a == nPlane - 1)
                {
                    if (i >= nSat - f_Delta && t == i - nSat + f_Delta)
                        continue;
                    else if (t == i + f_Delta)
                        continue;
                }     
                                
        // Take 2 nearest satellite to establish 5th link
                if (flag == 0)
                {
                    flag = 1;
                    satIndex[0] = a*nSat + i;
                    minDist[0] = geometry.distance_geo(satPosition[t], satPosition[satIndex[0]]);

                    satIndex[1] = satIndex[0] + 1;
                    minDist[1] = geometry.distance_geo(satPosition[t], satPosition[satIndex[1]]);
                    continue;
                }
                tempDist = geometry.distance_geo(satPosition[t], satPosition[a*nSat + i]);
                if (minDist[0] > tempDist)
                {
                    satIndex[1] = satIndex[0];
                    minDist[1] = minDist[0];
                    satIndex[0] = a*nSat + i;
                    minDist[0] = tempDist;
                }
                else if (minDist[1] > tempDist)
                {
                    satIndex[1] = a*nSat + i;
                    minDist[1] = tempDist;
                }
            }
        }
        if (link_init == 0)
        {
            fifthISL.link_pos.push_back(satPosition[satIndex[0]]);
            fifthISL.link_dex.push_back(satIndex[0]);
            fifthISL.link_dist.push_back(geometry.distance_geo(satPosition[t], satPosition[satIndex[0]]));
            fifth_secLinkDex.push_back(satIndex[1]);
        }
        else
        {
            fifthISL.link_pos[t] = satPosition[satIndex[0]];
            fifthISL.link_dex[t] = satIndex[0];
            fifthISL.link_dist[t] = geometry.distance_geo(satPosition[t], satPosition[satIndex[0]]);
            fifth_secLinkDex[t] = satIndex[1];
        }                
    }

    // Establish connection only when 2 sat connect to each other
    // Assign itself if no connection
    for (uint32_t t = 0; t < nPlane * nSat; t++)
    {
        uint32_t tmp = fifthISL.link_dex[t];
        if (t != fifthISL.link_dex[tmp])
        {
            fifthISL.link_dex[t] = t;
            fifthISL.link_pos[t] = satPosition[t];
            fifthISL.link_dist[t] = DBL_MAX;
        }
    }

    // Establish 5th link taking the 2nd nearest sat
    for (uint32_t t = 0; t < nPlane * nSat; t++)
    {
        uint32_t tmp;
        if (t == fifthISL.link_dex[t])
        {
            tmp = fifth_secLinkDex[t];
            if (tmp == fifthISL.link_dex[tmp])
            {
                fifthISL.link_pos[t] = satPosition[tmp];
                fifthISL.link_dex[t] = tmp;
                fifthISL.link_dist[t] = geometry.distance_geo(satPosition[t], satPosition[tmp]);
                fifthISL.link_pos[tmp] = satPosition[t];
                fifthISL.link_dex[tmp] = t;
                fifthISL.link_dist[tmp] = geometry.distance_geo(satPosition[t], satPosition[tmp]);
            }
        }
    }
    link_init = 1;
}

void Constellation::Routing (uint32_t source, uint32_t target)
{
    uint32_t counter = 0;
    // 2D vector for storing distances of 4 ISL links
    std::vector<std::vector<double> > graph (nPlane*nSat, std::vector<double> (nPlane*nSat, 0));

    for (uint32_t i = 0; i < nPlane*nSat; i++)
    {
        uint32_t a, b, c, d, e;
        a = firstISL.link_dex[i];
        b = secondISL.link_dex[i];
        c = thirdISL.link_dex[i];
        d = fourthISL.link_dex[i];

        // Check if there are fifth links in the satellites
        if (fifthISL.link_dex.size() > 0)
            e = fifthISL.link_dex[i];

        // store 4 ISL links distance. The rest is 0
        graph[i][a] = firstISL.link_dist[i];
        graph[i][b] = secondISL.link_dist[i];
        graph[i][c] = thirdISL.link_dist[i];
        graph[i][d] = fourthISL.link_dist[i];

        // Check if there are fifth links in the satellites
        if (fifthISL.link_dex.size() > 0)
            graph[i][e] = fifthISL.link_dist[i];
    }
    
    Dijkstra_prevNode = dijkstra(graph, source, target);

    // Print on the screen the distance and number of satellites on the routing path.
    
    uint32_t u = target;
    while (u != source)
    {
        u = Dijkstra_prevNode.link_dex[u];
        counter++;
    }

    if (source != target)
        std::cout << "Routing from satellite " << target << " to satellite " << source << std::endl;

    for (uint32_t i = 0; i <= counter; i++)
    {
        if (i == 0)
            std::cout << "distance\t" << "target\t";
        else if (i == counter)
            std::cout << "source" << std::endl;
        else
            std::cout << "sat " << i << "\t";
    }
    
    u = target;
    if (u != source)
        std::cout << Dijkstra_prevNode.link_dist[u] << "\t\t" << target << "\t";

    while (u != source)
    {
        std::cout << Dijkstra_prevNode.link_dex[u] << "\t";
        u = Dijkstra_prevNode.link_dex[u];
    }
    std::cout << std::endl;
}

uint32_t Constellation::minimumDist (std::vector<double> dist, std::vector<bool> Dset)   /*A method to find the vertex with minimum distance which is not yet included in Dset*/
{
	double min = DBL_MAX;
    uint32_t index;                 /*initialize min with the maximum possible value as infinity does not exist */
	for (uint32_t v = 0; v < nPlane*nSat; v++)
	{
		if (Dset[v] == false && dist[v] <= min)      
		{
			min = dist[v];
			index = v;
		}
	}
	return index;
}

linkInfo Constellation::dijkstra(std::vector<std::vector<double> > graph, uint32_t src, uint32_t tar) /*Method to implement shortest path algorithm*/
{
    linkInfo preNode;                           
	std::vector<bool> Dset;
    
	for(uint32_t i = 0; i < nPlane*nSat; i++)                    /*Initialize diDset[stance of all the vertex to INFINITY and Dset as false*/  
	{
        Dset.push_back (false);
		preNode.link_dist.push_back (DBL_MAX);
        preNode.link_dex.push_back (nPlane*nSat);               // Initialize with an unlikely value	
	}
	preNode.link_dist[src] = 0;                                   /*Initialize the distance of the source vertec to zero*/
	for(uint32_t c = 0; c < nPlane*nSat; c++)                           
	{
		uint32_t u = minimumDist (preNode.link_dist, Dset);              /*u is any vertex that is not yet included in Dset and has minimum distance*/

        if (u == tar)                               // target node reached
            break;

		Dset[u] = true;                              /*If the vertex with minimum distance found include it to Dset*/ 
		for(uint32_t v = 0; v < nPlane*nSat; v++)                  
		/*Update dist[v] if not in Dset and their is a path from src to v through u that has distance minimum than current value of dist[v]*/
			if (!Dset[v] && graph[u][v] && preNode.link_dist[u] != DBL_MAX && preNode.link_dist[u] + graph[u][v] < preNode.link_dist[v])
			{
                preNode.link_dist[v] = preNode.link_dist[u] + graph[u][v];        // Update total node min distance from src
                preNode.link_dex[v] = u;                            // Update previous node for that node
            }
	}

	// std::cout << "Satellite\t\tDistance from source" << std::endl;
	// for(uint32_t i = 0; i < nPlane*nSat; i++)                       /*will print the vertex with their distance from the source to the console */
	// {
	// 	// uint32_t t = 0;
	// 	std::cout << i << "\t\t\t"<< preNode.link_dist[i] << std::endl;
	// }

    return preNode;
}